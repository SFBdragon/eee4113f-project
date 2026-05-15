//! sender.rs — no_std, no alloc, no atomics
//!
//! Designed as a pure state machine.
//! The caller is responsible for mutual exclusion and I/O.

use crate::{CrcFn, LoRaAddr, rtt::RttEstimator, wifi::Mac};

use super::common::*;

/// This needs to exist as a static variable.
#[derive(Debug)]
pub struct Sender {
    pub state: SenderState,

    pub set_timer: extern "C" fn(u32),
    pub cancel_timer: extern "C" fn(),
    pub get_time_ms: extern "C" fn() -> u32,
    pub crc_fn: CrcFn,
}

#[derive(Debug)]
pub enum SenderState {
    None,
    Syn {
        syn_packet_buf: [u8; MAX_DATAGRAM],
        syn_sent: bool,
    },
    Connected {
        mac: Mac,
        conn: SenderConn,
    },
}

extern "C" fn default_set_timer(_: u32) {
    panic!()
}
extern "C" fn default_cancel_timer() {
    panic!()
}
extern "C" fn default_get_time() -> u32 {
    panic!()
}
extern "C" fn default_crc_fn(_: *const u8, _: usize) -> u16 {
    panic!()
}

impl Sender {
    const SYN_TIMEOUT_MS: u32 = 1000;

    pub const fn new() -> Self {
        Self {
            state: SenderState::None,
            set_timer: default_set_timer,
            cancel_timer: default_cancel_timer,
            get_time_ms: default_get_time,
            crc_fn: default_crc_fn,
        }
    }

    // All these APIs need to be trivially exposable over FFI.

    // Call `send_next` immediately after to send the SYN packet.
    pub fn connect(
        &mut self,
        controller_addr: LoRaAddr,
        set_timer: extern "C" fn(u32),
        cancel_timer: extern "C" fn(),
        get_time: extern "C" fn() -> u32,
        crc_fn: CrcFn,
    ) {
        self.set_timer = set_timer;
        self.cancel_timer = cancel_timer;
        self.get_time_ms = get_time;
        self.crc_fn = crc_fn;

        self.state = SenderState::Syn {
            syn_packet_buf: [0u8; MAX_DATAGRAM],
            syn_sent: false,
        };
        if let SenderState::Syn { syn_packet_buf, .. } = &mut self.state {
            let payload = controller_addr.to_le_bytes();
            let df = DataFrame {
                flags: FLAG_SYN,
                seq: 0,
                len: payload.len() as _,
                payload: &payload,
            };
            df.serialize(syn_packet_buf, self.crc_fn);
        }

        set_timer(Self::SYN_TIMEOUT_MS);
    }

    pub fn send_next(&mut self, mac: &mut Mac, data: &mut *const u8, len: &mut u16) -> bool {
        match &mut self.state {
            SenderState::None => false,
            SenderState::Syn {
                syn_packet_buf,
                syn_sent,
            } => {
                if !*syn_sent {
                    *data = syn_packet_buf.as_ptr();
                    *len = syn_packet_buf[DataFrame::LEN_OFFSET] as u16 + DG_OVERHEAD as u16;
                    *syn_sent = true;
                    *mac = Mac::bcast();
                    true
                } else {
                    false
                }
            }
            SenderState::Connected {
                mac: conn_mac,
                conn,
            } => {
                *mac = *conn_mac;
                conn.send_next(data, len)
            }
        }
    }

    pub fn available_payload_bytes(&self) -> u32 {
        match &self.state {
            SenderState::None => 0,
            SenderState::Syn { .. } => 0,
            SenderState::Connected { conn, .. } => conn.available_payload_bytes() as u32,
        }
    }

    pub fn push_message(&mut self, message: &[u8]) -> bool {
        match &mut self.state {
            SenderState::None => false,
            SenderState::Syn { .. } => false,
            SenderState::Connected { conn, .. } => conn.push_message(message),
        }
    }

    pub fn on_recv_ack(&mut self, mac: Mac, raw: &[u8]) -> bool {
        let mut try_send = false;

        if raw[0] & FLAG_ACK != 0 && raw[0] & FLAG_RST != 0 {
            (self.cancel_timer)();
            self.state = SenderState::None;
        }

        if let Some(next_state) = 'state: {
            match &mut self.state {
                SenderState::None => {}
                SenderState::Syn { syn_sent, .. } => {
                    if *syn_sent && raw[0] & FLAG_ACK != 0 && raw[0] & FLAG_SYN != 0 {
                        (self.cancel_timer)();
                        let conn = SenderConn::new(
                            self.set_timer,
                            self.cancel_timer,
                            self.get_time_ms,
                            self.crc_fn,
                        );

                        break 'state Some(SenderState::Connected { mac, conn });
                    }
                }
                SenderState::Connected { mac: _, conn } => {
                    try_send = conn.on_recv_ack(raw);
                }
            }

            None
        } {
            self.state = next_state;
        }

        try_send
    }

    pub fn on_timeout(&mut self) -> bool {
        match &mut self.state {
            SenderState::None => false,
            SenderState::Syn {
                syn_packet_buf: _,
                syn_sent,
            } => {
                *syn_sent = false;
                (self.set_timer)(Self::SYN_TIMEOUT_MS);
                true
            }
            SenderState::Connected { mac: _, conn } => conn.on_timeout(),
        }
    }
}

// ── Sender state machine ──────────────────────────────────────────────────────

/// It is assumed that SEQ_SPACE (256) is a multiple of SLOT_LEN for the rotates.
const SLOT_LEN: usize = 32;
type SlotBitMap = u32;

#[derive(Debug)]
pub struct SenderConn {
    pub slots: [[u8; MAX_DATAGRAM]; SLOT_LEN],
    pub occupied: SlotBitMap,
    pub retransmitted: SlotBitMap,
    pub sent_at_ms: [u32; SLOT_LEN],

    // Base is send_base.
    pub to_retransmit: SlotBitMap,

    /// The oldest unacknowledged slot. 0-SEQ_SPACE.
    pub send_base: u8,
    /// The next slot to send off. 0-SEQ_SPACE.
    pub seq_send: u8,
    /// The next slot to write to. 0-SEQ_SPACE.
    pub seq_next: u8,

    pub rtt: RttEstimator,
    pub aimd: WindowController,
    pub crc_fn: CrcFn,
    pub set_timer: extern "C" fn(u32),
    pub cancel_timer: extern "C" fn(),
    pub get_time_ms: extern "C" fn() -> u32,
}

impl SenderConn {
    pub const fn new(
        set_timer: extern "C" fn(u32),
        cancel_timer: extern "C" fn(),
        get_time_ms: extern "C" fn() -> u32,
        crc_fn: CrcFn,
    ) -> Self {
        Self {
            slots: [[0u8; MAX_DATAGRAM]; SLOT_LEN],
            occupied: 0,
            retransmitted: 0,
            sent_at_ms: [0; SLOT_LEN],

            to_retransmit: 0,

            send_base: 0,
            seq_send: 0,
            seq_next: 0,

            rtt: RttEstimator::wifi(),
            aimd: WindowController::new(),
            set_timer,
            cancel_timer,
            get_time_ms,
            crc_fn,
        }
    }

    #[inline]
    pub fn available_slots(&self) -> usize {
        SLOT_LEN - self.seq_next.wrapping_sub(self.send_base) as usize
    }

    #[inline]
    pub fn available_payload_bytes(&self) -> usize {
        self.available_slots() * MAX_PAYLOAD
    }

    /// How many sequence slots are currently in flight.
    #[inline]
    pub fn in_flight(&self) -> u8 {
        self.seq_send.wrapping_sub(self.send_base)
    }

    /// Returns true if there is room to send a new frame (not a retransmit).
    pub fn can_send_new(&self) -> bool {
        (self.in_flight() as usize) < self.aimd.window() as usize
    }

    /// Call if `push_message` or `on_rx_ack` return true (should send).
    /// Returns true if there's a frame to be sent: the frame pointer and len
    ///   have been written to `data` and `len` respectively.
    pub fn send_next(&mut self, data: &mut *const u8, len: &mut u16) -> bool {
        let retransmit_slots = self.to_retransmit & self.occupied;

        let seq = if retransmit_slots != 0 {
            self.slots[retransmit_slots.trailing_zeros() as usize][DataFrame::SEQ_OFFSET]
        } else if self.can_send_new() && self.seq_next != self.seq_send {
            self.seq_send
        } else {
            return false;
        };

        let slot = seq as usize % SLOT_LEN;
        *data = self.slots[slot].as_ptr();
        *len = self.slots[slot][DataFrame::LEN_OFFSET] as u16 + DG_OVERHEAD as u16;

        self.sent_at_ms[slot] = (self.get_time_ms)();
        self.to_retransmit &= !(1 << seq);

        let was_idle = self.in_flight() == 0;
        if was_idle {
            let rto = self.rtt.rto();
            (self.set_timer)(rto);
        }

        if retransmit_slots == 0 {
            self.seq_send = seq_add(self.seq_send, 1);
        }

        true
    }

    /// Push a message into the send queue.
    ///
    /// If this returns `true`, call `send_next` until that returns false to transmit.
    ///
    /// # Panics
    ///
    /// The message must fit in the queue.
    /// Panics if `message` is longer than `self.available_payload_bytes()`.
    pub fn push_message(&mut self, message: &[u8]) -> bool {
        if message.len() > self.available_payload_bytes() {
            panic!();
        }

        let mut b = 0;
        while b < message.len() {
            let last_in_message = b + MAX_PAYLOAD > message.len();
            let flags = if last_in_message { FLAG_END } else { 0 };
            let len = if last_in_message {
                message.len() - b
            } else {
                MAX_PAYLOAD
            };

            let dg = DataFrame {
                flags,
                seq: self.seq_next,
                len: len as u8,
                payload: &message[b..(b + len)],
            };

            let slot = self.seq_next as usize % SLOT_LEN;
            dg.serialize(&mut self.slots[slot], self.crc_fn);

            let bit = (1 as SlotBitMap) << slot;
            self.occupied |= bit;
            self.retransmitted &= !bit;

            b += len;
            self.seq_next = seq_add(self.seq_next, 1);
        }

        self.can_send_new()
    }

    /// Handle an ACK.
    /// Returns `true` if the caller should attempt to send frames.
    pub fn on_recv_ack(&mut self, raw: &[u8]) -> bool {
        let ack = match AckFrame::parse(raw, self.crc_fn) {
            Ok(a) => a,
            Err(_) => return false,
        };

        // Handling ack_base advance.

        // ack_base tells us the receiver has fully delivered everything before
        // it. Advance send_base to ack_base if it is ahead of us.
        let base_offset = ack.ack_base.wrapping_sub(self.send_base);
        let advanced = base_offset > 0 && base_offset <= self.in_flight();

        if advanced {
            // Mark all slots from send_base up to (but not including) ack_base
            // as ACKed, taking RTT samples where eligible.
            let now_ms = (self.get_time_ms)();

            // The frame at send_base should still be waiting there.
            // If not, the sender algo screwed up.
            // Crucially, it's the one we want to sample RTT against.
            //
            // If we didn't advance, the oldest frame wasn't ACK'd and must
            // be retransmitted, which means we mustn't sample with it.
            debug_assert!(1 << self.send_base & self.occupied != 0);

            // Karn's algorithm: don't sample retransmitted frames.
            if 1 << self.send_base & self.retransmitted == 0 {
                let slot = self.send_base as usize % SLOT_LEN;
                let rtt_sample = now_ms.wrapping_sub(self.sent_at_ms[slot]);
                self.rtt.update(rtt_sample);
            }

            // Now set all the packets between send_base and ack_base to unoccupied.
            let acked = (((1 as SlotBitMap) << base_offset) - 1)
                .rotate_left(self.send_base as u32 % SLOT_LEN as u32);
            self.occupied &= !acked;
            self.send_base = ack.ack_base;
        }

        // No indicated occupancies implies that there were no missing packets.
        // Thus bitmap==0 indicated a clean ACK.
        // self.send_base..self.seq_send remains unacknowledged.
        if ack.bitmap == 0 {
            // Clean ACK of more than one packet? Widen AIMD window.
            if advanced {
                self.aimd.on_clean_ack();
            }

            // All in-flight packets received? Cancel timer.
            if self.send_base == self.seq_send {
                (self.cancel_timer)();
                return false;
            }

            // Some packets still in flight. Oldest packet acknowledged? Reset RTO.
            if advanced {
                let rto = self.rtt.rto();
                (self.set_timer)(rto);
            }

            // Go check if a frame can be sent.
            return true;
        }

        // Handling unclean ACK.

        // First, handle all the ACK'd frames.
        // The bitmap is relative to ack_base (== receiver's recv_base).
        // Bit i means seq (ack_base + i) mod 256 was received.
        let acked = (ack.bitmap as SlotBitMap).rotate_left(ack.ack_base as u32 % SLOT_LEN as u32);
        // Ignore acks for unoccupied (already-acked) frames.
        let acked = acked & self.occupied;
        self.occupied &= !acked;

        // Assert that send_base is un-ACK'd. Should be the case.
        debug_assert!(1 << self.send_base & self.occupied != 0);

        // Should we flag all unacked frames before the most recently acked from for retransmission here?

        true
    }

    /// Call when the RTO timer fires. Flags frames to retransmit.
    /// If this returns `true`, call `send_next` until it returns false to retransmit frames.
    pub fn on_timeout(&mut self) -> bool {
        self.rtt.on_timeout();
        self.aimd.on_loss();

        let in_flight = self.in_flight() as usize;

        let mask = (((1 as SlotBitMap) << in_flight) - 1)
            .rotate_left(self.send_base as u32 % SLOT_LEN as u32);
        self.retransmitted |= mask & self.occupied;
        self.to_retransmit |= mask & self.occupied;

        self.to_retransmit & self.occupied != 0
    }
}

// ── AIMD window controller ────────────────────────────────────────────────────

#[derive(Debug)]
pub struct WindowController {
    pub cwnd: u8, // current window size [1, MAX_WINDOW]
    pub ssthresh: u8,
    /// Counts clean ACK rounds for additive increase.
    pub ack_rounds: u8,
}

impl WindowController {
    pub const fn new() -> Self {
        Self {
            cwnd: 1,
            ssthresh: MAX_WINDOW as u8,
            ack_rounds: 0,
        }
    }

    /// Call when a batch of new frames is ACKed without loss.
    pub fn on_clean_ack(&mut self) {
        if self.cwnd < self.ssthresh {
            // Slow start: double each round (clamp to ssthresh)
            self.cwnd = (self.cwnd.saturating_mul(2)).min(self.ssthresh);
        } else {
            // Congestion avoidance: +1 per RTT (approximate: count rounds)
            self.ack_rounds = self.ack_rounds.saturating_add(1);
            if self.ack_rounds >= self.cwnd {
                self.cwnd = (self.cwnd + 1).min(MAX_WINDOW as u8);
                self.ack_rounds = 0;
            }
        }
    }

    /// Call on RTO timeout.
    pub fn on_loss(&mut self) {
        self.ssthresh = (self.cwnd / 2).max(1);
        self.cwnd = 1; // reset to 1 on loss like TCP Reno
    }

    #[inline]
    pub fn window(&self) -> u8 {
        self.cwnd
    }
}

#[cfg(any(test, feature = "test-utils"))]
pub mod test_utils {
    use super::*;
    use crate::wifi::common::test_utils::*;

    pub fn make_sender() -> SenderConn {
        SenderConn::new(noop_timer_set, noop_timer_cancel, time_zero, test_crc)
    }
}

#[cfg(test)]
mod sender_tests {

    use super::*;
    use crate::wifi::common::test_utils::*;

    // =========================================================================
    // Sender — SYN / SYN-ACK handshake
    // =========================================================================

    fn make_top_level_sender() -> Sender {
        let mut s = Sender::new();
        s.connect(
            LoRaAddr(0xBEEF),
            noop_timer_set,
            noop_timer_cancel,
            time_zero,
            test_crc,
        );
        s
    }

    fn some_mac() -> Mac {
        Mac::from(0x001122334455)
    }

    fn syn_ack_bytes() -> Vec<u8> {
        AckFrame::syn(0).to_bytes(test_crc).to_vec()
    }

    // --- State before any interaction ---

    #[test]
    fn sender_new_does_not_send() {
        let mut s = Sender::new();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;
        assert!(!s.send_next(&mut mac, &mut data, &mut len));
    }

    #[test]
    fn sender_new_rejects_messages() {
        let mut s = Sender::new();
        assert!(!s.push_message(b"hello"));
    }

    // --- connect() transitions to Syn ---

    #[test]
    fn sender_connect_produces_syn_frame_on_send_next() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        let sent = s.send_next(&mut mac, &mut data, &mut len);
        assert!(sent);
        assert!(!data.is_null());

        // Frame must be broadcast.
        assert_eq!(mac, Mac::bcast());

        // NOTE: len here reflects only the payload-length byte (DataFrame::LEN_OFFSET),
        // not the full wire length including header+CRC. This is a bug in Sender::send_next's
        // Syn branch — kept as-is here so the test documents current behaviour.
        // To parse the actual frame you must use the full syn_packet_buf length instead.
    }

    #[test]
    fn sender_syn_frame_has_syn_flag_and_sync_word() {
        let mut s = Sender::new();
        s.connect(
            LoRaAddr(0x1234),
            noop_timer_set,
            noop_timer_cancel,
            time_zero,
            test_crc,
        );

        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;
        s.send_next(&mut mac, &mut data, &mut len);

        assert_eq!(len as usize, DG_OVERHEAD + size_of::<LoRaAddr>());

        // Read the full serialised SYN frame out of the buffer.
        let raw = unsafe { std::slice::from_raw_parts(data, len as usize) };
        let parsed = DataFrame::parse(raw, test_crc).unwrap();

        assert_ne!(parsed.flags & FLAG_SYN, 0);
        assert_eq!(parsed.seq, 0);
        assert_eq!(parsed.payload, LoRaAddr::from_raw(0x1234).to_le_bytes());
    }

    #[test]
    fn sender_syn_sent_only_once_until_timeout() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        assert!(s.send_next(&mut mac, &mut data, &mut len));
        // Second call before timeout: nothing to send.
        assert!(!s.send_next(&mut mac, &mut data, &mut len));
    }

    #[test]
    fn sender_syn_still_rejects_push_message() {
        let mut s = make_top_level_sender();
        assert!(!s.push_message(b"data"));
    }

    // --- SYN-ACK before SYN is sent is ignored ---

    #[test]
    fn sender_syn_ack_before_syn_sent_is_ignored() {
        let mut s = make_top_level_sender();
        // SYN hasn't been sent yet (syn_sent == false).
        let try_send = s.on_recv_ack(some_mac(), &syn_ack_bytes());
        assert!(!try_send);
        // Should still be in Syn state: can still send the SYN.
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;
        assert!(s.send_next(&mut mac, &mut data, &mut len));
    }

    // --- Timeout retriggers SYN ---

    #[test]
    fn sender_timeout_re_arms_syn_for_retransmit() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        // Send SYN once.
        assert!(s.send_next(&mut mac, &mut data, &mut len));
        // Nothing more to send.
        assert!(!s.send_next(&mut mac, &mut data, &mut len));

        // Timer fires (no response from receiver).
        let try_send = s.on_timeout();
        assert!(try_send);

        // SYN should be re-sendable now.
        assert!(s.send_next(&mut mac, &mut data, &mut len));
    }

    // --- SYN-ACK transitions to Connected ---

    #[test]
    fn sender_syn_ack_transitions_to_connected() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        s.send_next(&mut mac, &mut data, &mut len); // send SYN

        s.on_recv_ack(some_mac(), &syn_ack_bytes());

        // Now in Connected: push_message should be accepted.
        assert!(s.push_message(b"hello"));
    }

    #[test]
    fn sender_connected_mac_is_taken_from_syn_ack_sender() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        s.send_next(&mut mac, &mut data, &mut len);
        s.on_recv_ack(some_mac(), &syn_ack_bytes());
        s.push_message(b"x");

        // send_next in Connected state should write the peer's MAC, not bcast.
        s.send_next(&mut mac, &mut data, &mut len);
        assert_eq!(mac, some_mac());
    }

    #[test]
    fn sender_non_syn_ack_does_not_transition() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        s.send_next(&mut mac, &mut data, &mut len);

        // Plain ACK (no SYN bit) must not transition.
        let plain_ack = AckFrame::ack(0, 0).to_bytes(test_crc);
        s.on_recv_ack(some_mac(), &plain_ack);

        // Still in Syn: push_message rejected.
        assert!(!s.push_message(b"hello"));
    }

    #[test]
    fn sender_rst_does_not_transition() {
        let mut s = make_top_level_sender();
        let mut mac = Mac::bcast();
        let mut data = std::ptr::null();
        let mut len = 0u16;

        s.send_next(&mut mac, &mut data, &mut len);

        let rst = AckFrame::rst().to_bytes(test_crc);
        s.on_recv_ack(some_mac(), &rst);

        assert!(!s.push_message(b"hello"));
    }
}

#[cfg(test)]
mod connection_tests {
    // Test window management, ACK processing, AIMD, RTT estimation, timer arming,
    // retransmit on timeout, Karn's algorithm

    use super::test_utils::*;
    use super::*;
    use crate::wifi::common::test_utils::*;

    // =========================================================================
    // sender — initial state
    // =========================================================================

    #[test]
    fn sender_initial_state() {
        let s = make_sender();
        assert_eq!(s.occupied, 0);
        assert_eq!(s.retransmitted, 0);
        assert_eq!(s.to_retransmit, 0);
        assert_eq!(s.send_base, 0);
        assert_eq!(s.seq_send, 0);
        assert_eq!(s.seq_next, 0);
        assert_eq!(s.in_flight(), 0);
        assert_eq!(s.aimd.window(), 1); // starts at cwnd=1
    }

    // =========================================================================
    // sender — push_message / send_next
    // =========================================================================

    #[test]
    fn sender_push_single_frame_send_single_frame() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        let try_send = s.push_message(b"hello");
        assert!(try_send);
        assert_eq!(s.occupied, 1);
        assert_eq!(s.send_base, 0);
        assert_eq!(s.seq_send, 0);
        assert_eq!(s.seq_next, 1);
        assert_eq!(s.in_flight(), 0);

        let mut data = std::ptr::null();
        let mut len = 0u16;

        let sent = s.send_next(&mut data, &mut len);
        assert!(sent);
        assert_eq!(s.seq_send, 1);
        assert_eq!(s.in_flight(), 1);
        assert_eq!(s.send_base, 0);

        // Second call: nothing more to send
        let sent = s.send_next(&mut data, &mut len);
        assert!(!sent);
        assert_eq!(s.seq_send, 1);
        assert_eq!(s.seq_next, 1);
        assert_eq!(s.in_flight(), 1);
    }

    #[test]
    fn sender_push_frame_blocked_when_window_full() {
        let mut s = make_sender();
        s.aimd.cwnd = 2;

        s.push_message(b"a");
        s.push_message(b"b");
        s.push_message(b"c");

        let mut data = std::ptr::null();
        let mut len = 0u16;

        // Two slots in window
        assert!(s.send_next(&mut data, &mut len));
        assert!(s.send_next(&mut data, &mut len));
        // Third blocked by cwnd
        assert!(!s.send_next(&mut data, &mut len));
    }

    #[test]
    fn sender_push_message_fragments_oversized_payload() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;
        let too_big = vec![0u8; MAX_PAYLOAD + 1];
        let try_send = s.push_message(&too_big);
        assert!(try_send);

        assert_eq!(s.seq_next, 2);
        // Both slots occupied
        assert_eq!(s.occupied.count_ones(), 2);

        let mut data = std::ptr::null();
        let mut len = 0u16;
        assert!(s.send_next(&mut data, &mut len));
        assert!(s.send_next(&mut data, &mut len));
        assert!(!s.send_next(&mut data, &mut len));
    }

    #[test]
    fn sender_push_message_accepts_max_payload() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;
        let exactly_max = vec![0xBBu8; MAX_PAYLOAD];
        // Should not panic; exactly one slot used
        s.push_message(&exactly_max);
        assert_eq!(s.seq_next, 1);
        assert_eq!(s.occupied.count_ones(), 1);
    }

    #[test]
    fn sender_seq_numbers_increment_per_frame() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;

        for expected_seq in 0u8..8 {
            s.push_message(b"x");
            // seq is stored at DataFrame::SEQ_OFFSET in the serialized slot
            let slot = expected_seq as usize % SLOT_LEN;
            assert_eq!(
                s.slots[slot][DataFrame::SEQ_OFFSET],
                expected_seq,
                "wrong seq at push {}",
                expected_seq
            );
        }
    }

    #[test]
    fn sender_push_message_serializes_valid_data_frame() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        let payload = b"test payload";
        s.push_message(payload);

        // Reconstruct the frame length from the slot
        let slot = s.send_base as usize;
        let frame_len = s.slots[slot][DataFrame::LEN_OFFSET] as usize + DG_OVERHEAD;
        let parsed = DataFrame::parse(&s.slots[slot][..frame_len], test_crc).unwrap();
        assert_eq!(parsed.seq, 0);
        assert_eq!(parsed.payload, payload);
    }

    #[test]
    fn sender_push_message_sets_end_flag_on_last_fragment() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;

        // Single-frame message: must have FLAG_END
        s.push_message(b"small");
        let slot = 0;
        let frame_len = s.slots[slot][DataFrame::LEN_OFFSET] as usize + DG_OVERHEAD;
        let parsed = DataFrame::parse(&s.slots[slot][..frame_len], test_crc).unwrap();
        assert_ne!(
            parsed.flags & FLAG_END,
            0,
            "single-frame message must have FLAG_END"
        );
    }

    #[test]
    fn sender_push_message_only_last_fragment_has_end_flag() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;
        let msg = vec![0u8; MAX_PAYLOAD + 1]; // forces 2 frames
        s.push_message(&msg);

        // First frame: no END
        let slot0 = 0usize;
        let len0 = s.slots[slot0][DataFrame::LEN_OFFSET] as usize + DG_OVERHEAD;
        let frame0 = DataFrame::parse(&s.slots[slot0][..len0], test_crc).unwrap();
        assert_eq!(
            frame0.flags & FLAG_END,
            0,
            "first fragment must not have FLAG_END"
        );

        // Second frame: END
        let slot1 = 1usize;
        let len1 = s.slots[slot1][DataFrame::LEN_OFFSET] as usize + DG_OVERHEAD;
        let frame1 = DataFrame::parse(&s.slots[slot1][..len1], test_crc).unwrap();
        assert_ne!(
            frame1.flags & FLAG_END,
            0,
            "last fragment must have FLAG_END"
        );
    }

    // =========================================================================
    // sender — on_rx_ack: window advance and AIMD
    // =========================================================================

    /// Helper: push N single-byte messages and drain send_next.
    fn push_and_send_n(s: &mut SenderConn, n: u8) {
        for _ in 0..n {
            s.push_message(b"x");
        }
        let mut data = std::ptr::null();
        let mut len = 0u16;
        while s.send_next(&mut data, &mut len) {}
    }

    #[test]
    fn sender_clean_ack_advances_send_base() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 2);

        // ack_base=2 means receiver has fully delivered seqs 0 and 1.
        // bitmap=0 means no selective info (clean ACK).
        let ack = AckFrame::ack(2, 0).to_bytes(test_crc);
        let try_send = s.on_recv_ack(&ack);

        assert_eq!(s.send_base, 2);
        assert_eq!(s.in_flight(), 0);
        // All in-flight gone; timer cancelled, no retransmit needed.
        assert!(!try_send);
    }

    #[test]
    fn sender_partial_clean_ack_leaves_remaining_in_flight() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 3); // seqs 0,1,2 in flight

        // Receiver has delivered 0 and 1 (ack_base=2), seq 2 still outstanding.
        let ack = AckFrame::ack(2, 0).to_bytes(test_crc);
        let try_send = s.on_recv_ack(&ack);

        assert_eq!(s.send_base, 2);
        assert_eq!(s.in_flight(), 1); // seq 2 still in flight
        assert!(try_send);
    }

    #[test]
    fn sender_selective_ack_marks_received_frames() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 3); // seqs 0,1,2

        // ack_base=0 (seq 0 not yet delivered); bitmap bit 1 means seq 1 received,
        // bit 2 means seq 2 received. Seq 0 still missing.
        let ack = AckFrame::ack(0, 0b110).to_bytes(test_crc);
        s.on_recv_ack(&ack);

        // send_base must not advance (ack_base didn't move)
        assert_eq!(s.send_base, 0);
        // Slots for seq 1 and 2 should be freed
        let slot1_bit = 1u32 << (1usize % SLOT_LEN);
        let slot2_bit = 1u32 << (2usize % SLOT_LEN);
        assert_eq!(s.occupied & slot1_bit, 0, "slot 1 should be freed");
        assert_eq!(s.occupied & slot2_bit, 0, "slot 2 should be freed");
    }

    #[test]
    fn sender_ack_malformed_frame_is_ignored() {
        let mut s = make_sender();
        s.aimd.cwnd = 2;
        push_and_send_n(&mut s, 1);

        let try_send = s.on_recv_ack(b"garbage bytes that are not a valid frame at all");
        assert!(!try_send);
        assert_eq!(s.send_base, 0); // no advance
    }

    #[test]
    fn sender_ack_does_not_advance_base_past_seq_send() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 2); // seqs 0,1

        // ack_base=5 is beyond seq_send=2; should be ignored
        let ack = AckFrame::ack(5, 0).to_bytes(test_crc);
        s.on_recv_ack(&ack);
        assert_eq!(s.send_base, 0);
    }

    // =========================================================================
    // sender — AIMD
    // =========================================================================

    #[test]
    fn sender_aimd_slow_start_doubles_cwnd() {
        let mut s = make_sender();
        assert_eq!(s.aimd.window(), 1);
        s.aimd.on_clean_ack();
        assert_eq!(s.aimd.window(), 2);
        s.aimd.on_clean_ack();
        assert_eq!(s.aimd.window(), 4);
        s.aimd.on_clean_ack();
        assert_eq!(s.aimd.window(), 8);
    }

    #[test]
    fn sender_aimd_loss_halves_ssthresh_and_resets_cwnd() {
        let mut s = make_sender();
        s.aimd.cwnd = 32;
        s.aimd.on_loss();
        assert_eq!(s.aimd.cwnd, 1);
        assert_eq!(s.aimd.ssthresh, 16);
    }

    #[test]
    fn sender_aimd_loss_from_cwnd_1_sets_ssthresh_to_1() {
        let mut s = make_sender();
        assert_eq!(s.aimd.cwnd, 1);
        s.aimd.on_loss();
        assert_eq!(s.aimd.ssthresh, 1); // max(1/2, 1) = 1
        assert_eq!(s.aimd.cwnd, 1);
    }

    #[test]
    fn sender_aimd_cwnd_clamps_to_max_window() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;
        s.aimd.ssthresh = MAX_WINDOW as u8; // force congestion-avoidance path
        for _ in 0..=MAX_WINDOW {
            s.aimd.on_clean_ack();
        }
        assert!(s.aimd.window() <= MAX_WINDOW as u8);
    }

    #[test]
    fn sender_aimd_congestion_avoidance_increments_slowly() {
        let mut s = make_sender();
        s.aimd.cwnd = 16;
        s.aimd.ssthresh = 8; // already above ssthresh → congestion avoidance
        let initial = s.aimd.window();
        for _ in 0..initial {
            s.aimd.on_clean_ack();
        }
        assert_eq!(s.aimd.window(), initial + 1);
    }

    // =========================================================================
    // sender — on_timeout: retransmit and AIMD loss
    // =========================================================================

    #[test]
    fn sender_timer_fire_resets_cwnd_and_sets_ssthresh() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 2);

        let cwnd_before = s.aimd.window();
        let try_send = s.on_timeout();

        assert!(try_send);
        assert_eq!(s.aimd.window(), 1);
        assert!(s.aimd.ssthresh <= cwnd_before / 2 + 1);
    }

    #[test]
    fn sender_timer_fire_marks_in_flight_for_retransmit() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 3); // seqs 0,1,2 all in flight

        // After loss, all three should be flagged for retransmit
        s.on_timeout();

        // to_retransmit & occupied should have exactly 3 bits set
        let pending = s.to_retransmit & s.occupied;
        assert_eq!(pending.count_ones(), 3);
        // That frame must be seq 0 (send_base)
        assert_eq!(pending, 0b111 << (s.send_base as usize % SLOT_LEN));
    }

    #[test]
    fn sender_timer_fire_sets_retransmitted_bitmap() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 1); // seq 0

        assert_eq!(s.retransmitted, 0);
        s.on_timeout();
        // Slot 0 bit must be set in retransmitted
        assert_ne!(s.retransmitted & 1, 0);
    }

    #[test]
    fn sender_send_next_transmits_retransmit_frame() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 2); // seqs 0,1

        s.on_timeout(); // cwnd drops to 1, seq 0 queued for retransmit

        let mut data = std::ptr::null();
        let mut len = 0u16;
        let sent = s.send_next(&mut data, &mut len);
        assert!(sent);

        // Verify the retransmitted frame is seq 0
        let raw = unsafe { std::slice::from_raw_parts(data, len as usize) };
        let parsed = DataFrame::parse(raw, test_crc).unwrap();
        assert_eq!(parsed.seq, 0);
    }

    // =========================================================================
    // sender — Karn's algorithm
    // =========================================================================

    #[test]
    fn sender_karn_skips_rtt_sample_for_retransmitted_frame() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 1); // seq 0
        s.on_timeout(); // marks seq 0 as retransmitted

        assert!(!s.rtt.have_sample);

        // ACK seq 0 via ack_base advance
        let ack = AckFrame::ack(1, 0).to_bytes(test_crc);
        s.on_recv_ack(&ack);

        // RTT estimator must NOT have been updated
        assert!(!s.rtt.have_sample);
    }

    #[test]
    fn sender_karn_samples_rtt_for_non_retransmitted_frame() {
        let mut s = make_sender();
        s.aimd.cwnd = 4;
        push_and_send_n(&mut s, 1); // seq 0, not retransmitted

        assert!(!s.rtt.have_sample);

        let ack = AckFrame::ack(1, 0).to_bytes(test_crc);
        s.on_recv_ack(&ack);

        assert!(s.rtt.have_sample);
    }

    // =========================================================================
    // sender — seq number wraparound
    // =========================================================================

    #[test]
    fn sender_seq_wraps_at_255() {
        let mut s = make_sender();
        s.aimd.cwnd = MAX_WINDOW as u8;
        // Manually position near wraparound
        s.send_base = 254;
        s.seq_send = 254;
        s.seq_next = 254;

        s.push_message(b"a"); // seq 254
        s.push_message(b"b"); // seq 255
        s.push_message(b"c"); // seq 0 (wrapped)

        assert_eq!(s.seq_next, 1); // next would be seq 1

        // Verify seq 255 is in slot 255 % SLOT_LEN = 31
        let slot_255 = 255usize % SLOT_LEN;
        assert_eq!(s.slots[slot_255][DataFrame::SEQ_OFFSET], 255);

        // Verify wrapped frame is in slot 0 % SLOT_LEN = 0
        let slot_0 = 0usize % SLOT_LEN;
        assert_eq!(s.slots[slot_0][DataFrame::SEQ_OFFSET], 0);
    }
}
