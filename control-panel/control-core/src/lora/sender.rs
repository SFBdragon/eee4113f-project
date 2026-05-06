//! sender.rs — no_std, no alloc, no atomics
//!
//! Designed as a pure state machine.
//! The caller is responsible for mutual exclusion and I/O.

use control_protocol::{CrcFn, DeviceId, RttEstimator, lora::MAX_FRAME};

use crate::lora::LoRaDev;

pub const INITIAL_RTO_MS: u32 = 2000;

pub trait LoRaTimer {
    fn set(&mut self, timeout_ms: u32);
    fn cancel(&mut self);
    fn get_time(&self) -> u32;
}

pub struct Tranceiver<T: LoRaTimer> {
    device_id: DeviceId,
    timer: T,
    crc_fn: CrcFn,
    
    rtt: RttEstimator,
    sequence_flag: bool,

    frame_to_send: [u8; MAX_FRAME],
}

impl<T: LoRaTimer> Tranceiver<T> {
    pub const fn new(device_id: DeviceId, timer: T) -> Self {
        Self {
            device_id,
            timer,
            crc_fn: compute_crc_raw,
            rtt: RttEstimator::new(INITIAL_RTO_MS),
            sequence_flag: false,
        }
    }

    pub fn send_and_recv(&mut self, dev: &LoRaDev) -> 
}


// ── Sender state machine ──────────────────────────────────────────────────────

/// It is assumed that SEQ_SPACE (256) is a multiple of SLOT_LEN for the rotates. 
const SLOT_LEN: usize = 32;
type SlotBitMap = u32;

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
    pub set_timer: fn(u32),
    pub cancel_timer: fn(),
    pub get_time_ms: fn() -> u32,
}

impl SenderConn {
    pub const fn new(set_timer: fn(u32), cancel_timer: fn(), get_time_ms: fn() -> u32, crc_fn: CrcFn) -> Self {
        Self {
            slots: [[0u8; MAX_DATAGRAM]; SLOT_LEN],
            occupied: 0,
            retransmitted: 0,
            sent_at_ms: [0; SLOT_LEN],

            to_retransmit: 0,

            send_base: 0,
            seq_send: 0,
            seq_next: 0,

            rtt: RttEstimator::new(),
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
    pub fn push_message(
        &mut self,
        message: &[u8],
    ) -> bool {
        if message.len() > self.available_payload_bytes() { panic!(); }
    
        let mut b = 0;
        while b < message.len() {
            let last_in_message = b + MAX_PAYLOAD > message.len();
            let flags = if last_in_message { FLAG_END } else { 0 };
            let len = if last_in_message { message.len() - b } else { MAX_PAYLOAD };

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
            Err(e) => return false,
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
            let acked = (((1 as SlotBitMap) << base_offset) - 1).rotate_left(self.send_base as u32 % SLOT_LEN as u32);
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

        let mask = (((1 as SlotBitMap) << in_flight) - 1).rotate_left(self.send_base as u32 % SLOT_LEN as u32);
        self.retransmitted |= mask & self.occupied;
        self.to_retransmit |= mask & self.occupied;

        self.to_retransmit & self.occupied != 0
    }
}
