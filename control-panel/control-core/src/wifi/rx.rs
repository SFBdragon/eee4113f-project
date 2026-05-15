//! receiver.rs — runs on hosted system - std is available
//!
//! The receiver runs a delayed-ACK policy: send ACK after every N frames
//! received, OR after ACK_DELAY_MS, whichever comes first.
//!
//! The receiver does no retransmission. It builds and sends AckFrames only.

use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use control_protocol::{
    LoRaAddr,
    wifi::{Mac, common::*},
};

// ── Delayed ACK config ────────────────────────────────────────────────────────

/// Send an ACK after this many frames received (if timer hasn't fired first).
pub const ACK_EVERY_N_FRAMES: usize = 4;

/// Send an ACK after this many ms since last ACK (if N frames not yet reached).
pub const ACK_DELAY: Duration = Duration::from_millis(20);

// ── Receive buffer ────────────────────────────────────────────────────────────

/// One slot per sequence number in the window.
/// Payload is heap-allocated since the receiver is std.
pub struct RecvSlot {
    // Putting the optional here results in layout optimizations.
    pub data: Option<Box<[u8]>>,
    pub flags: u8,
}

// ── Receiver state machine ────────────────────────────────────────────────────

pub struct WiFiRx {
    pub timeout: Option<crossbeam_channel::Receiver<Instant>>,
    controller_addr: LoRaAddr,
    syn_ack: HashMap<Mac, u8>,
    connection: Option<(Mac, Connection)>,
}

impl WiFiRx {
    pub fn new(controller_addr: LoRaAddr) -> Self {
        Self {
            timeout: None,
            controller_addr,
            syn_ack: HashMap::new(),
            connection: None,
        }
    }

    pub fn on_recv(
        &mut self,
        module_mac: Mac,
        bytes: &[u8],
        connected: impl FnOnce(Mac),
        disconnected: impl FnOnce(Mac),
        on_message: impl FnMut(Vec<u8>),
        on_ping: impl FnOnce((Mac, LoRaAddr)),
    ) -> Option<(Mac, [u8; ACK_SIZE])> {
        let frame = match DataFrame::parse(bytes, compute_crc_raw) {
            Ok(f) => f,
            Err(e) => {
                tracing::warn!(?e, "WiFi WiFiRx::on_recv parsing error.");
                return None;
            }
        };

        tracing::debug!(%module_mac, ?frame, "On recv WiFi packet.");

        if frame.flags & FLAG_PING == FLAG_PING {
            on_ping((
                module_mac,
                LoRaAddr::from_raw(u16::from_le_bytes(frame.payload.try_into().unwrap())),
            ));

            return None;
        }

        if frame.flags & FLAG_SYN == FLAG_SYN {
            // SYN packet. If the sync word matches, SYN-ACK to continue connection setup.
            // If the sync word doesn't match, this is the wrong device. Ignore it.
            // The sync word is allowing us to single out the device we want to connect to,
            // as determined by the LoRa-based control communication.

            tracing::debug!("FLAG SYN");

            if frame.payload == &self.controller_addr.to_le_bytes() {
                // Prepare to sync to this connection if we hear back again.
                self.syn_ack.insert(module_mac, frame.seq);
                // SYN-ACK and return
                Some((
                    module_mac,
                    AckFrame::syn(frame.seq).to_bytes(compute_crc_raw),
                ))
            } else {
                None
            }
        } else {
            // Not a SYN. Actually has data.

            // Is it the follow up to a SYN? If so, establish a connection.
            if let Some(syn_seq) = self.syn_ack.get(&module_mac).copied() {
                // If a connection already exists, indicate disconnection to the app.
                if let Some((con_mac, _)) = &self.connection {
                    self.timeout = None;
                    disconnected(*con_mac);
                }

                tracing::debug!("WiFi handshake done.");

                // Switch connection.
                self.connection = Some((module_mac, Connection::new(syn_seq)));
                // On establishing a connection, wipe out the existing sync-requests.
                // Other clients that attempted to connect will need to try reconnect.
                self.syn_ack.clear();

                // Signal to the app that we connected.
                connected(module_mac);
            }

            // Is the frame for the current connection?
            if let Some((conn_mac, conn)) = &mut self.connection {
                if module_mac == *conn_mac {
                    return conn
                        .on_recv(frame, &mut self.timeout, on_message)
                        .map(|a| (module_mac, a));
                }
            }

            // Not for any connections we know about, and not a SYN.
            // Send a RST to tell them to reconnect or go away.
            Some((module_mac, AckFrame::rst().to_bytes(compute_crc_raw)))
        }
    }

    /// Call when a connection's delayed-ACK timer fires.
    pub fn on_timeout(&mut self) -> (Mac, [u8; ACK_SIZE]) {
        self.timeout = None;

        if let Some((mac, conn)) = &mut self.connection {
            (*mac, conn.on_timeout())
        } else {
            unreachable!()
        }
    }
}

struct Connection {
    /// Ring buffer indexed by seq mod MAX_WINDOW.
    slots: [RecvSlot; MAX_WINDOW],
    recv_base: u8,
    /// Frames received since last ACK (for delayed-ACK N-frame trigger).
    frames_since_ack: usize,

    message: Vec<u8>,
}

impl Connection {
    pub fn new(seq_start: u8) -> Self {
        let slots = core::array::from_fn(|_| RecvSlot {
            data: None,
            flags: 0,
        });
        Self {
            slots,
            recv_base: seq_start,
            frames_since_ack: 0,
            message: Vec::new(),
        }
    }

    /// Build the current ACK bitmap reflecting which slots in [recv_base,
    /// recv_base + MAX_WINDOW) have been received.
    fn build_bitmap(&self) -> WindowBitMap {
        let mut bitmap = 0;
        for i in 0..MAX_WINDOW {
            let idx = (self.recv_base as usize + i) % MAX_WINDOW;
            if self.slots[idx].data.is_some() {
                bitmap |= 1 << i;
            }
        }
        bitmap
    }

    /// Serialize an ACK frame for the current window state.
    fn make_ack(&self) -> [u8; ACK_SIZE] {
        let frame = AckFrame {
            flags: FLAG_ACK,
            ack_base: self.recv_base,
            bitmap: self.build_bitmap(),
        };
        frame.to_bytes(compute_crc_raw)
    }

    /// Drain in-order frames from recv_base and return the first one ready for
    /// delivery (caller must call repeatedly until None to drain the full run).
    fn drain(&mut self) -> Option<(u8, Box<[u8]>)> {
        let idx = (self.recv_base as usize) % MAX_WINDOW;
        if self.slots[idx].data.is_some() {
            let payload = self.slots[idx].data.take().unwrap();
            let flags = self.slots[idx].flags;
            self.recv_base = self.recv_base.wrapping_add(1);
            Some((flags, payload))
        } else {
            None
        }
    }

    /// Call when a raw frame is received over ESP-NOW.
    /// This packet is
    pub fn on_recv(
        &mut self,
        frame: DataFrame,
        timer: &mut Option<crossbeam_channel::Receiver<Instant>>,
        mut on_message: impl FnMut(Vec<u8>),
    ) -> Option<[u8; ACK_SIZE]> {
        let seq = frame.seq;

        // Discard duplicates or frames outside our window
        if !seq_in_window(seq, self.recv_base, MAX_WINDOW as u8) {
            // Out of window — send an ACK anyway (may be a retransmit of
            // something we already ACKed; ACK helps the sender advance)
            return Some(self.make_ack());
        }

        let idx = (seq as usize) % MAX_WINDOW;
        if self.slots[idx].data.is_none() {
            let boxed = frame.payload.to_vec().into_boxed_slice();
            self.slots[idx].data = Some(boxed);
            self.slots[idx].flags = frame.flags;
        }

        // Drain in-order frames
        while let Some((flags, payload)) = self.drain() {
            self.message.extend_from_slice(&payload);

            if flags & FLAG_END != 0 {
                let message = std::mem::replace(&mut self.message, Vec::new());
                on_message(message);
            }
        }

        self.frames_since_ack += 1;
        let should_ack = self.frames_since_ack >= ACK_EVERY_N_FRAMES; // TODO or if FIN packet?

        if should_ack {
            self.frames_since_ack = 0;
            *timer = None;

            return Some(self.make_ack());
        } else {
            if timer.is_none() {
                *timer = Some(crossbeam_channel::after(ACK_DELAY))
            }

            None
        }
    }

    /// Call when the delayed-ACK timer fires.
    pub fn on_timeout(&mut self) -> [u8; ACK_SIZE] {
        self.frames_since_ack = 0;
        self.make_ack()
    }
}

extern "C" fn compute_crc_raw(data: *const u8, len: usize) -> u16 {
    let buf = unsafe { std::slice::from_raw_parts(data, len) };
    crate::crc::compute_crc(buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use control_protocol::wifi::sender::test_utils::*;

    // ── Test helpers ──────────────────────────────────────────────────────────

    const TEST_ADDR: LoRaAddr = LoRaAddr::from_raw(0x1ABC);

    fn some_mac() -> Mac {
        Mac::from(0x001122334455u64)
    }

    fn other_mac() -> Mac {
        Mac::from(0x00AABBCCDD00u64)
    }

    /// Serialize a DATA frame manually, bypassing the Sender state machine.
    pub fn make_data_frame(seq: u8, flags: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; MAX_DATAGRAM];
        let frame = DataFrame {
            flags,
            seq,
            len: payload.len() as u8,
            payload,
        };
        let len = frame.serialize(&mut buf, compute_crc_raw);
        buf[..len].to_vec()
    }

    /// Drive the SYN handshake to completion. Returns a `Receiver` that has
    /// an established connection from `mac`, with the connection's seq base
    /// set to `syn_seq`. Also returns any SYN-ACK bytes produced.
    ///
    /// The handshake is:
    ///   1. Send SYN frame (FLAG_SYN, payload = device_id LE bytes).
    ///   2. Receiver replies with SYN-ACK.
    ///   3. Send any non-SYN data frame — this causes the receiver to commit the
    ///      connection (the first data frame after a SYN-ACK establishes it).
    fn make_connected(mac: Mac, syn_seq: u8) -> WiFiRx {
        let mut rx = WiFiRx::new(TEST_ADDR);

        // collected connected/disconnected callbacks
        let mut connected_macs: Vec<Mac> = Vec::new();
        let mut disconnected_macs: Vec<Mac> = Vec::new();

        // Step 1: SYN frame with the correct address payload.
        let syn_frame = {
            let payload = TEST_ADDR.to_le_bytes();
            make_data_frame(syn_seq, FLAG_SYN, &payload)
        };
        let syn_ack = rx.on_recv(
            mac,
            &syn_frame,
            |m| connected_macs.push(m),
            |m| disconnected_macs.push(m),
            |_| panic!(),
            |_| panic!(),
        );
        assert!(syn_ack.is_some(), "SYN must produce a SYN-ACK");

        assert_eq!(connected_macs.len(), 0);
        assert_eq!(disconnected_macs.len(), 0);

        // Step 2: first data frame to commit the connection.
        let first_data = make_data_frame(syn_seq, FLAG_END, b"");
        rx.on_recv(
            mac,
            &first_data,
            |m| connected_macs.push(m),
            |m| disconnected_macs.push(m),
            |_| {},
            |_| panic!(),
        );

        assert_eq!(connected_macs.len(), 1);
        assert_eq!(disconnected_macs.len(), 0);

        rx
    }

    /// Feed a raw frame through an already-connected `Receiver` and return the
    /// optional (mac, ack_bytes) pair. Panics if any `connected`/`disconnected`
    /// callback fires unexpectedly.
    fn feed(
        recv: &mut WiFiRx,
        mac: Mac,
        raw: &[u8],
        messages: &mut Vec<Vec<u8>>,
    ) -> Option<[u8; ACK_SIZE]> {
        recv.on_recv(
            mac,
            raw,
            |_| panic!("unexpected connect callback"),
            |_| panic!("unexpected disconnect callback"),
            |msg| messages.push(msg),
            |_| panic!(),
        )
        .map(|(_, ack)| ack)
    }

    // =========================================================================
    // Receiver — initial state
    // =========================================================================

    #[test]
    fn receiver_initial_state_has_no_connection() {
        let recv = WiFiRx::new(TEST_ADDR);
        assert!(recv.connection.is_none());
        assert!(recv.timeout.is_none());
        assert!(recv.syn_ack.is_empty());
        assert_eq!(recv.controller_addr, TEST_ADDR);
    }

    // =========================================================================
    // Receiver — SYN handshake
    // =========================================================================

    #[test]
    fn receiver_replies_syn_ack_for_correct_device_id() {
        let mut recv = WiFiRx::new(TEST_ADDR);
        let payload = TEST_ADDR.to_le_bytes();
        let syn = make_data_frame(0, FLAG_SYN, &payload);

        let result = recv.on_recv(some_mac(), &syn, |_| {}, |_| {}, |_| {}, |_| panic!());

        let (_, ack_bytes) = result.expect("SYN must produce a SYN-ACK");
        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        assert_ne!(parsed.flags & FLAG_ACK, 0);
        assert_ne!(parsed.flags & FLAG_SYN, 0);
        assert_eq!(
            parsed.ack_base, 0,
            "SYN-ACK ack_base should echo the SYN seq"
        );
    }

    #[test]
    fn receiver_ignores_syn_with_wrong_device_id() {
        let mut rx = WiFiRx::new(TEST_ADDR);
        // Wrong payload — different device_id
        let payload = LoRaAddr::from_raw(999).to_le_bytes();
        let syn = make_data_frame(0, FLAG_SYN, &payload);

        let result = rx.on_recv(
            some_mac(),
            &syn,
            |_| panic!("should not connect"),
            |_| {},
            |_| {},
            |_| panic!(),
        );
        assert!(
            result.is_none(),
            "wrong device_id SYN must be silently dropped"
        );
    }

    #[test]
    fn receiver_syn_with_nonzero_seq_echoes_that_seq_in_syn_ack() {
        let mut rx = WiFiRx::new(TEST_ADDR);
        let payload = TEST_ADDR.to_le_bytes();
        let syn = make_data_frame(42, FLAG_SYN, &payload);

        let (_, ack_bytes) = rx
            .on_recv(some_mac(), &syn, |_| {}, |_| {}, |_| {}, |_| panic!())
            .unwrap();

        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        assert_eq!(parsed.ack_base, 42);
    }

    #[test]
    fn receiver_fires_connected_callback_on_first_data_after_syn() {
        let mut connected_macs: Vec<Mac> = Vec::new();
        let mut rx = WiFiRx::new(TEST_ADDR);

        let payload = TEST_ADDR.to_le_bytes();
        let syn = make_data_frame(0, FLAG_SYN, &payload);
        rx.on_recv(some_mac(), &syn, |_| {}, |_| {}, |_| {}, |_| panic!());

        let first_data = make_data_frame(0, FLAG_END, b"");
        rx.on_recv(
            some_mac(),
            &first_data,
            |m| connected_macs.push(m),
            |_| {},
            |_| {},
            |_| panic!(),
        );

        assert_eq!(connected_macs, vec![some_mac()]);
    }

    #[test]
    fn receiver_data_before_syn_returns_rst() {
        let mut rx = WiFiRx::new(TEST_ADDR);
        let frame = make_data_frame(0, FLAG_END, b"hello");

        let result = rx.on_recv(
            some_mac(),
            &frame,
            |_| panic!("should not connect"),
            |_| {},
            |_| {},
            |_| panic!(),
        );

        let (_, ack_bytes) = result.expect("unknown sender must get RST");
        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        assert_ne!(parsed.flags & FLAG_RST, 0);
    }

    #[test]
    fn receiver_malformed_frame_returns_none() {
        let mut rx = WiFiRx::new(TEST_ADDR);

        let result = rx.on_recv(
            some_mac(),
            b"not a frame",
            |_| {},
            |_| {},
            |_| {},
            |_| panic!(),
        );
        assert!(result.is_none());
    }

    #[test]
    fn receiver_new_syn_from_different_mac_disconnects_old_connection() {
        // Establish first connection from mac A.
        let mut recv = make_connected(some_mac(), 0);

        // New SYN + data from mac B.
        let mut disconnected_macs = Vec::new();
        let mut connected_macs = Vec::new();

        let payload = TEST_ADDR.to_le_bytes();
        let syn = make_data_frame(0, FLAG_SYN, &payload);
        recv.on_recv(
            other_mac(),
            &syn,
            |m| connected_macs.push(m),
            |m| disconnected_macs.push(m),
            |_| {},
            |_| panic!(),
        );

        let first_data = make_data_frame(0, FLAG_END, b"");
        recv.on_recv(
            other_mac(),
            &first_data,
            |m| connected_macs.push(m),
            |m| disconnected_macs.push(m),
            |_| {},
            |_| panic!(),
        );

        assert!(
            disconnected_macs.contains(&some_mac()),
            "old connection must be signalled as disconnected"
        );
        assert!(
            connected_macs.contains(&other_mac()),
            "new connection must be signalled as connected"
        );
    }

    // =========================================================================
    // Connection — basic delivery
    // =========================================================================

    #[test]
    fn connection_delivers_single_end_frame() {
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();
        // seq=1 because establish_connection already consumed seq=0
        let frame = make_data_frame(1, FLAG_END, b"hello");
        feed(&mut recv, some_mac(), &frame, &mut messages);

        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0], b"hello");
    }

    #[test]
    fn connection_reassembles_fragmented_message() {
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();

        // seq=1: fragment, no FLAG_END
        let f1 = make_data_frame(1, 0, b"hello ");
        feed(&mut recv, some_mac(), &f1, &mut messages);
        assert_eq!(messages.len(), 0, "no message before FLAG_END");

        // seq=2: final fragment with FLAG_END
        let f2 = make_data_frame(2, FLAG_END, b"world");
        feed(&mut recv, some_mac(), &f2, &mut messages);

        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0], b"hello world");
    }

    #[test]
    fn connection_recv_base_advances_as_frames_drain() {
        // We can't inspect recv_base directly; verify indirectly via bitmap in ACK.
        // After seq 0 (from establish) + seq 1 drain in-order, the ACK's ack_base
        // should be 2 (both drained).
        let mut recv = make_connected(some_mac(), 0);

        let frame = make_data_frame(1, FLAG_END, b"x");
        // Feed enough frames to trigger the N-frame ACK (4 frames total including
        // the establish frame).
        let f2 = make_data_frame(2, 0, b"a");
        let f3 = make_data_frame(3, 0, b"b");
        let mut messages = Vec::new();
        feed(&mut recv, some_mac(), &frame, &mut messages);
        feed(&mut recv, some_mac(), &f2, &mut messages);
        let ack_bytes = feed(&mut recv, some_mac(), &f3, &mut messages);

        // We've sent 3 more frames after the establish frame (which was frame #1),
        // so this is the 4th total — should trigger an ACK.
        let ack_bytes = ack_bytes.expect("4th frame should trigger N-frame ACK");
        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        // All four frames (seq 0,1,2,3) drained in order → ack_base should be 4.
        assert_eq!(parsed.ack_base, 4);
    }

    #[test]
    fn connection_buffers_out_of_order_and_delivers_on_gap_fill() {
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();

        // seq=3 arrives before seq=1 and seq=2 (out of order)
        let f3 = make_data_frame(3, 0, b"c");
        feed(&mut recv, some_mac(), &f3, &mut messages);
        assert_eq!(messages.len(), 0);

        // seq=2 arrives (still a gap at seq=1)
        let f2 = make_data_frame(2, 0, b"b");
        feed(&mut recv, some_mac(), &f2, &mut messages);
        assert_eq!(messages.len(), 0);

        // seq=1 fills the gap; seq=1,2,3 drain. No FLAG_END so no message yet.
        let f1 = make_data_frame(1, 0, b"a");
        feed(&mut recv, some_mac(), &f1, &mut messages);
        // Still no complete message — no FLAG_END seen.
        assert_eq!(messages.len(), 0);

        // seq=4 with FLAG_END — entire message a+b+c+d should be delivered.
        let f4 = make_data_frame(4, FLAG_END, b"d");
        feed(&mut recv, some_mac(), &f4, &mut messages);
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0], b"abcd");
    }

    #[test]
    fn connection_discards_duplicate_frames() {
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();
        let frame = make_data_frame(1, FLAG_END, b"once");

        feed(&mut recv, some_mac(), &frame, &mut messages);
        feed(&mut recv, some_mac(), &frame, &mut messages); // duplicate

        assert_eq!(messages.len(), 1, "duplicate frame must not deliver twice");
    }

    #[test]
    fn connection_out_of_window_frame_returns_ack_no_delivery() {
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();
        // recv_base is 1 after establish (seq 0 drained); window is [1, 1+MAX_WINDOW).
        // MAX_WINDOW = 32, so seq 33 is out of window.
        let oow = make_data_frame(1 + MAX_WINDOW as u8, FLAG_END, b"oow");
        let ack = feed(&mut recv, some_mac(), &oow, &mut messages);

        assert_eq!(
            messages.len(),
            0,
            "out-of-window frame must not be delivered"
        );
        assert!(
            ack.is_some(),
            "out-of-window frame should still elicit an ACK"
        );
    }

    // =========================================================================
    // Connection — delayed ACK policy
    // =========================================================================

    #[test]
    fn connection_acks_after_n_frames() {
        let mut recv = make_connected(some_mac(), 0);

        // The establish helper already consumed 1 frame. Feed ACK_EVERY_N_FRAMES-1
        // more to reach the threshold.
        let mut ack_count = 0usize;
        let mut messages = Vec::new();

        for i in 1..(ACK_EVERY_N_FRAMES as u8) {
            let raw = make_data_frame(i, 0, b"x");
            if feed(&mut recv, some_mac(), &raw, &mut messages).is_some() {
                ack_count += 1;
            }
        }

        assert_eq!(ack_count, 1, "exactly one N-frame ACK must be sent");
    }

    #[test]
    fn connection_no_ack_before_n_frames() {
        let mut recv = make_connected(some_mac(), 0);

        // Feed fewer than ACK_EVERY_N_FRAMES-1 additional frames (the establish
        // frame counts as the first).
        let mut messages = Vec::new();
        let mut ack_count = 0usize;
        // Feed ACK_EVERY_N_FRAMES - 2 more (total = ACK_EVERY_N_FRAMES - 1, below threshold).
        for i in 1..(ACK_EVERY_N_FRAMES as u8 - 1) {
            let raw = make_data_frame(i, 0, b"y");
            if feed(&mut recv, some_mac(), &raw, &mut messages).is_some() {
                ack_count += 1;
            }
        }

        assert_eq!(ack_count, 0, "must not ACK before the N-frame threshold");
    }

    #[test]
    fn connection_on_timeout_returns_ack() {
        let mut recv = make_connected(some_mac(), 0);

        // Feed one frame — below N threshold, timer should be armed.
        let mut messages = Vec::new();
        let frame = make_data_frame(1, 0, b"t");
        let immediate_ack = feed(&mut recv, some_mac(), &frame, &mut messages);
        assert!(
            immediate_ack.is_none(),
            "single frame should not trigger N-frame ACK"
        );

        // Simulate timer fire.
        let (mac, ack_bytes) = recv.on_timeout();
        assert_eq!(mac, some_mac());
        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        assert_ne!(parsed.flags & FLAG_ACK, 0);
    }

    #[test]
    fn connection_n_frame_ack_resets_frame_counter() {
        // After the N-frame ACK fires, subsequent frames should again need N
        // frames to trigger the next ACK (or the timer).
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();

        // Fill up to the N-frame ACK.
        for i in 1..(ACK_EVERY_N_FRAMES as u8) {
            let raw = make_data_frame(i, 0, b"x");
            feed(&mut recv, some_mac(), &raw, &mut messages);
        }

        // Now send one more frame — must NOT trigger another immediate ACK.
        let next_seq = ACK_EVERY_N_FRAMES as u8;
        let raw = make_data_frame(next_seq, 0, b"x");
        let ack = feed(&mut recv, some_mac(), &raw, &mut messages);
        assert!(
            ack.is_none(),
            "frame counter should have reset; no ACK until N more frames"
        );
    }

    // =========================================================================
    // Connection — bitmap correctness
    // =========================================================================

    #[test]
    fn connection_bitmap_reflects_gap() {
        // After establish (seq=0 drained, recv_base=1), receive seq=1 and seq=3
        // but skip seq=2. The ACK bitmap, relative to recv_base=2 (seq=1 drained),
        // should have bit 1 set (seq=3) and bit 0 clear (seq=2 missing).
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();

        let f1 = make_data_frame(1, 0, b"a"); // drains immediately → recv_base=2
        feed(&mut recv, some_mac(), &f1, &mut messages);

        let f3 = make_data_frame(3, 0, b"c"); // buffered at offset 1 from recv_base=2
        feed(&mut recv, some_mac(), &f3, &mut messages);

        // One more frame to hit the N-frame threshold and get the ACK.
        let f4 = make_data_frame(4, 0, b"d");
        let ack_bytes = feed(&mut recv, some_mac(), &f4, &mut messages)
            .expect("should reach N-frame ACK threshold");

        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        assert_eq!(
            parsed.ack_base, 2,
            "recv_base should be 2 (gap at seq 2 blocks drain)"
        );
        // Bit 0 → seq 2 (missing): 0. Bit 1 → seq 3 (present): 1. Bit 2 → seq 4 (present): 1.
        assert_eq!(
            parsed.bitmap & 0b001,
            0,
            "bit 0 (seq 2) must be 0 — missing"
        );
        assert_ne!(
            parsed.bitmap & 0b010,
            0,
            "bit 1 (seq 3) must be 1 — received"
        );
        assert_ne!(
            parsed.bitmap & 0b100,
            0,
            "bit 2 (seq 4) must be 1 — received"
        );
    }

    #[test]
    fn connection_bitmap_zero_when_window_fully_drained() {
        let mut recv = make_connected(some_mac(), 0);

        // Feed ACK_EVERY_N_FRAMES-1 consecutive in-order frames to trigger ACK.
        let mut messages = Vec::new();
        let mut ack_bytes = None;
        for i in 1..(ACK_EVERY_N_FRAMES as u8) {
            let raw = make_data_frame(i, 0, b"z");
            ack_bytes = feed(&mut recv, some_mac(), &raw, &mut messages);
        }

        let ack_bytes = ack_bytes.expect("N-frame ACK must have fired");
        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        assert_eq!(
            parsed.bitmap, 0,
            "fully-drained window must produce bitmap=0 (clean ACK)"
        );
    }

    #[test]
    fn connection_ack_is_parseable() {
        let mut recv = make_connected(some_mac(), 0);

        let mut messages = Vec::new();
        let mut ack_bytes = None;
        for i in 1..(ACK_EVERY_N_FRAMES as u8) {
            let raw = make_data_frame(i, 0, b"p");
            ack_bytes = feed(&mut recv, some_mac(), &raw, &mut messages);
        }
        let ack_bytes = ack_bytes.unwrap();
        // AckFrame::parse must succeed — the sender would use this.
        assert!(AckFrame::parse(&ack_bytes, compute_crc_raw).is_ok());
    }

    // =========================================================================
    // Connection — seq wraparound
    // =========================================================================

    #[test]
    fn connection_seq_wraparound_delivers_correctly() {
        // Start with syn_seq=250 so we cross the 255→0 boundary during receive.
        let mut recv = make_connected(some_mac(), 250);

        let mut messages = Vec::new();
        // Seqs 251..=255 then 0..=4, last one with FLAG_END.
        let seqs: Vec<u8> = (251u16..=259).map(|s| s as u8).collect(); // wraps: 251,252,253,254,255,0,1,2,3
        for (i, &seq) in seqs.iter().enumerate() {
            let flags = if i == seqs.len() - 1 { FLAG_END } else { 0 };
            let payload = [seq]; // use seq as payload byte for easy verification
            let raw = make_data_frame(seq, flags, &payload);
            feed(&mut recv, some_mac(), &raw, &mut messages);
        }

        assert_eq!(messages.len(), 1);
        let expected: Vec<u8> = seqs.iter().copied().collect();
        assert_eq!(
            messages[0], expected,
            "message must be assembled correctly across wrap"
        );
    }

    // =========================================================================
    // Integration — Sender + Receiver
    // =========================================================================

    /// Feed a serialized frame through an established receiver. Returns the
    /// optional ACK bytes (without the leading mac).
    fn feed_raw(
        recv: &mut WiFiRx,
        mac: Mac,
        raw: &[u8],
        messages: &mut Vec<Vec<u8>>,
    ) -> Option<[u8; ACK_SIZE]> {
        recv.on_recv(
            mac,
            raw,
            |_| panic!("unexpected connect"),
            |_| panic!("unexpected disconnect"),
            |m| messages.push(m),
            |_| panic!(),
        )
        .map(|(_, a)| a)
    }

    #[test]
    fn integration_single_message_round_trip() {
        // Establish: use sender helpers from test_utils
        let mut sender = make_sender();
        sender.aimd.cwnd = MAX_WINDOW as u8;

        let mut recv = make_connected(some_mac(), 0);

        // Push a short message through the sender.
        sender.push_message(b"hello world");
        let mut raw_frames = Vec::new();
        {
            let mut data = std::ptr::null();
            let mut len = 0u16;
            while sender.send_next(&mut data, &mut len) {
                raw_frames.push(unsafe { std::slice::from_raw_parts(data, len as usize) }.to_vec());
            }
        }

        let mut messages = Vec::new();
        let mut final_ack = None;
        for raw in &raw_frames {
            final_ack = feed_raw(&mut recv, some_mac(), raw, &mut messages);
        }

        // If N-frame ACK didn't fire, force the timer.
        if final_ack.is_none() {
            let (_, ack_bytes) = recv.on_timeout();
            final_ack = Some(ack_bytes);
        }

        let ack = final_ack.unwrap();
        sender.on_recv_ack(&ack);

        assert_eq!(messages, vec![b"hello world".to_vec()]);
        assert_eq!(
            sender.send_base, sender.seq_send,
            "all frames must be ACKed"
        );
    }

    #[test]
    fn integration_selective_retransmit_fills_gap() {
        let mut sender = make_sender();
        sender.aimd.cwnd = MAX_WINDOW as u8;

        let mut recv = make_connected(some_mac(), 0);

        // Push three single-frame messages.
        sender.push_message(b"msg0");
        sender.push_message(b"msg1");
        sender.push_message(b"msg2");

        let mut frames: Vec<Vec<u8>> = Vec::new();
        {
            let mut data = std::ptr::null();
            let mut len = 0u16;
            while sender.send_next(&mut data, &mut len) {
                frames.push(unsafe { std::slice::from_raw_parts(data, len as usize) }.to_vec());
            }
        }
        assert_eq!(frames.len(), 3);

        let mut messages = Vec::new();

        // Deliver frames 0 and 2, drop frame 1.
        feed_raw(&mut recv, some_mac(), &frames[0], &mut messages);
        feed_raw(&mut recv, some_mac(), &frames[2], &mut messages);

        // msg0 should have been delivered (seq=1 after establish seq=0).
        // msg1 and msg2 are stuck behind the gap at seq=2.
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0], b"msg0");

        // Flush ACK so the sender knows about the gap.
        let (_, ack_bytes) = recv.on_timeout();
        let parsed = AckFrame::parse(&ack_bytes, compute_crc_raw).unwrap();
        // Bit for the frame at offset 1 from recv_base should be set (frame 2 received),
        // bit 0 (frame 1 / "msg1") should be clear.
        assert_eq!(parsed.bitmap & 1, 0, "gap frame must be flagged as missing");
        assert_ne!(
            parsed.bitmap & 2,
            0,
            "frame beyond gap must show as received"
        );

        sender.on_recv_ack(&ack_bytes);

        // Now deliver frame 1 (gap fill).
        feed_raw(&mut recv, some_mac(), &frames[1], &mut messages);
        // msg1 and msg2 should now drain.
        assert_eq!(messages.len(), 3);
        assert_eq!(messages[1], b"msg1");
        assert_eq!(messages[2], b"msg2");
    }

    #[test]
    fn integration_large_transfer_all_messages_delivered() {
        let mut sender = make_sender();
        sender.aimd.cwnd = MAX_WINDOW as u8;

        let mut recv = make_connected(some_mac(), 0);

        let total = 50usize;
        let payloads: Vec<Vec<u8>> = (0..total)
            .map(|i| format!("message{:03}", i).into_bytes())
            .collect();

        let mut all_messages: Vec<Vec<u8>> = Vec::new();

        for payload in &payloads {
            sender.aimd.cwnd = MAX_WINDOW as u8; // keep window pinned for delivery test
            sender.push_message(payload);

            let mut data = std::ptr::null();
            let mut len = 0u16;
            while sender.send_next(&mut data, &mut len) {
                let raw = unsafe { std::slice::from_raw_parts(data, len as usize) }.to_vec();
                let ack = feed_raw(&mut recv, some_mac(), &raw, &mut all_messages);
                if let Some(ack_bytes) = ack {
                    sender.on_recv_ack(&ack_bytes);
                }
            }

            // Flush any pending delayed ACK.
            let (_, ack_bytes) = recv.on_timeout();
            sender.on_recv_ack(&ack_bytes);
        }

        assert_eq!(all_messages.len(), total, "all messages must be delivered");
        for (i, msg) in all_messages.iter().enumerate() {
            assert_eq!(
                msg.as_slice(),
                format!("message{:03}", i).as_bytes(),
                "message {} payload mismatch",
                i
            );
        }
    }
}
