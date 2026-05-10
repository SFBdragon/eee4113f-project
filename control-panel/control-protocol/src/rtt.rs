// ── RTT estimator (RFC 6298 s2) ───────────────────────────────────────────────

/// All times in milliseconds. Uses integer arithmetic — no floats.
/// SRTT and RTTVAR are stored scaled by 8 (<<3) to preserve sub-ms precision
/// without floating point, matching the RFC's recommended implementation.
#[derive(Debug)]
pub struct RttEstimator {
    est_rtt_x8: u32,
    var_rtt_x8: u32,
    rto: u32,
    min_rto: u32,
    max_rto: u32,
    pub have_sample: bool,
}

impl RttEstimator {
    pub const fn wifi() -> Self {
        Self::new(1000, 200, 4_000)
    }

    pub const fn lora() -> Self {
        Self::new(2000, 500, 16_000)
    }

    pub const fn new(initial_rto: u32, min_rto: u32, max_rto: u32) -> Self {
        Self {
            est_rtt_x8: 0,
            var_rtt_x8: 0,
            rto: initial_rto,
            min_rto,
            max_rto,
            have_sample: false,
        }
    }

    pub fn update(&mut self, rtt: u32) {
        if !self.have_sample {
            // RFC 6298 s2.2 first sample
            self.est_rtt_x8 = rtt << 3;
            self.var_rtt_x8 = (rtt << 3) >> 1; // var_rtt_0 = rtt/2
            self.have_sample = true;
        } else {
            let est_rtt = self.est_rtt_x8 >> 3;
            let delta_x8 = if rtt > est_rtt {
                rtt - est_rtt
            } else {
                est_rtt - rtt
            } << 3;
            // VARRTT = (1 - 1/4)*VARRTT + 1/4*|ESTRTT - RTT|
            self.var_rtt_x8 = self.var_rtt_x8 - (self.var_rtt_x8 >> 2) + (delta_x8 >> 2);
            // ESTRTT = (1 - 1/8)*ESTRTT + 1/8*RTT
            self.est_rtt_x8 = self.est_rtt_x8 - (self.est_rtt_x8 >> 3) + rtt;
        }
        self.recompute_rto();
    }

    pub fn on_timeout(&mut self) {
        // RFC 6298 s5.5: back off RTO
        self.rto = (self.rto * 2).min(self.max_rto);
    }

    pub fn recompute_rto(&mut self) {
        // RTO = SRTT + max(G, 4*RTTVAR); G = clock granularity (we use 1ms)
        let srtt = self.est_rtt_x8 >> 3;
        let rttvar = self.var_rtt_x8 >> 3;
        let rto = srtt + (rttvar << 2).max(1);
        self.rto = rto.clamp(self.min_rto, self.max_rto);
    }

    #[inline]
    pub fn rto(&self) -> u32 {
        self.rto
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // RTT estimator
    // =========================================================================

    #[test]
    fn rtt_first_sample_sets_srtt_and_rttvar() {
        let mut rtt = RttEstimator::wifi();
        assert!(!rtt.have_sample);
        rtt.update(100);
        assert!(rtt.have_sample);
        // RFC 6298 s2.2: SRTT = R, stored x8
        assert_eq!(rtt.est_rtt_x8, 800);
        // RTTVAR = R/2, stored x8
        assert_eq!(rtt.var_rtt_x8, 400);
    }

    #[test]
    fn rtt_rto_clamps_to_min() {
        let mut rtt = RttEstimator::wifi();
        rtt.update(1); // very short RTT
        assert!(rtt.rto() >= rtt.min_rto);
    }

    #[test]
    fn rtt_rto_clamps_to_max_on_backoff() {
        let mut rtt = RttEstimator::wifi();
        rtt.update(100);
        for _ in 0..20 {
            rtt.on_timeout();
        }
        assert_eq!(rtt.rto(), rtt.max_rto);
    }

    #[test]
    fn rtt_backoff_doubles_rto() {
        let mut rtt = RttEstimator::wifi();
        rtt.update(200);
        let before = rtt.rto();
        rtt.on_timeout();
        assert_eq!(rtt.rto(), (before * 2).min(rtt.max_rto));
    }

    #[test]
    fn rtt_subsequent_samples_converge() {
        let mut rtt = RttEstimator::wifi();
        for _ in 0..20 {
            rtt.update(100);
        }
        assert!(rtt.rto() < 1_000);
        assert!(rtt.rto() >= rtt.min_rto);
    }

    #[test]
    fn rtt_spike_increases_rto() {
        let mut rtt = RttEstimator::wifi();
        for _ in 0..10 {
            rtt.update(100);
        }
        let stable_rto = rtt.rto();
        rtt.update(2_000);
        assert!(rtt.rto() > stable_rto);
    }
}
