//! A deterministic PRNG the whole workspace shares.
//!
//! The `gold` profile needs a run to replay tick-for-tick from a checkpoint and
//! an intent log (spec §6.5). That rules out any generator whose algorithm may
//! change under a dependency bump, so the driver carries its own SplitMix64:
//! 64 bits of state, trivially checkpointable, identical on every platform.

/// SplitMix64. Small, fast, and stable across versions by construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub const fn new(seed: u64) -> Self {
        Rng { state: seed }
    }

    /// Restore from a checkpoint.
    pub const fn from_state(state: u64) -> Self {
        Rng { state }
    }

    /// The value to write into a checkpoint.
    pub const fn state(self) -> u64 {
        self.state
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in `0..n`. Returns 0 when `n` is 0 rather than dividing by it.
    pub fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            return 0;
        }
        // Lemire's multiply-shift. Biased by at most 2^-64 for our range sizes,
        // which no ABM will notice and which stays deterministic.
        ((self.next_u64() as u128 * n as u128) >> 64) as u64
    }

    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Pick an element, or `None` from an empty slice.
    pub fn choose<'a, T>(&mut self, items: &'a [T]) -> Option<&'a T> {
        if items.is_empty() {
            None
        } else {
            items.get(self.below(items.len() as u64) as usize)
        }
    }

    /// A generator seeded from this one and a cell id, so per-cell work is
    /// reproducible regardless of the order cells are visited in.
    pub fn for_cell(self, cell: u64) -> Rng {
        let mut mixer = Rng::new(self.state ^ cell.wrapping_mul(0x9e37_79b9_7f4a_7c15));
        Rng::new(mixer.next_u64())
    }
}

impl Default for Rng {
    fn default() -> Self {
        Rng::new(0x5eed_1234_5678_9abc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_gives_the_same_stream() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        let mut c = Rng::new(43);
        let from_a: Vec<u64> = (0..8).map(|_| a.next_u64()).collect();
        let from_b: Vec<u64> = (0..8).map(|_| b.next_u64()).collect();
        let from_c: Vec<u64> = (0..8).map(|_| c.next_u64()).collect();
        assert_eq!(from_a, from_b);
        assert_ne!(from_a, from_c);
    }

    #[test]
    fn a_checkpointed_generator_resumes_mid_stream() {
        let mut live = Rng::new(7);
        for _ in 0..100 {
            live.next_u64();
        }
        let mut restored = Rng::from_state(live.state());
        assert_eq!(live.next_u64(), restored.next_u64());
    }

    #[test]
    fn bounded_draws_stay_in_range() {
        let mut rng = Rng::new(9);
        for _ in 0..10_000 {
            assert!(rng.below(6) < 6);
            let u = rng.unit();
            assert!((0.0..1.0).contains(&u));
        }
        assert_eq!(rng.below(0), 0);
        assert_eq!(rng.choose::<u8>(&[]), None);
    }

    #[test]
    fn per_cell_streams_are_independent_of_visit_order() {
        let parent = Rng::new(1234);
        let first = parent.for_cell(0xabc).state();
        let second = parent.for_cell(0xdef).state();
        assert_ne!(first, second);
        assert_eq!(first, parent.for_cell(0xabc).state());
    }
}
