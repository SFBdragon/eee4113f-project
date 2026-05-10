use control_core::protocol::app::StorageInfo;

pub struct RingBufferPoints {
    pub p_overwritable_1: f32,
    pub p_preserved_1: f32,
    pub p_free_2: f32,
    pub p_overwritable_2: f32,
    pub p_preserved_2: f32,
}

impl RingBufferPoints {
    pub fn from_storage_info(storage: &StorageInfo, is_full: bool) -> Self {
        // Regions to size:
        // [empty] [overwritable] [preserved] [empty] [overwritable] [preserved]
        //
        // Cases:
        // [empty] [overwritable] [preserved] [empty]
        //         [overwritable] [preserved] [empty] [overwritable]
        //                        [preserved] [empty] [overwritable] [preserved]

        let tot = storage.total_blocks as f64;
        let mut p_overwritable_1 = 0.0;
        let mut p_preserved_1 = 0.0;
        let p_free_2;
        let mut p_overwritable_2 = 1.0;
        let mut p_preserved_2 = 1.0;

        debug_assert!(storage.coherent());
        if is_full {
            debug_assert_eq!(storage.available_begin, storage.available_end);
            p_overwritable_1 = storage.available_begin as f64 / tot;
            p_preserved_1 = p_overwritable_1;
            p_free_2 = p_overwritable_1;
        } else {
            if storage.available_begin < storage.available_end {
                p_overwritable_1 = storage.available_begin as f64 / tot;
            }

            if storage.overwritable_end <= storage.available_end {
                p_preserved_1 = storage.overwritable_end as f64 / tot;
            }

            p_free_2 = storage.available_end as f64 / tot;

            if storage.available_end < storage.available_begin {
                p_overwritable_2 = storage.available_begin as f64 / tot;
            }

            if storage.available_end < storage.overwritable_end {
                p_preserved_2 = storage.overwritable_end as f64 / tot;
            }
        }

        Self {
            p_overwritable_1: p_overwritable_1 as f32,
            p_preserved_1: p_preserved_1 as f32,
            p_free_2: p_free_2 as f32,
            p_overwritable_2: p_overwritable_2 as f32,
            p_preserved_2: p_preserved_2 as f32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Case 1: Contiguous layout (No wrap)
    /// Logic: [0] --- [begin] --- [over_end] --- [avail_end] --- [total]
    #[test]
    fn test_contiguous_storage() {
        let storage = StorageInfo {
            total_blocks: 100,
            available_begin: 10,
            overwritable_end: 40,
            available_end: 80,
            generation: 0,
        };

        let res = RingBufferPoints::from_storage_info(&storage, false);

        // p_overwritable_1 should be 10/100 = 0.1
        assert_eq!(res.p_overwritable_1, 0.1);
        // p_preserved_1 should be 40/100 = 0.4
        assert_eq!(res.p_preserved_1, 0.4);
        // p_free_2 should be 80/100 = 0.8
        assert_eq!(res.p_free_2, 0.8);
        // Since avail_end (80) is NOT < begin or over_end, these default to 1.0
        assert_eq!(res.p_overwritable_2, 1.0);
        assert_eq!(res.p_preserved_2, 1.0);
    }

    /// Case 2: Wrapped layout
    /// Logic: [0] --- [over_end] --- [avail_end] --- [begin] --- [total]
    #[test]
    fn test_overwritable_wraps() {
        let storage = StorageInfo {
            total_blocks: 100,
            available_begin: 60,
            overwritable_end: 10,
            available_end: 20,
            generation: 0,
        };

        let res = RingBufferPoints::from_storage_info(&storage, false);

        assert_eq!(res.p_overwritable_1, 0.0);
        assert_eq!(res.p_preserved_1, 0.1);
        assert_eq!(res.p_free_2, 0.2);
        assert_eq!(res.p_overwritable_2, 0.6);
        assert_eq!(res.p_preserved_2, 1.0);
    }

    /// Case 2: Wrapped layout
    /// Logic: [0] --- [avail_end] --- [begin] --- [over_end] --- [total]
    #[test]
    fn test_readable_wraps() {
        let storage = StorageInfo {
            total_blocks: 100,
            available_begin: 60,
            overwritable_end: 80,
            available_end: 20,
            generation: 0,
        };

        let res = RingBufferPoints::from_storage_info(&storage, false);

        // avail_begin (60) is NOT < avail_end (20), so overwritable_1 remains 0.0
        assert_eq!(res.p_overwritable_1, 0.0);
        // over_end (80) is NOT < avail_end (20), so preserved_1 remains 0.0
        assert_eq!(res.p_preserved_1, 0.0);

        // p_free_2 is always avail_end / tot = 0.2
        assert_eq!(res.p_free_2, 0.2);

        // avail_end (20) < avail_begin (60), so p_overwritable_2 = 60/100 = 0.6
        assert_eq!(res.p_overwritable_2, 0.6);
        // avail_end (20) < over_end (80), so p_preserved_2 = 80/100 = 0.8
        assert_eq!(res.p_preserved_2, 0.8);
    }

    /// Case 3: Empty / Zero state
    #[test]
    fn test_zero_indices() {
        let storage = StorageInfo {
            total_blocks: 100,
            available_begin: 0,
            overwritable_end: 0,
            available_end: 0,
            generation: 0,
        };

        let res = RingBufferPoints::from_storage_info(&storage, false);

        assert_eq!(res.p_overwritable_1, 0.0);
        assert_eq!(res.p_preserved_1, 0.0);
        assert_eq!(res.p_free_2, 0.0);
        assert_eq!(res.p_overwritable_2, 1.0);
        assert_eq!(res.p_preserved_2, 1.0);
    }

    /// Case 4: Full Buffer (at the end)
    #[test]
    fn test_at_full_at_end() {
        let storage = StorageInfo {
            total_blocks: 100,
            available_begin: 100,
            overwritable_end: 100,
            available_end: 100,
            generation: 0,
        };

        let res = RingBufferPoints::from_storage_info(&storage, true);

        assert_eq!(res.p_overwritable_1, 1.0);
        assert_eq!(res.p_preserved_1, 1.0);
        assert_eq!(res.p_free_2, 1.0);
        assert_eq!(res.p_overwritable_2, 1.0);
        assert_eq!(res.p_preserved_2, 1.0);
    }

    /// Case 4: Full Buffer (at the end)
    #[test]
    fn test_at_middle() {
        let storage = StorageInfo {
            total_blocks: 100,
            available_begin: 50,
            overwritable_end: 50,
            available_end: 50,
            generation: 0,
        };

        let res = RingBufferPoints::from_storage_info(&storage, true);

        assert_eq!(res.p_overwritable_1, 0.5);
        assert_eq!(res.p_preserved_1, 0.5);
        assert_eq!(res.p_free_2, 0.5);
        assert_eq!(res.p_overwritable_2, 1.0);
        assert_eq!(res.p_preserved_2, 1.0);
    }
}
