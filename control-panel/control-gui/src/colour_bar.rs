use slint::Image;

pub fn pixel_buffer(width: usize, grey: u8) -> Vec<u8> {
    vec![grey; width * 4]
}

pub fn pixel_buffer_to_image(buf: &[u8]) -> Image {
    let mut pixel_buf = slint::SharedPixelBuffer::<slint::Rgba8Pixel>::new(buf.len() as u32 / 4, 1);
    pixel_buf.make_mut_bytes().copy_from_slice(&buf);
    Image::from_rgba8(pixel_buf)
}

pub fn rasterize_range(buf: &mut [u8], colour: [u8; 4], lo: usize, hi: usize, total: usize) {
    let px_lo = lo * buf.len() / total / 4;
    let px_hi = hi * buf.len() / total / 4;

    if lo <= hi {
        for px in px_lo..px_hi {
            buf[(px * 4)..(px * 4 + 4)].copy_from_slice(&colour);
        }
    } else {
        for px in (px_lo + 0)..(buf.len() / 4) {
            buf[(px * 4)..(px * 4 + 4)].copy_from_slice(&colour);
        }

        for px in 0..px_hi {
            buf[(px * 4)..(px * 4 + 4)].copy_from_slice(&colour);
        }
    }
}

/// Rasterize filled ranges into a 1-row RGBA pixel buffer.
/// `ranges` – sorted, non-overlapping [start, end) intervals
/// `total`  – the domain max (i.e. your MAX)
/// `width`  – pixel width of the bar widget
pub fn rasterize_ranges(buf: &mut [u8], fill: [u8; 4], ranges: &[(u32, u32)], total: u32) {
    let w = buf.len() / 4;

    for px in 0..w {
        // Map pixel to domain interval [lo, hi)
        // Using u64 to avoid overflow in the multiply
        let lo = (px as u64 * total as u64 / w as u64) as u32;
        let hi = ((px as u64 + 1) * total as u64 / w as u64).min(total as u64) as u32;
        let hi = hi.max(lo + 1);

        // Coverage: fraction of [lo, hi) that is filled
        let domain_width = hi - lo;
        let covered = covered_in(ranges, lo, hi);

        // Threshold at 50% — communicates gaps/blocks clearly
        if covered * 2 >= domain_width {
            let base = px * 4;
            buf[base..base + 4].copy_from_slice(&fill);
        }
    }
}

/// Sum of covered length within [lo, hi) given sorted non-overlapping ranges.
fn covered_in(ranges: &[(u32, u32)], lo: u32, hi: u32) -> u32 {
    // Note that ranges are closed. [from, to]

    // Binary search to the first range that might overlap [lo, hi)
    let start = ranges.partition_point(|&(_, end)| end < lo);

    let mut covered = 0u32;
    for &(rlo, rhi) in &ranges[start..] {
        if rlo >= hi {
            break;
        }
        covered += (rhi + 1).min(hi) - rlo.max(lo);
    }
    covered
}
