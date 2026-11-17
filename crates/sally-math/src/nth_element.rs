//! The selection the robust fits and the panel's error percentiles use: after `nth_element(v, nth)`, `v[nth]` is the
//! value a full sort would put there (the nth smallest), and only that value is read. Rust's own selection
//! (`select_nth_unstable_by`) with `f64::total_cmp`, so NaNs order after every number instead of comparing false both
//! ways.
//!
//! For values without a NaN the selected value is the one C++'s std::nth_element selects; where NaNs land and how the
//! rest of the slice is left differ, and nothing reads those.

/// The nth smallest value of `v` moved to `v[nth]`; nothing for an empty slice or `nth` past its end
pub fn nth_element(v: &mut [f64], nth: usize) {
	if nth < v.len() {
		v.select_nth_unstable_by(nth, f64::total_cmp);
	}
}

#[cfg(test)]
mod tests {
	use super::nth_element;

	fn nth_sorted(v: &[f64], nth: usize) -> f64 {
		let mut s = v.to_vec();
		s.sort_by(f64::total_cmp);
		s[nth]
	}

	#[test]
	fn selects_the_nth_smallest() {
		let mut x: u64 = 0x2545_f491_4f6c_dd1d;
		for len in 1..200usize {
			let v: Vec<f64> = (0..len)
				.map(|_| {
					x ^= x << 13;
					x ^= x >> 7;
					x ^= x << 17;
					// few distinct values, so ties are common
					(x % 37) as f64 * 0.25
				})
				.collect();
			for nth in [0, len / 2, (len as f64 * 0.7) as usize, (len as f64 * 0.9) as usize, len - 1] {
				let mut w = v.clone();
				nth_element(&mut w, nth);
				assert_eq!(w[nth], nth_sorted(&v, nth));
			}
		}
	}

	#[test]
	fn nans_and_edges_do_not_panic() {
		let mut v = vec![3.0, f64::NAN, 1.0, f64::NAN, 2.0];
		nth_element(&mut v, 1);
		assert_eq!(v[1], 2.0);
		let mut e: Vec<f64> = Vec::new();
		nth_element(&mut e, 0);
		let mut one = vec![5.0];
		nth_element(&mut one, 1);
		assert_eq!(one, vec![5.0]);
	}
}
