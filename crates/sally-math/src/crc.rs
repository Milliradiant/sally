//! CRC-32 (IEEE, zip's and zlib's: polynomial 0xedb88320, bits reflected, the register inverted going in and coming
//! out). The packed files' blocks, the zip files sally reads and writes and the base stations' OOTX packets are all
//! checked with it.

const TABLE: [u32; 256] = {
	let mut table = [0u32; 256];
	let mut i = 0;
	while i < 256 {
		let mut c = i as u32;
		let mut k = 0;
		while k < 8 {
			c = if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 };
			k += 1;
		}
		table[i] = c;
		i += 1;
	}
	table
};

/// The CRC-32 of `bytes` (0 for none)
pub fn crc32(bytes: &[u8]) -> u32 {
	!bytes.iter().fold(!0u32, |c, &b| TABLE[((c ^ b as u32) & 0xff) as usize] ^ (c >> 8))
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_checksum_is_zips() {
		assert_eq!(crc32(b""), 0);
		assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
		// the table is crc32.c's: its first entries and its last
		assert_eq!(TABLE[..4], [0x0000_0000, 0x7707_3096, 0xee0e_612c, 0x9909_51ba]);
		assert_eq!(TABLE[255], 0x2d02_ef8d);
	}
}
