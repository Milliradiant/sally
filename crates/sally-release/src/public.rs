//! `sally-release --public <folder> --names <file>`: the tree as the public repository gets it, written into a
//! folder that is empty or holds a `.git` alone (the public repository's checkout, its files taken out). What is
//! private about the tree is all in the names file, which is itself never part of what is written.
//!
//! 1. The tree's tracked files, as git lists them, but for what the names file leaves out. The tree has to be as its
//!    commit is: a file changed since is an error.
//! 2. Every device id swapped for its public one, in the files' text and in their names: the names file's pairs,
//!    each id in upper and in lower case wherever its digits stand as a word of their own (after `LHR-` or `0x`
//!    too). An id of a device's shape the file does not know (`LHR-` or `LHB-` and eight hex digits, a dongle's ten
//!    and its three characters) gets a random public one, added to the file; one the file lists alone on a line is
//!    made up and stays.
//! 3. Cargo.lock written again for the crates that are left (`cargo metadata` there).
//! 4. What was written read through for what must not be in it: an id's private form, and the texts the names file
//!    marks with `!`. Any found is an error.
//!
//! The tests, clippy, the installer from the export and the commit are the steps after it.

use std::collections::{BTreeMap, BTreeSet};
use std::hash::{BuildHasher, Hasher};
use std::path::{Path, PathBuf};
use std::process::Command;

/// The names file: `real public` a line for an id and its public name; an id alone on a line for one that is made
/// up; `!text` for a text the public tree must not hold; `-path` for a file left out, or a folder and all in it
/// where the path ends in `/`, and `+path` for a file kept all the same; `#` starts a comment
#[derive(Debug, Default, PartialEq)]
pub struct Names {
	/// real id, public id: the digits alone, in upper case
	pub pairs: BTreeMap<String, String>,
	pub made_up: BTreeSet<String>,
	pub never: Vec<String>,
	pub left_out: Vec<String>,
	pub kept: Vec<String>,
}

impl Names {
	pub fn parse(text: &str) -> Result<Names, String> {
		let mut n = Names::default();
		for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
			let marked = [('!', &mut n.never), ('-', &mut n.left_out), ('+', &mut n.kept)]
				.into_iter()
				.find_map(|(mark, list)| line.strip_prefix(mark).map(|t| list.push(t.trim().to_string())));
			if marked.is_some() {
				continue;
			}
			let words: Vec<&str> = line.split_whitespace().collect();
			match words[..] {
				[real, public] if real.len() == public.len() => {
					n.pairs.insert(real.to_ascii_uppercase(), public.to_ascii_uppercase());
				}
				[id] => {
					n.made_up.insert(id.to_ascii_uppercase());
				}
				_ => return Err(format!("not a line of a names file: {line}")),
			}
		}
		Ok(n)
	}

	/// Whether a tracked file (its path in the tree, `/` between its parts) stays private
	pub fn private(&self, path: &str) -> bool {
		!self.kept.iter().any(|k| k == path)
			&& self.left_out.iter().any(|l| l == path || (l.ends_with('/') && path.starts_with(l.as_str())))
	}

	/// Whether an id (its digits, upper case) is one the file has: a real one, a public one or a made-up one
	fn knows(&self, id: &str) -> bool {
		self.pairs.contains_key(id) || self.made_up.contains(id) || self.pairs.values().any(|p| p == id)
	}
}

fn word(c: u8) -> bool {
	c.is_ascii_alphanumeric()
}

fn hex(c: u8) -> bool {
	c.is_ascii_digit() || (b'a'..=b'f').contains(&c.to_ascii_lowercase())
}

/// Whether the text has a word of its own at `at`, `len` long: no letter or digit before it (but the `0x` of a hex
/// number) nor after it
fn alone(text: &[u8], at: usize, len: usize) -> bool {
	let before = at == 0
		|| !word(text[at - 1])
		|| (text[at - 1].eq_ignore_ascii_case(&b'x')
			&& at >= 2
			&& text[at - 2] == b'0'
			&& (at == 2 || !word(text[at - 3])));
	before && text.get(at + len).is_none_or(|&c| !word(c))
}

/// The text with every id of `pairs` swapped for its public one: in upper case and in lower, wherever its digits
/// stand alone
pub fn swap(text: &[u8], pairs: &BTreeMap<String, String>) -> Vec<u8> {
	// by length, so that a text is walked once for all the ids of one length
	let mut by_len: BTreeMap<usize, BTreeMap<Vec<u8>, Vec<u8>>> = BTreeMap::new();
	for (real, public) in pairs {
		let of = by_len.entry(real.len()).or_default();
		// (an id of digits alone reads the same in both cases: its public one is then in upper case)
		of.insert(real.to_ascii_lowercase().into_bytes(), public.to_ascii_lowercase().into_bytes());
		of.insert(real.as_bytes().to_vec(), public.as_bytes().to_vec());
	}
	let mut out = text.to_vec();
	for (&len, of) in &by_len {
		let mut at = 0;
		while at + len <= out.len() {
			match of.get(&out[at..at + len]).filter(|_| alone(&out, at, len)) {
				Some(public) => {
					out[at..at + len].copy_from_slice(public);
					at += len;
				}
				None => at += 1,
			}
		}
	}
	out
}

/// The ids of a device's shape in the text, their digits in upper case: `LHR-` or `LHB-` and eight hex digits, a
/// dongle's ten hex digits before `-` and three letters or digits
pub fn ids(text: &[u8]) -> BTreeSet<String> {
	let mut found = BTreeSet::new();
	let upper = |b: &[u8]| String::from_utf8_lossy(b).to_ascii_uppercase();
	for at in 0..text.len() {
		let rest = &text[at..];
		if rest.len() >= 12
			&& (rest[..4].eq_ignore_ascii_case(b"LHR-") || rest[..4].eq_ignore_ascii_case(b"LHB-"))
			&& rest[4..12].iter().all(|&c| hex(c))
			&& rest.get(12).is_none_or(|&c| !word(c))
		{
			found.insert(upper(&rest[4..12]));
		}
		if rest.len() >= 14
			&& (at == 0 || !word(text[at - 1]))
			&& rest[..10].iter().all(|&c| hex(c))
			&& rest[10] == b'-'
			&& rest[11..14].iter().all(|&c| word(c))
			&& rest.get(14).is_none_or(|&c| !word(c))
		{
			found.insert(upper(&rest[..10]));
		}
	}
	found
}

/// A random id of the shape of `like`: hex digits, in upper case
fn random_like(like: &str, taken: &dyn Fn(&str) -> bool) -> String {
	loop {
		let mut id = String::new();
		while id.len() < like.len() {
			let h = std::collections::hash_map::RandomState::new().build_hasher().finish();
			id.push_str(&format!("{h:016X}"));
		}
		id.truncate(like.len());
		if !taken(&id) {
			return id;
		}
	}
}

/// Where a text of `needles` stands in `text`, whatever its case: the first found
fn holds<'a>(text: &[u8], needles: &'a [String]) -> Option<&'a str> {
	let lower = text.to_ascii_lowercase();
	needles
		.iter()
		.find(|n| {
			let n = n.to_ascii_lowercase();
			!n.is_empty() && lower.windows(n.len()).any(|w| w == n.as_bytes())
		})
		.map(String::as_str)
}

fn git(root: &Path, args: &[&str]) -> Result<Vec<u8>, String> {
	let o = Command::new("git").current_dir(root).args(args).output().map_err(|e| format!("cannot run git: {e}"))?;
	if !o.status.success() {
		return Err(format!("git {}: {}", args.join(" "), String::from_utf8_lossy(&o.stderr).trim()));
	}
	Ok(o.stdout)
}

/// The export of the tree at `root` into `to`, by the names file `names_file`
pub fn export(root: &Path, to: &Path, names_file: &Path) -> Result<(), String> {
	let changed = git(root, &["status", "--porcelain"])?;
	if !changed.is_empty() {
		return Err(format!(
			"the tree is not as its commit is:\n{}the export is made from a commit",
			String::from_utf8_lossy(&changed)
		));
	}
	std::fs::create_dir_all(to).map_err(|e| format!("cannot make {}: {e}", to.display()))?;
	let there: Vec<_> = std::fs::read_dir(to)
		.map_err(|e| format!("cannot read {}: {e}", to.display()))?
		.flatten()
		.map(|e| e.file_name())
		.filter(|n| n != ".git")
		.collect();
	if !there.is_empty() {
		return Err(format!("{} is not empty: {} and {} more", to.display(), there[0].display(), there.len() - 1));
	}

	let listed = git(root, &["ls-files", "-z"])?;
	let shown = names_file.display();
	let names_text = std::fs::read_to_string(names_file).map_err(|e| format!("cannot read {shown}: {e}"))?;
	let mut names = Names::parse(&names_text)?;
	// the names file is the private tree's own: where it is in the tree, it has to be among what it leaves out
	let absolute = |p: &Path| std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
	if let Ok(inside) = absolute(names_file).strip_prefix(absolute(root)) {
		let inside = inside.to_string_lossy().replace('\\', "/");
		if !names.private(&inside) {
			return Err(format!("{shown} does not leave itself out (a line `-{inside}`)"));
		}
	}
	let files: Vec<String> = listed
		.split(|&c| c == 0)
		.filter(|f| !f.is_empty())
		.map(|f| String::from_utf8_lossy(f).into_owned())
		.filter(|f| !names.private(f))
		.collect();

	// the ids the names file has not met: a public name for each, kept in the file
	let mut texts: Vec<(String, Vec<u8>)> = Vec::new();
	let mut new = BTreeSet::new();
	for f in &files {
		let bytes = std::fs::read(root.join(f)).map_err(|e| format!("cannot read {f}: {e}"))?;
		new.extend(ids(&bytes).into_iter().chain(ids(f.as_bytes())).filter(|id| !names.knows(id)));
		texts.push((f.clone(), bytes));
	}
	if !new.is_empty() {
		let mut added = String::new();
		for id in &new {
			let public = random_like(id, &|p| names.knows(p) || new.contains(p));
			added.push_str(&format!("{id} {public}\n"));
			names.pairs.insert(id.clone(), public);
		}
		let mut text = names_text.clone();
		if !text.ends_with('\n') {
			text.push('\n');
		}
		std::fs::write(names_file, text + &added).map_err(|e| format!("cannot write {shown}: {e}"))?;
		println!("{} ids the names file had not met, each given a public one and added to {shown}:", new.len());
		print!("{added}");
		println!("(one that is made up belongs alone on its line there; commit the file, then export again)");
		return Err("the names file changed: the export is made from a commit".into());
	}

	let mut swapped = 0;
	for (f, bytes) in &texts {
		let out = swap(bytes, &names.pairs);
		swapped += usize::from(out != *bytes);
		let path = to.join(String::from_utf8_lossy(&swap(f.as_bytes(), &names.pairs)).as_ref());
		if let Some(dir) = path.parent() {
			std::fs::create_dir_all(dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
		}
		std::fs::write(&path, out).map_err(|e| format!("cannot write {}: {e}", path.display()))?;
		#[cfg(unix)]
		{
			let mode = std::fs::metadata(root.join(f)).map_err(|e| format!("{f}: {e}"))?.permissions();
			std::fs::set_permissions(&path, mode).map_err(|e| format!("{}: {e}", path.display()))?;
		}
	}

	// Cargo.lock for the crates that are left
	let o = super::cargo()
		.current_dir(to)
		.args(["metadata", "--format-version", "1"])
		.output()
		.map_err(|e| format!("cannot run cargo: {e}"))?;
	if !o.status.success() {
		return Err(format!("cargo metadata in the export: {}", String::from_utf8_lossy(&o.stderr).trim()));
	}

	// nothing private left in what was written
	let mut never: Vec<String> = names.pairs.keys().cloned().collect();
	never.extend(names.never.iter().cloned());
	let mut left = Vec::new();
	let mut written: Vec<PathBuf> = Vec::new();
	walk(to, &mut written)?;
	for path in &written {
		let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
		let name = path.strip_prefix(to).unwrap_or(path).to_string_lossy().into_owned();
		if let Some(n) = holds(&bytes, &never).or_else(|| holds(name.as_bytes(), &never)) {
			left.push(format!("{name}: {n}"));
		}
	}
	if !left.is_empty() {
		return Err(format!("the export holds what it must not:\n{}", left.join("\n")));
	}
	println!(
		"the public tree: {} ({} files, device ids swapped in {swapped}, {} left out as private)",
		to.display(),
		written.len(),
		listed.split(|&c| c == 0).filter(|f| !f.is_empty()).count() - files.len()
	);
	Ok(())
}

/// Every file under `dir`, but a `.git` at its top
fn walk(dir: &Path, into: &mut Vec<PathBuf>) -> Result<(), String> {
	for e in std::fs::read_dir(dir).map_err(|e| format!("cannot read {}: {e}", dir.display()))?.flatten() {
		let path = e.path();
		if e.file_name() == ".git" || e.file_name() == "target" {
			continue;
		}
		if path.is_dir() {
			walk(&path, into)?;
		} else {
			into.push(path);
		}
	}
	Ok(())
}

#[cfg(test)]
mod tests {
	use super::*;

	fn pairs(list: &[(&str, &str)]) -> BTreeMap<String, String> {
		list.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect()
	}

	#[test]
	fn what_stays_private() {
		let n = Names::parse("-NOTES.md\n-notes/\n+notes/guide.md\n-crates/tools/\n").unwrap();
		for f in ["NOTES.md", "notes/plan.md", "notes/old/guide.md", "crates/tools/src/main.rs"] {
			assert!(n.private(f), "{f}");
		}
		for f in ["README.md", "notes/guide.md", "notes.md", "more/NOTES.md", "crates/tool/src/main.rs", "crates/tools"]
		{
			assert!(!n.private(f), "{f}");
		}
	}

	#[test]
	fn the_names_file() {
		let n =
			Names::parse("# a comment\n1A2B3C4D 99887766\n\n0000000A\n!Some Folder\naabbccddee 0011223344\n").unwrap();
		assert_eq!(n.pairs, pairs(&[("1A2B3C4D", "99887766"), ("AABBCCDDEE", "0011223344")]));
		assert_eq!(n.made_up.iter().collect::<Vec<_>>(), ["0000000A"]);
		assert_eq!(n.never, ["Some Folder"]);
		assert!(n.left_out.is_empty() && n.kept.is_empty());
		assert!(n.knows("1A2B3C4D") && n.knows("99887766") && n.knows("0000000A") && !n.knows("12345678"));
		// a pair of two lengths, and a line of three words
		assert!(Names::parse("1A2B3C4D 998877\n").is_err());
		assert!(Names::parse("1A2B3C4D 99887766 x\n").is_err());
	}

	#[test]
	fn an_id_is_swapped_where_it_stands_alone() {
		let p = pairs(&[("1A2B3C4D", "99887766"), ("AABBCCDDEE", "0011223344")]);
		let s = |t: &str| String::from_utf8(swap(t.as_bytes(), &p)).unwrap();
		assert_eq!(s("LHR-1A2B3C4D and lhr-1a2b3c4d"), "LHR-99887766 and lhr-99887766");
		assert_eq!(s("0x1A2B3C4D, 1a2b3c4d; \"AABBCCDDEE-1YX\""), "0x99887766, 99887766; \"0011223344-1YX\"");
		assert_eq!(s("AABBCCDDEE_config.json"), "0011223344_config.json");
		// part of a longer word or number, of mixed case, or after an x that is not a hex number's
		for t in ["01A2B3C4D", "1A2B3C4D5", "x1A2B3C4D", "1a2B3c4D", "max1A2B3C4D", "a0x1A2B3C4D"] {
			assert_eq!(s(t), t);
		}
		assert_eq!(s("1A2B3C4D"), "99887766");
		// an id of digits alone has one case: its public one comes in upper case
		let p = pairs(&[("12345678", "AB12CD34")]);
		assert_eq!(swap(b"LHR-12345678 0x12345678", &p), b"LHR-AB12CD34 0xAB12CD34");
	}

	#[test]
	fn the_ids_of_a_devices_shape() {
		let t =
			b"LHR-1A2B3C4D lhb-00ff00ff \"AABBCCDDEE-1YX\" LHR-1A2B3C4D5 LHR-1A2B3C4 0123456789-ABCD xAABBCCDDEE-1YX";
		assert_eq!(ids(t).into_iter().collect::<Vec<_>>(), ["00FF00FF", "1A2B3C4D", "AABBCCDDEE"]);
	}

	#[test]
	fn a_random_id_has_the_shape_and_is_new() {
		let id = random_like("AABBCCDDEE", &|_| false);
		assert!(id.len() == 10 && id.bytes().all(|c| hex(c) && !c.is_ascii_lowercase()));
		assert_ne!(random_like("1A2B3C4D", &|_| false), random_like("1A2B3C4D", &|_| false));
	}

	#[test]
	fn what_must_not_be_there_is_found_in_any_case() {
		let never = vec!["1A2B3C4D".to_string(), "Some Folder".to_string()];
		assert_eq!(holds(b"at lhr-1a2b3c4d", &never), Some("1A2B3C4D"));
		assert_eq!(holds(b"in some folder\\logs", &never), Some("Some Folder"));
		assert_eq!(holds(b"LHR-99887766", &never), None);
	}
}
