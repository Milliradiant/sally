# Developing sally

What sally is made of and how it is built. The README is for someone who uses sally; this page is for working on it.
The working notes (the plan, the design, what was learned, how a change is checked, the driver's and the panel's own
pages) are kept outside the public repository: a comment in the code that names a file under `docs/` other than this
page and the runtime's protocol means one of those.

sally is built on [libsurvive](https://github.com/cntools/libsurvive)'s front half (report parsing, OOTX, LH2 sweep
decoding, device sensor models), ported to Rust, and replaces the back half: the base station map,
the tracker, the outputs, and the SteamVR side.


What sally is not: it never touches the display side (for a lighthouse headset it only replaces the pose), it does not
integrate SlimeVR, and it does not replace pairing, firmware updates or base station power management (Valve's driver
does those).

## How it fits together

```
Valve's lighthouse driver + sally driver (in vrserver)      sally runtime (sally run)                panel
Valve runs the dongles as usual                              HID tap: reads the dongles read-only
sally driver swaps Valve's poses for sally's,        <---    decode, live map solve, tracker   --->   live compare vs Valve,
  Valve's pose as fallback, switches blended          --->    poses + diagnostics on UDP 9100         3D room, parameters
  Valve's own poses forwarded for the compare
```

## Status

- **Input**: the HID tap (`--hidtap 1`) reads the dongles next to Valve's driver, no USBPcap, no admin rights. Its
  recordings replay through the same parser (`--hidtap-playback`); old USBPcap captures and openhouse's recordings
  replay once `sally convert` has turned them into tap recordings. A recording is text (`.hidtap`), or the same
  text packed to a fraction of its size (`.hidtapz`, [below](#tap-recordings)); everything that reads a recording
  reads either.
- **Map**: solved live from every device's light, persisted between sessions (`--map-solve 1`), moved stations
  relocated, per-station corrections to the factory calibration; the calibration is the frame anchor, made by `sally
  calibrate` from a recording of any room (the stations placed from nothing, levelled to gravity by the devices'
  accelerometers) and kept up by the runtime itself, session by session.
- **Tracking**: the tracker: one IMU-driven estimator per device, every light
  hit applied as a measurement at its own time, each device's size learned and carried between sessions
  (`crates/sally-track`).
- **SteamVR**: the driver: pose override only, Valve as per-device fallback, starts the runtime itself; the output
  layer (smoothing, prediction and offsets per device class, all off by default).
- **Panel**: a desktop panel, and the same panel as a SteamVR dashboard overlay: what the runtime is doing per device
  and per station, a page per device, the live comparison against Valve, recordings replayed.
- **Diagnosis**: per device every 0.5 s, the cause of a problem (light, reflections, radio, base station, device), live
  and on replays; the panel's Check: a stretch of the session checked per device and made into one file to send,
  every serial in it a stand-in (`crates/sally-check`).
- **Hardware**: tested with VIVE Tracker 3.0, Index controllers and Tundra trackers on LH2 base stations (up to four),
  and the Index headset. Untried, and so "might work": other lighthouse trackers and controllers,
  other lighthouse headsets, more than four stations. LH1 base stations are not supported: sally says so in its log
  and leaves the tracking to Valve's driver. sally only reads the devices: it writes no firmware and nothing a device
  or a base station keeps.

## The programs

All of sally is Rust, one crate each under `crates/`:

| program | crate | what |
|---|---|---|
| `sally` | `sally` (with `sally-runtime`, `sally-track`, `sally-hid`, `sally-lighthouse`, `sally-map`, `sally-math`, `sally-json`, `sally-os`) | the runtime (`sally run`, which the SteamVR driver starts), `replay`, `calibrate`, `ctl` (the runtime's protocol), `setup` / `off` / `stop` / `copy-log` (SteamVR set up for sally and back), `convert` (USBPcap captures and openhouse recordings to tap recordings), `probe` (the dongles checked, their configs fetched) |
| `driver_sally.dll` | `sally-driver` | the SteamVR driver: sally's poses in place of Valve's, Valve's as the fallback |
| `sally-panel`, `sally-poses` | `sally-panel` | the panel and its SteamVR overlay; every SteamVR device's pose to a CSV |
| `sally-check` | `sally-check` | the check (what a session's devices sent, read per device for where trouble comes from) and the share (a session as one file to send, with stand-ins for every serial): a library the panel's Check tab runs; the program checks a session that came back |
| `sally-setup.exe` | `sally-setup` | the installer and uninstaller: one file that carries the others, puts them in the user's own folder and has `sally setup` put them on SteamVR's list |
| `sally-release` | `sally-release` | a release made from the tree: built without the build machine's paths, checked, the third-party notices, the project's folder laid out and the installer packed |
| `sally-bench` | `sally-bench` (not in the public repository) | the replay bench and its scorecard |

| path | what |
|---|---|
| `crates/` | the crates above |
| `docs/` | this page and the runtime's protocol (`sally_runtime_protocol.md`) |
| `README.md` | for someone who uses sally: what it is, installing, using and removing it |
| `LICENSE`, `licenses/` | sally's license with libsurvive's and jsmn's notices; the licenses of what else its source takes from (OpenVR, Python) |
| `Cargo.toml`, `rust-toolchain.toml`, `rustfmt.toml` | the workspace, the pinned Rust, the one style |

## Building

With the Rust in `rust-toolchain.toml`: `cargo build --release`, `cargo test`, `cargo clippy --all-targets -- -D
warnings`, `cargo fmt --check`. Replaying a tap recording, no hardware needed (the bench's replays add the devices'
configs and the station map to this, for poses):

```
cargo build --release -p sally
target/release/sally replay --hidtap-playback session.hidtap --configfile config.json --playback-factor 0
```

Windows, cross-compiled from Linux: `--target x86_64-pc-windows-gnullvm`, with llvm-mingw's
`x86_64-w64-mingw32-clang` as the linker (`CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER`, or `linker` under
`[target.x86_64-pc-windows-gnullvm]` in Cargo's config). Whatever links them, the programs import the math functions
they call from the Windows UCRT, as llvm-mingw's builds do: that is what keeps a Windows build's replays the same to
the bit. `cargo build --release --target x86_64-pc-windows-gnullvm` gives `sally.exe`,
`driver_sally.dll` (it goes in the driver folder `crates/sally-driver/package/` lays out),
`sally-panel.exe` and `sally-poses.exe` (OpenVR's `openvr_api.dll` beside them),
`sally-check.exe` and `sally-bench.exe`.

On the Windows PC `sally setup` (from the folder the runtime sits in, `survive\hidtap\` of the project) puts the
driver on SteamVR's list and writes its settings, SteamVR running or not; from then on starting SteamVR is all a
session needs. `sally off` turns it off again, and `sally uninstall` takes sally out of SteamVR altogether.

A release (the programs without the build machine's paths in them, the notices, the folder `sally setup` works from,
and the installer, `sally-setup.exe`, which is what someone else runs): `cargo run -p sally-release`.

## Tap recordings

A tap recording is every report the dongles sent, a line each (`crates/sally-hid/src/line.rs`): the PC's clock in
microseconds, the device the report came from, the report's bytes in hex; with the lines that say which device is on
which dongle. It is what `sally replay`, `sally calibrate`, the panel's Replay tab and the runtime's own calibration
read, about 2 GB for an hour of a full set of trackers.

**Packed** (`crates/sally-hid/src/pack.rs`, where the format is written down), the same recording is about a
fourteenth of that and unpacks to the text's own bytes: nothing is left out or rounded. The packer knows what a
report is made of (the half of a radio report that is left over from the one before, the IMU sample next to the last
ones, a sensor's light a rotation after its last) and writes only what could not be told ahead, through an adaptive
range coder. A recording is written packed when the file it is given (`--hidtap-record`) ends in `.hidtapz`, which is
the name `sally setup` gives a session's, in blocks of half a second, so one being written, or one whose writer was
stopped, reads up to its last whole block. Whether a recording is packed is read off its first bytes, not its name.

**Without what nothing reads** (`crates/sally-hid/src/strip.rs`). A radio dongle's report is one or two packets in
slots of 29 bytes; after a packet the dongle puts four bytes of its own, which follow from nothing, and the rest of
the slot is left over from earlier packets. The parser reads a packet to the length it gives, so those bytes are
read by nothing, and they were a third of a packed recording. The runtime writes its recording with zeros in their
place (`--hidtap-record-strip`, on unless set to 0): some 120 MB an hour, and a replay reads from it what the
session read, to the bit. That last part needs care: a packet that is not laid out as its flags say (one with no
payload, one cut short) is read past its end by libsurvive's parser, into those bytes. Such a report is kept whole;
only a packet that every reading stays inside is cut, and a test holds the parser to that on reports of every shape
(`strip.rs`, `the_parser_reads_nothing_that_is_left_out`). Anything that reads more of a report than the parser did
before must look there first.

**A packet is read to its own end and no further** (`--hidtap-past-end`, 2 unless set). What libsurvive reads past
it is the dongle's bytes: flags for a packet that has none, the end of an IMU sample or of a light entry, a battery
level. About one packet in 20,000 is such; the runtime stops the reading there, keeps what the packet had given up
to then, and says how many there were when the session ends. 1 only counts them, 0 reads on as libsurvive does.

```
sally pack [--strip] [--to <folder>] [--replace] [--force] <recording>...    x.hidtap -> x.hidtapz
sally unpack [--to <folder>] [--replace] [--force] <recording>...            x.hidtapz -> x.hidtap
```

The new file goes beside the old one or into `--to`'s folder, and is read back and compared before it counts.
`--replace` then removes the one it was made from. `--strip` leaves out what the runtime leaves out, from a text
recording or a packed one (with `--replace`, in its place). The packed format is at its second version (the zeros
after a packet said in a bit); the first is still read, and a build from before the second reads the first alone.
A build from before the packed form reads text alone: `sally unpack` gives it that.

## Upstream

sally is based on cntools/libsurvive at 44e9a29 and follows it loosely: upstream compatibility is not a constraint.
The C it kept is ported to Rust where sally ran it, so an upstream fix worth having is ported, not
cherry-picked.

```
git remote add upstream https://github.com/cntools/libsurvive.git
git fetch upstream
git log --oneline 44e9a29..upstream/master     # what is new upstream
```
