# sally runtime protocol

Status: protocol 5, 2026-11-07 (protocol 4 plus `pair`; protocol 4, 2026-10-30, was protocol 3 plus the diagnosis `D` and `C` lines; protocol 3 was protocol 2 plus `sensors`, battery on `DEV` and `sub state`; protocol 2 was protocol 1 plus `Q`, the extra `DEV` fields and `quit`; nothing removed or reordered). Implements design doc step 1 (sections 4.4 and 4.5). The code is
`crates/sally-runtime/src/protocol` (the C runtime's driver_pose_udp.c, ported line for line: docs/checking.md); the
command line client is `sally ctl` (sallyctl.py, the reference client, until it left on 9 Nov; the same commands and
output).

## Turning it on

| option | default | meaning |
|---|---|---|
| `--runtime-port PORT` | 0 (off) | bind UDP `PORT` for subscriptions and control |
| `--runtime-bind ADDR` | 127.0.0.1 | address to bind. Anyone who can reach it can change parameters and write files, so keep it on loopback |
| `--runtime-state-hz HZ` | 100 | maximum `S` lines per device per second (0 = every pose) |
| `--runtime-diag-period S` | 1 | seconds of run time between `H`/`T`/`M` summaries |
| `--runtime-sub-timeout S` | 5 | a subscriber that has not re-sent `sub` or `ping` for this long is dropped |
| `--pose-udp PORT`, `--pose-udp-host H` | off | legacy push of `P`/`V`/`L` to H:PORT, unchanged from the first version |
| `--runtime-driver-file FILE` | off | the driver stream written to FILE: every `Q` and `L` line (and `P` for a device without `Q`), none dropped, with each device's `DEV` line (as `devices` gives it) before its first `Q`. What the SteamVR driver is sent, for the bench (`sally-bench output`, docs/bench.md) |

Either `--runtime-port` or `--pose-udp` loads the plugin; both can be on at once. `--runtime-driver-file` loads it
too, and on its own opens no port (a bench replay: nothing to subscribe, several at once). Run time below means
`survive_run_time` (seconds since the context started, playback time on a replay).

## Transport

UDP, one ASCII line per datagram, `\n` terminated, fields separated by single spaces. Every line starts with a
one-letter type (streams), `OK`/`ERR` (command results), or `PARAM`/`DEV`/`M` inside a multi-line reply.

Subscribe by sending `sub pose`, `sub diag` or `sub all` (plain `sub` = all) from the socket you want the streams on.
`sub driver` is the lean stream the SteamVR driver uses: `Q` and `L` lines only, plus `P` for a device that has had no
`Q` for 0.5 s (one datagram per report instead of four).
`sub state` (protocol 3) is the `S` lines alone; the panel takes `sub driver state diag`, which carries everything it
draws at about a third of the full pose stream's datagrams.
Repeat it (or `ping`) at least every `runtime-sub-timeout` seconds; a client that stops doing that is dropped, so a
crashed panel costs nothing. If the runtime restarts, the next `sub` re-attaches with no other handshake. Up to 16
subscribers; when full, the stalest one is replaced. A new subscriber gets the map (`L` and/or `M` lines) right away.

The socket is non-blocking: if a send would block the line is dropped and counted (shown at shutdown), the solver never
waits on a client.

## Pose stream (`sub pose`)

| line | when | fields |
|---|---|---|
| `P` | every report | codename serial timecode run_time wall_epoch x y z qw qx qy qz |
| `V` | every report | codename serial timecode run_time wall_epoch vx vy vz wx wy wz |
| `L` | station pose set or moved | bsd_idx station_id wall_epoch x y z qw qx qy qz |
| `Q` | every report with a velocity | codename serial timecode run_time x y z qw qx qy qz vx vy vz wx wy wz ax ay az lead dx dy dz dvx dvy dvz alx aly alz cx cy cz cqw cqx cqy cqz ret_age ret_gap |
| `S` | every report, at most `runtime-state-hz` | codename serial timecode run_time state light_age pos_sd rot_sd lh_mask res |

Frame: libsurvive world, metres, Z up, quaternions wxyz, velocities m/s and rad/s (axis-angle rate). `serial` is `-`
when unknown. `wall_epoch` is the Unix time when the line was formatted. `timecode` is the device clock in ticks of
`timebase_hz` (from `devices`), unwrapped to 64 bits.

The pose is the device's head (origin) point. The linear velocity is libsurvive's, which is the velocity of the IMU
point, not of the origin: for an Index controller the two differ by up to ~1 m/s while the wrist turns. A consumer
moves it with `v_origin = v_imu + w x r` (`r` = IMU to origin, from `head2imu` in `devices`). Angular velocity is in
the world frame.

`Q` (subscribers only) carries the pose and the velocity of the same tracker report, so nothing has to pair a `P` with
a `V`. A device whose filter does not report velocity (very first second, or `--use-kalman 0`) has `P` but no `Q`.
The ten fields after `wz` (since 2026-11-03) are the hand-over look-ahead (the tracker; all 0 without it,
`docs/sally_driver.md`): the state is on the gyro's time, but the accelerometer's newest samples belong up to `lead`
seconds past it (7 ms on trackers, 2.5 on controllers, 0 while coasting). Over that time the head point moved `dx dy
dz` and the IMU point's velocity changed `dvx dvy dvz` (world, measured: the samples integrated with the orientation
turned by `w`); `ax ay az` is the IMU point's acceleration at the end of the lead (world, gravity and bias out). The
pose and velocity fields before them are the state's own (the pose `lead` older than the look-ahead). The SteamVR
driver applies the look-ahead, declares the pose `lead` younger and carries the rest of its age with the acceleration.

Since 2026-11-04 the angular acceleration is in them too: the head point's acceleration is `a_imu + alpha x r + w x (w
x r)` (`r` = IMU point to head point), and a consumer that moves the IMU point's quantities to the head point the way
the driver does (`v + w x r`, `a + w x (w x r)`) had no `alpha`. So `ax ay az` carries `alpha x r` on top of the IMU
point's acceleration, `dvx dvy dvz` the change of `w x r` over the lead (`alpha x r` times the lead) and `dx dy dz` its
displacement (half of that times the lead), with `r` at the end of the lead: such a consumer then hands over the head
point as it moves. The last three fields, `alx aly alz`, are the angular acceleration itself (world, rad/s^2, the gyro's
newest change; 0 across a gap or with the gyro at full scale), for the orientation and the angular velocity; they are
not to be added to the head point again. Older consumers read the first 18 (or 28) fields and ignore the rest.

Since 2026-11-08 nine more, for the output layer (the SteamVR driver's, `docs/sally_driver.md` "The output layer"): what
the state's corrections did, apart from its motion, and the light's return. `cx cy cz` is the head point's corrections
summed since the device's slot began (m, world): each report adds the IMU point's correction and what the orientation's
correction did to the lever (`r` the IMU point to the head point as it stands: `dp + r - e^-1 r`). `cqw cqx cqy cqz` is
the orientation's corrections composed on the world side (the tracker's sum since it began; the same for every point of
the device). Neither moves with the motion, so what changes between two Q lines is that report's correction: a restart,
an update, a zero-velocity update. `ret_age` is how long ago the latest light return began (s, -1 before the first): the
first correction of the light (an update, or the restart when the coasted state does not fit) after no sweep at all
reached the tracker for 0.1 s or longer (the tracker's `RETURN_DARK`; a state the light disagrees with gets its sweeps
all along, and its restart is no return); `ret_gap` is how long the device was dark before it (s). Without a seeded
tracker the sums stand and `ret_age` is -1. Older consumers read the first 28 fields. Recordings carry each report's
correction for 0.6 s after a return (`TRACK_RETURN` lines at `--track-verbose 2`).

From the evening of 2026-11-04 to 2026-11-05 the runtime sent `alpha` as 0 (`HANDOVER_ALPHA` 0: on a shaken
controller the differenced gyro looked like noise as much as motion, findings.md section 8). It is on again since 5 Nov
(`HANDOVER_ALPHA` in crates/sally-track/src/tracker.rs, whose comment has why: that measurement compared the
velocity carried ahead with the path's at the state's time, and its session had a controller's radio holding its light
back), so `alpha` and the `alpha x r` parts of `ax ay az`, `dvx dvy dvz` and `dx dy dz` are the measured ones.

`S` fields:

- `state`: the tracker's state (`survive_core2_diag_state`, tracking.md 3c): `tracking` (light accepted within 0.1 s),
  `dark` (no accepted light for longer, not yet coasting), `coasting` (no accepted light for `track-gap-coast` s),
  `reacquiring` (lost, waiting for its seed solve, or not yet solved after a start), `none` (never solved).
- `light_age`: seconds since the tracker last accepted light, -1 if never.
- `pos_sd`: square root of the summed position variance (m^2), a 1-sigma radius; `rot_sd` is -1 (the tracker does not
  report it).
- `lh_mask`: hex bit mask of the stations whose sweeps the tracker got in the last 0.5 s.
- `res` (since 7 Nov 2026, an eleventh field older consumers do not read): the light's residual, mrad: the median
  |residual| of each sweep the tracker's update takes (its hits' angles against the pose's prediction, the recordings' TRACK_SWEEP
  median), smoothed 0.8 / 0.2 over the sweeps; -1 without a sweep in the last 0.5 s. How well the light fits the pose,
  where `pos_sd` is how uncertain the pose is (the panel shows both).

(Before 4 Nov the same fields came from the old tracker when the new tracker was off: its `kalman-*` coast and reacquire
rules, its claimed rotation sd and the stations of its latest solver pose.)

## Diagnostics stream (`sub diag`)

| line | when | fields |
|---|---|---|
| `R` | every solve | codename run_time solve_time outcome meas rms dropped_lh dropped_rms x y z n then n x (lh hits rms) |
| `H` | each diag period, per device per station with hits | codename lh period_s n_sensors count_0 ... count_{n-1} |
| `T` | each diag period, per device | codename run_time state light_age reported dropped obs single_lh single_lh_dropped coast_imu obs_gated solves_ok solves_rejected failed error_gate single_lh_gate jump_gate station_discard light_res max_err_gate few_meas_gate rot_gate settle_gate n then n x (lh res gated) |
| `M` | each diag period, per station | bsd_idx station_id mode position_set locked x y z qw qx qy qz |
| `F` | a station's seen/lost transition | bsd_idx seen\|lost run_time |
| `D` | each diagnosis window (0.5 s), per device | codename run_time headline severity bars loss_pct gap_ms stations sensors back_pct gated discards state light_age charge n_causes |
| `C` | after its `D` line, one per cause | codename run_time cause severity text... |

`R` (one per optimizer run, accepted or not; the optimizer was the old tracker's, so since 4 Nov no `R` lines are
sent, and with the new tracker on none were):

- `outcome`: `ok`, `failed` (optimizer status), `error_gate` (residual above the solver's limit), `single_lh_gate`
  (`mpfit-single-lh-max-err`), `jump_gate` (`mpfit-jump-gate-*`), `station_discard` (the re-solve without an
  inconsistent station landed more than `mpfit-station-gate-max-jump` from the filter), `max_err_gate`
  (`mpfit-max-err`), `few_meas_gate` (`mpfit-min-valid-meas`), `rot_gate` (removed, always 0), `settle_gate`
  (`mpfit-settle-time`: a first solve after a light gap resting on fewer stations than before it). The last four were
  added with the solver gates after protocol 1; on `T` lines their counts follow `light_res` so older parsers keep
  working, and `solves_rejected` includes them.
- `rms` and the per-station `rms` are radians, over that station's valid light measurements in the final fit.
  `hits` counts those measurements (both axes).
- `dropped_lh` / `dropped_rms`: the station the consistency gate removed and its residual in the first fit, -1 / 0 if
  none. When the gate tries a re-solve and reverts to the first fit, the per-station numbers are the first fit's.
- `x y z`: the solved pose (not the filtered one), zero for `failed`.

`T` keeps its fields so parsers keep working, but the old tracker's ones are zeros since 4 Nov (as they were with core
2 on): `reported dropped obs single_lh single_lh_dropped coast_imu obs_gated`, the solve counts (`solves_ok` ...
`settle_gate`, the old solver's), `light_res`, and the per-station list (`n` is 0). What is live on a `T` line is
`state` and `light_age`, as on `S`.

`H` counts angle events (both axes) per sensor from that station over `period_s`. It feeds the coverage heat map and
the per-device light display; a sensor with 0 got no light.

`T` counters `reported` through `obs_gated` are cumulative since the device connected (diff them). `single_lh_dropped`,
`obs_gated` and `rot_gate` are always 0 (those features were removed; the fields stay so positions do not move). The solve outcome
counts are for this period only. `light_res` is the tracker's running average (0.9/0.1 per update) of its per-hit
light update error; it stays 0 when the per-hit light model is not running. Protocol 4 appends the same per station:
`res` the running mean |innovation| (rad) of that station's hits in the per-hit model, `gated` the hits of it the
model's gate rejected (cumulative since the device connected); only stations the model has seen are listed. The
solver's own per-station fit residuals are on the `R` lines.

`F` is derived from `H`: a station is seen while any device got light from it during the period. "Contradicts the
map" (moved station) is design step 4 and not emitted yet.

`D` / `C` (protocol 4, design 9.2) are the diagnosis (`crates/sally-runtime/src/diagnosis.rs`, `--diagnosis 1` by
default): what is wrong with the device right now, and why. The same verdicts go into recordings as `DIAG` lines, so
the live view, the replay viewer and the bench read the same thing.

- `headline`: the worst cause, `fine` when there is none. Causes, root first (on equal severity the earlier one is
  the headline): `radio` (reports lost between the device and its dongle, or, since 4 Nov 2026, its light held back:
  the device's IMU stream 19 ms or more ahead of its light as it arrives, the median over 60 s, off under 15,
  severity 2 at 40;
  the text then says "its light comes N ms late" and the recording's DIAG token is `radio:1:late<ms>`), `station` (a base station it uses went
  dark for every device, disagrees with the map, or was just relocated), `device` (battery under 10 %, no reports for
  3 s, no sensor layout), `light` (no light for a while, or only one station sees it), `reflect` (hits from behind
  their sensor, hits the filter's gate rejected, stations the solver dropped while the device was slow; never above
  severity 1, since reflections that are present and filtered are the common case).
- `severity`: 0 fine, 1 hurting a little, 2 hurting.
- `bars`: radio signal 0-4 from `loss_pct` (reports missing against the device's own usual rate, averaged over the last 2 s: 4 under 1 %, 3 under
  5 %, 2 under 15 %, 1 under 40 %); `gap_ms` the longest silence in its IMU stream.
- `stations` / `sensors`: stations with 4+ hits and sensors lit in the window; `back_pct`: share of hits on sensors
  facing away from their station (only judged while the filter tracks); `gated`: hits the filter's per-hit gate
  rejected; `discards`: stations the solver dropped as inconsistent while the device moved slower than
  `diagnosis-discard-vmax`.
- `state`, `light_age`: as on `S`. `charge`: battery percent as the device reports it (0 when unknown).
- `C` text is the plain-language line for the user ("base station ch 9 went dark for every device"); it runs to the
  end of the line.

## Commands

Send a line, get the reply on the same socket (the panel, `sally-panel`, is the main client). Prefix a command with `#<id> ` and every reply line carries the same
prefix, so a client can tell replies from stream lines and from each other. Multi-line replies end with the `OK` line.

| command | reply |
|---|---|
| `hello` | `OK hello sally-runtime <protocol> <run_time> <devices> <stations>` |
| `help` | `OK help <command list>` |
| `ping` | `OK ping <run_time> subscribed\|not_subscribed` (refreshes the subscription) |
| `sub [pose\|diag\|all\|driver\|state]...` | `OK sub pose diag` |
| `unsub` | `OK unsub` |
| `get <name>` | `OK get <name> <type> <value> <bound>` |
| `set <name> <value>` | `OK set <name> <type> <old> <new> <bound>` |
| `params [prefix]` | `PARAM <name> <type> <value> <bound>` per parameter, then `OK params <n>` |
| `describe <name>` | `OK describe <name> <type> <default> <description...>` |
| `map` | `M` line per station, then `OK map locked\|unlocked <stations>` |
| `map lock` / `map unlock` | same, after setting `lock-lighthouses` |
| `devices` | `DEV <codename> <serial> <sensors> <state> <light_age> <subtype> <timebase_hz> <head2imu x y z qw qx qy qz> <charge> <charging> <dongle>` per device, then `OK devices <n>`; `dongle` is the USB serial of the radio the device talks through (hidtap), `-` when unknown |
| `sensors <codename>` | `SENS <codename> <index> <x y z> <nx ny nz>` per sensor (device frame, metres, unit normal), then `OK sensors <codename> <n>`; `n` is 0 until the device's config has been read |
| `pair <dongle> <device serial\|->` | `OK pair <dongle> <device>` (protocol 5): the dongle carries that device from now on, as SteamVR says (below) |
| `record` | `OK record recording\|idle` |
| `record start <file.rec[.gz]>` | `OK record started <file>` |
| `record stop` | `OK record stopped` |
| `quit` | `OK quit`, then the runtime ends the session as `--stop-after` does (recordings closed) |
| `calibrate` (`calibrate start`) | `OK calibrate started on <recording> so far`: a calibration of the session so far, in the background (below) |
| `calibrate status` | `OK calibrate idle`, or its parts joined by ` \| `: `running <session\|so-far\|new-room> <recording> <replaying\|solving> <share read> <still moments>`, `newroom <seconds>` (stations the room's calibration does not place, and the seconds until the session is calibrated for them: 0.16), `ready <recording> <why>`, `pending <recording>:<wins> ...`, `last <time> <step> <recording> <text>` (calibration\log.txt's last line), `lh1` (LH1 base stations were seen this run, which sally does not track with: 0.16) |
| `calibrate apply` | `OK calibrate apply: ...`, then the runtime ends as for `quit` (the SteamVR driver starts it again, and the start takes the ready calibration up); `ERR` when none is ready |
| `reload` | `ERR ... not implemented yet` |

`calibrate` (9 Nov) answers only in the live runtime (the dongles, a config file and `--hidtap-record`): elsewhere
`ERR calibrate needs the live runtime (...)`. The live runtime keeps the room's calibration up by itself (design 4.2,
`crates/sally/src/autocal.rs`): every session calibrated at its end and judged by the sessions after it, a new room's
stations calibrated during its first session. `calibrate` is the one asked for: the session so far, judged on the last
quarter of its own still moments against the room's calibration, ready (for the next start, or `calibrate apply`)
when it predicts them better, else nothing changes. It is not a protocol version of its own: an older runtime answers
`ERR calibrate not implemented yet`, which a client takes the same way.

`charge` is the battery percentage the device last reported, -1 when none has arrived (recordings carry none); `charging` is 0/1.

`pair` is how the SteamVR driver tells the runtime which device is on which dongle (Valve's driver knows:
`Prop_ConnectedWirelessDongle_String`, docs/sally_driver.md "Which device is which"). The live tap applies it at the
next poll: the dongle's object gets that device's own config, from `<device>_config.json` in the runtime's folder or
else SteamVR's copy (`<Steam config>\lighthouse\<serial, lower case>\config.json`, found through
`%LOCALAPPDATA%\openvr\openvrpaths.vrpath`); an object that had another device's config starts over first (a new
tracker, and the map solve forgets what it measured on it), and any other dongle whose object had that device gives it
up. The dongle's cache (`<dongle>_config.json`) is rewritten, so the next start begins right, and the tap recording
gets an `A` line, so a replay switches where the live run did (`crates/sally-hid/src/line.rs`). `-` takes the
dongle's device away. A dongle not open yet gets its device when it opens; a device with no config anywhere leaves its
dongle with none (logged once, looked for again every 5 s). A playback ignores `pair`: its recording has the `A`
lines. Asking again is harmless (the driver does every second): a dongle that has the device already is left as it is.

`subtype` is one of `generic`, `vive_hmd`, `index_hmd`, `wand`, `knuckles_r`, `knuckles_l`, `tracker`,
`tracker_gen2`; `generic` (and a `-` serial) means the device's config has not been read yet.

Errors are `ERR <command> <message>`. Types are `float`, `int`, `bool` (`0/1`, `on/off`, `true/false` accepted on set)
and `string`. String values with spaces come back with `_` in place of the spaces.

### What a `set` reaches

Every `--name value` option can be set. `bound` says how many live variables the set wrote into immediately. A
tracker option's name from before 11 Nov 2026 (`core2-gate`) is taken for the option (`track-gate`) by `get`, `set`
and `describe` as on the command line, and the reply names it as it is now:

- `bound` >= 1: struct-bound parameters, which is almost everything sally tunes (`track-*`, `map-solve-*`, `light-*`):
  every device's tracker or the solver picks the value up on its next update. (The `kalman-*` / `mpfit-*` families went
  with the old tracker on 4 Nov: a set of one is stored and reaches nothing.)
- `bound` = 0: the value is stored, and takes effect only where the code reads it again. Some do on every use
  (`lock-lighthouses`, read on every OOTX packet), others only at startup (`use-imu`, `required-meas`, `time-window`
  and other `survive_configi` reads in init code). Check the code before relying on one of these.
- Strings never reach a bound variable live.

Values set over the wire are session-only. The config file is never written by `set`; put the value in
`survive_args.txt` to keep it. A set of a `map-solve-*` value can land while a solve is running on another thread; that
solve sees either the old or the new value.

`map unlock` re-enables config file writes (`lock-lighthouses` implies `config-readonly`), so an unlocked map that
gets re-solved is saved.

### Recording

`record start` opens a libsurvive text recording and writes the header a startup recording would have: every
`OPTION`, each connected device's `CONFIG` and the known station poses, then everything from that point on, so the file
replays like one started with `--record`. Only one recording at a time (a startup `--record` counts). USB packet
captures (`.pcap`) cannot be started here (the C runtime's `driver_usbmon` set those up at startup; sally has none).

## Example session

Captured on 30 Oct from the C runtime's simulator (`--simulator --runtime-port 9100 --kalman-coast-after 0.5`: one
simulated 20-sensor device, two stations), with sallyctl.py, which `sally ctl` replaced word for word. With real
devices `bound` is the number of connected devices that carry the parameter. `kalman-coast-after` was an old-tracker
option; a `track-*` one shows the same exchange now.

```
$ sally ctl hello
OK hello sally-runtime 2 5.028927 1 2
$ sally ctl get kalman-coast-after
OK get kalman-coast-after float 0.5 1
$ sally ctl set kalman-coast-after 0.4
OK set kalman-coast-after float 0.5 0.4 1
$ sally ctl devices
DEV SM0 - 20 tracking 0.0000 wand 48000000 0.000000 0.000000 0.000000 1.0000000 0.0000000 0.0000000 0.0000000
OK devices 1
$ sally ctl watch all --only S,R,H,T
R SM0 12.104913 11.360000 ok 32 0.000235 -1 0.000000 -0.76481 -0.13155 1.22266 2 0 4 0.000314 1 28 0.000221
S SM0 - 545760000 12.115170 tracking 0.0100 0.002244 0.012053 3
H SM0 0 1.009 20 0 17 28 0 3 0 0 0 0 0 18 0 33 0 20 0 0 0 36 4
T SM0 7.026529 tracking 0.0000 610 16 625 0 0 0 0 93 0 0 0 0 0 0 3.45356e-05
```

## Tests

- The protocol's lines and replies were held to the C runtime's when it was ported (docs/checking.md): with a
  subscriber pulling every stream the recording is unchanged (the runtime only observes), and every stream line and
  every command's reply the same as the C's but the wall clock. The C runtime's simulator test (test_runtime_sim.py)
  and compare_replays.py left with the C (`git show 92d8db1:unported/tools/sally_runtime/`).
- `sally-panel`'s tests hold its client to the protocol line by line and over a socket, against a stand-in runtime
  (`crates/sally-panel/src/fake_runtime.rs`); `sally ctl`'s parser has its own tests.

## Not in protocol 1

- `reload` (re-read the config file). (`calibrate` came on 9 Nov, above.)
- Starting/stopping a USB capture live.
- `reacquiring` tracker state, per-station map uncertainty and moved-station flags (design 4.2, 4.3).
- Binary encoding. Text until measured to matter (design 4.4).
