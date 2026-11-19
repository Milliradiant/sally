# sally

sally is tracking for SteamVR. If your trackers, controllers or headset are tracked by base stations (VIVE trackers,
Tundra trackers, Index controllers, an Index or Vive headset), sally works out where they are in place of SteamVR's
own tracking, and aims to do it more steadily.

SteamVR's own tracking keeps running underneath. Whenever sally has nothing for a device, SteamVR's tracking is used
for it, and one switch takes you back to SteamVR's tracking for everything.

## What you need

- A Windows PC with SteamVR.
- 2.0 base stations (the ones with a curved front). The older 1.0 base stations are not supported: with them sally
  says so and SteamVR's tracking stays in use.
- Your trackers and controllers set up in SteamVR as usual. sally changes nothing about pairing, buttons, battery
  levels or vibration: all of that stays SteamVR's.

A Quest or another headset with its own tracking works too, with the trackers on sally.

## Installing

1. Download `sally-setup.exe` from the [releases page](../../releases): the newest release is at the top.
2. Run it. Windows may say it protected your PC, because the file is new and not signed: choose **More info**, then
   **Run anyway**.
3. Press OK.

That is all. It does not matter whether SteamVR is running or your trackers are on. If SteamVR was running, restart
it once.

To update, download the newer `sally-setup.exe` and run it the same way. Your settings stay.

## Using it

Start SteamVR the way you always do. sally starts with it.

sally has a panel that says what it is doing. It sits on the taskbar while SteamVR runs, it is the "sally" tab in
SteamVR's dashboard, and you can open it from the Start menu. While sally is not tracking yet, a line at the top of the
panel says why and what to do, if there is anything to do. When the line is gone, sally is tracking.

- **The first time in a room** sally needs a few minutes to learn where your base stations are. Keep your trackers on
  and move them around the room. SteamVR's tracking is used until sally is ready, so you can just start playing.
- **Every time after that** sally takes over within about half a minute of moving around.

At the top right of the panel is a switch, **sally | Valve**. Valve is SteamVR's own tracking. Switch whenever you
like, in the middle of a game too: it is the quickest way to see the difference, and to go back if something looks
wrong.

## Settings worth knowing

Everything is in the panel, under **Settings**.

- **Headset tracking**: for an Index or Vive headset, whether the headset itself is tracked by sally or by SteamVR.
  It starts on sally.
- **Sessions kept**: sally records what your trackers send during each session, about 120 MB an hour. It uses the
  recordings to keep its picture of your room right, and a check (below) is made from the one being recorded. Only
  the last few sessions are kept, and you choose how many. The panel shows how much space they take.
- **Pose log**: off unless you switch it on. With it on, a session also keeps what SteamVR's own tracking made of
  your devices, so the **Replay** tab can show the two side by side. It takes about 100 MB an hour with a full set
  of trackers, and it is kept and deleted with the sessions above.
- **Devices**: give your trackers names and say where you wear them.

## If something is wrong

- Look at the line at the top of the panel first.
- Flip the switch to **Valve**. If the problem stays, it is not sally.
- If SteamVR starts without sally (after a crash SteamVR switches add-ons off), switch it on again in SteamVR:
  Settings, Startup / Shutdown, Manage add-ons, sally. Then restart SteamVR.

## Asking for help

Open the panel's **Check** tab while SteamVR runs. Press **Start**, do whatever goes wrong for a minute or two, and
press **Stop**. sally then says for each tracker, controller and headset where its trouble comes from (the light of
the base stations, the radio, or the device itself), and makes one file to send: **Show the file to send** opens its
folder.

To send it, join [sally's Discord server](https://discord.gg/4A836TmKtv), post the file there and say what you saw.

The file does not say whose it is. Every serial number in it is replaced by a stand-in (your trackers', your
dongles', your base stations'), and it holds neither your Windows user name nor your PC's name, so it can be posted
where others can see it. What it does hold is what it takes to look into the problem: what your devices sent during
the check (the light they saw and how they moved), where your base stations stand, your devices' factory
calibration, sally's settings and log, and the lines of SteamVR's log that come from its lighthouse driver and from
sally. **Open the report** shows the same check as a page, with a list of which of your devices each stand-in is.
That page stays on your PC.

## What sally does and does not touch

- It only listens to your trackers, controllers and base stations. It never writes anything to them: no firmware, no
  settings.
- It installs into your own user folder (`%LOCALAPPDATA%\sally`) and adds itself to SteamVR's add-ons. It needs no
  administrator rights.
- It has been used with VIVE Tracker 3.0, Tundra trackers, Index controllers and an Index headset, with up to four
  2.0 base stations, on one PC. Other base-station hardware should work and has not been tried.

## Removing it

Close SteamVR. In Windows open Settings, Apps, Installed apps, find sally and choose Uninstall. Everything sally put
on the PC is removed, including what it learned about your room and its recordings.

## More

sally is built on [libsurvive](https://github.com/cntools/libsurvive). How it works and how to build it:
[docs/developing.md](docs/developing.md). License: MIT, see [LICENSE](LICENSE).
