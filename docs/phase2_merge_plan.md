# Bringing `rtk_gnss_phase_2` back into `rtk_gnss`: proposed pull-request breakdown

Draft for discussion. Based on a read of `RI401/rtk_gnss_phase_2` branch `phase-2-dev` as of its last commit (2026-10-11): 80 commits ahead of `rtk_gnss` `main` (`f1a2d00`), 59 files changed, +6,716 / −554 lines. I have not reviewed `wifi_provision.c`, `console.c`, the `gnss_lc29h.c` changes or the Android app line by line, and I have not checked that each commit below applies on its own.

## 1. A mechanical problem to fix first

`rtk_gnss_phase_2` is **not a GitHub fork** of `darylbcarr/rtk_gnss` (GitHub reports no parent). GitHub only offers pull requests between repositories in the same fork network, so a pull request from it into `rtk_gnss` is not possible as things stand. Pick one:

1. **Collaborator access (simplest).** Add the colleague as a collaborator on `darylbcarr/rtk_gnss`. They push topic branches there (`feature/...`, `fix/...`) and open pull requests against `main`.
2. **Re-fork.** Fork `darylbcarr/rtk_gnss` afresh and push the topic branches to that fork. Pull requests then work normally.
3. **Pull locally.** Add their repo as a remote and cherry-pick or merge from it by hand. No pull-request review on GitHub.

The rest of this document assumes option 1 or 2.

## 2. Decisions to make before any merge

These change what the pull requests contain, so settle them first.

| # | Decision | Why it matters |
|---|---|---|
| 1 | **Is removing the ZED-F9P driver intended?** Commit `41c8560` deletes `gnss_ubx.c` and the `GNSS_DRIVER` Kconfig choice, making the project LC29H-only. | `CLAUDE.md` still describes the ZED-F9P as the verified driver. If LC29H-only is the decision, update the docs; if not, restore it. |
| 2 | **Where does pole length live?** The fork added a pole-height setting (`2455c4d`) and removed it (`dc1c213`: "belongs in the phone app, not here"). The IMU branch (`feature/imu-tilt-level`) stores pole length on the device in NVS and uses it on the Level screen. | Tilt-corrected capture needs the length wherever the correction is computed. Either the device keeps it (then add console commands so the phone app can set it) or the phone app owns it. |
| 3 | **Settings-page security.** `wifi_provision` and the console let anyone on the network (or on the serial port) read and change WiFi and NTRIP credentials. | Check whether the page needs a password or only runs on a private network, before the device is used elsewhere. |
| 4 | **`sdkconfig` in git.** The fork edits the tracked `sdkconfig` seven times. | It is a merge-conflict magnet and mixes personal settings into the history. Moving real defaults into `sdkconfig.defaults` and untracking `sdkconfig` avoids this. |
| 5 | **Are the `PROMPT_*.md` files meant to live in the repo?** | They are AI prompts for the Android app and WiFi access work, not project documentation. |

## 3. Proposed pull requests, in merge order

Each should build with `idf.py build`, and its description should say what was tested on hardware and what was not. Commit hashes are from `phase-2-dev`.

| PR | Contents | Key commits | Notes |
|---|---|---|---|
| **1. Crash and stability fixes** | Spinlock crash after NTRIP bootstrap, rover crash on first RTCM byte in LoRa mode, RTCM ring buffer, PAIR007 message-enable ordering, WiFi connect timeout, synthetic `$GST` accuracy fix | `fa157d1`, `689f53a`, `9a6c628`, `6037bd8`, `71a2b77` | Smallest and lowest risk. Merge first. Some of these may depend on code from later commits and need minor rework to stand alone |
| **2. LC29H driver changes** | Reset via PAIR007/PAIR023 instead of PQTMSRR, MSM4 default for the LoRa link, ellipsoidal height and geodetic-to-ECEF support | `fca16d7`, `b41b8ac`, `f8fa226`, `0c02392`, `cff5fb5`, `7c65325`, `c317cba` | Hold the ZED-F9P removal (`41c8560`) as its own decision (see item 1) |
| **3. Runtime configuration** | WiFi/NTRIP/correction source in NVS, serial console, runtime correction-source menu entry, console commands mirroring the menu | `df3a830`, `97c048a`, `9a00583`, `e894bd4` | Touches `config.c`, `config.h`, `ui.c`, `main.cpp`, the same files as the IMU branch. See section 4 |
| **4. Base station fixed-position (NTRIP bootstrap)** | Position averaging, scatter check, two agreeing windows, save to NVS, `base_pos set` | `37be0d9`, `8ed8349`, `083b285`, `c35543c`, `87eb408`, `b6dc627`, `9aa2b35` | The `ISSUES.md` entries describe a confirmed false Fix during bootstrap and the fix. Include hardware results in the description |
| **5. Web setup page and status page** | `wifi_provision`, `device_status`, mDNS, "Clear & re-measure" control | `fc65514`, `3dce043`, `3293241`, `422347a`, `2d1fd9d`, `3856911` | Review authentication here (decision 3). Adds the `mdns` managed component and `dependencies.lock` |
| **6. Phone and laptop tools** | `android/` app and `tools/wifi_config_app` | `eb72332` onward, `41b4b72` | Keep out of the firmware history if it can live in its own folder or repo. The Android app's first commit says it is unverified on hardware; its README records later hardware tests |
| **7. Documentation** | `ISSUES.md`, `HANDOFF.md`, `FIELD_WORKFLOW.md`, session notes | Many | Can merge alongside the PR each entry relates to, or last. `ISSUES.md` records open bugs worth keeping visible |

The IMU work on `feature/imu-tilt-level` would go in after PR 3, so it is rebased once onto the final `config` and `ui` code.

## 4. Where this overlaps with the IMU branch

A trial merge of `phase-2-dev` onto the IMU branch conflicts in three files:

- `components/config/include/config.h` and `components/ui/ui.c`: both sides added entries, so the resolution is to keep both.
- `components/config/config.c`: both sides appended new functions at the end of the file and git interleaved them. This needs a hand merge. Naively keeping both sides does not compile.
- `main/main.cpp` merged automatically.

After the merge the Rover menu has 8 entries, which the scrolling menu handles. The IMU branch does not touch the IMU-free files the fork changed, and the fork does not touch `components/imu`.

## 5. Working conventions

- **One pull request per topic**, small enough to review in one sitting. The large merge above is easier to review as seven pieces than as one.
- **Branch from the latest `main`** and rebase before opening a pull request.
- **Each pull request states what was verified on hardware** and on which board (rover or base).
- **Ownership by component** to limit conflicts. Suggested split: colleague owns `base_station`, `wifi_provision`, `device_status`, `field_console` and the phone tools; you own `imu`, `display`, `ui`, `capture` and the transport layer. Anything touching `config`, `main.cpp` or `sdkconfig` is agreed first.
- **Do not commit personal build artifacts** (`sdkconfig`, `.cache/`, `build/`).
- **Hardware-affecting changes** (pin maps, I2C bus, `sdkconfig` defaults) get an explicit second review, since they can break the other person's board.
- **Update `CLAUDE.md`** when a change alters the architecture or development status, so it stays accurate.
