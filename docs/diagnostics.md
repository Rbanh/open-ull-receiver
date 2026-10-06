# Reading the receiver without disturbing it

The Supermini exposes payload-free, read-only development counters over USB EP0. They report frame timing, SPI/USB queue health, retransmission outcomes, and parent-control activity. They do **not** return audio, packets, identity material, or pairing credentials.

## Continuous Windows capture

Firmware USB revision `0104` also exposes these counters through a separate
vendor HID collection. Windows uses its built-in HID and USB audio drivers;
diagnostic reads can run alongside playback without installing libusb or WinUSB.
The media-control collection retains consumer report ID 1.

```powershell
python tools/watch_receiver.py --probe
python tools/watch_receiver.py --output receiver-watch
```

The watcher samples radio, codec, and USB pages once a second, with per-channel
histograms every 30 seconds. Page RF15 adds USB stream state, PCM input/consumption
totals, queue depth, feedback in 16.16 format, malformed-packet counts, device
uptime, and schema version 1. HID feature report ID is `0x10 + page`; each report
is one ID byte plus the existing 64-byte RF page. Feature reports contain no
commands and their SET_REPORT callback has no effect.

Logs reconnect automatically after unplugging. They rotate at 16 MiB and retain
up to seven days or 512 MiB by default. A per-directory mutex prevents duplicate
collectors. Reboots start a fresh baseline. If headset reconnection resets only
some counters while device uptime advances, the watcher retains changes in the
other counters and records `partial_counter_reset`. Uint32 counter wraps do not
become spurious loss spikes. Stream inactivity is excluded from
playback fault alerts. Sample pages are sequential snapshots, so individual
1-second counts can differ slightly at page boundaries.
Compact per-minute totals and queue ranges are also retained for 30 days in
`minute-*.jsonl`, so long-term trends survive detailed-log rotation.

```powershell
python tools/watch_receiver.py --output receiver-watch --mark "Sound degraded"
python tools/watch_receiver.py --output receiver-watch --mark "Sound clear again"
python tools/watch_receiver.py --output receiver-watch --summarize --hours 1
python tools/watch_receiver.py --output receiver-watch --stop
```

Use markers to compare clear and degraded intervals. `no_stereo_ack` means no
authenticated reply confirmed stereo reception in that observation window;
it does not by itself prove audible loss. Queue starvation, SPI errors, skipped
radio slots, feedback drift, and channel reply ratios help narrow the cause.
The watcher does not change radio settings or firmware automatically.

For login autostart, create a shortcut to `pythonw.exe` with the absolute watcher
script path and `--output` directory as arguments in your Windows Startup folder.
The local collector can run independently of Codex; scheduled analysis of local
files requires the computer awake and Codex running.

## Linux EP0 capture

On Linux, the running app appears as `cafe:4011`. The scripts use the system `libusb-1.0` library and need permission to open the corresponding `/dev/bus/usb` node. A temporary ACL or a root-run invocation is enough; no persistent udev rule is required just to inspect a session.

```sh
python3 tools/read_usb_retry_stats.py --seconds 30
python3 tools/read_usb_parent_stats.py --seconds 30 > control-timing.jsonl
python3 tools/read_usb_status_probe.py --seconds 30
```

The first command prints counter snapshots and their 30-second deltas. The second prints time-stamped changes as newline-delimited JSON. It is useful while turning the wheel or pressing play/pause: `parent_accepted` tells you when an authenticated control message entered the receiver; `hid_queued` and `hid_sent` show whether Linux HID delivery followed it. If those rise together but the action was late, the delay came before the USB HID queue.

The status probe reports counts and only the source, length, and command byte of the most recent unrecognized authenticated proprietary control. It also counts authenticated control-only radio frames that the audio receiver still rejects, with only length and command metadata when the inner proprietary envelope is valid. It intentionally omits packet contents and all known setup commands. It can reveal whether the headset sends a status message spontaneously, including during connection setup. An unrecognized command is **not** automatically a battery report; the receiver does not yet expose a battery percentage.

A few counters need careful interpretation:

- `submitted`, `audio_completed`, and `skipped` show whether the local 5 ms radio schedule kept up.
- `source_underflows`, `spi_timeouts`, `spi_crc_errors`, `playback_queue_drops`, `usb_playback_dropped_frames`, and `mic_dropped` indicate local pipeline trouble. Compare their **deltas**, since most are cumulative since boot.
- `retry_attempted` and `retry_completed` count second-transmission reservations. `no_stereo_ack` means neither authenticated reply confirmed stereo in its measured windows; it does **not** prove that the headset missed the audio.
- Reply-window diagnostics use the controller's half-microsecond clock. The first/second boundary is 1,420 microseconds (2,840 clock units). Builds before the October 6, 2026 unit correction misclassified first-window replies as second-window replies; their overall stereo-confirmation totals remain comparable. This measurement correction does not change radio scheduling or playback.
- `control_repeat_allowed` and `control_repeat_held` count control-bearing audio frames admitted to or withheld from retransmission. In quiet playback, one in four 30 ms control polls is intentionally held to single-send, providing a control opportunity every 120 ms. A real control reply starts a five-second period of single-send control polls. Ordinary audio retransmissions continue.
- `parent_duplicates`, `parent_rejected`, and `parent_stale` identify distinct control-link failure modes. Their absolute values matter less than whether they rise during a problem.

For context, the tested two-board stack completed 3,003 of 3,003 audio frames over one quiet 15-second interval with zero local queue, SPI, USB, or mic drops and five frames without stereo ACK. This is a snapshot, not a packet-loss guarantee. If playback sounds bad, compare a healthy and bad interval under the same audio source before changing radio parameters.

The diagnostics are development interfaces, not part of the USB Audio or HID standards. Normal listening and microphone use do not require either script or elevated USB access.
