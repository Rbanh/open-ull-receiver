# Reading the receiver without disturbing it

The Supermini exposes payload-free, read-only development counters over USB EP0. They report frame timing, SPI/USB queue health, retransmission outcomes, and parent-control activity. They do **not** return audio, packets, identity material, or pairing credentials.

On Linux, the running app appears as `cafe:4011`. The scripts use the system `libusb-1.0` library and need permission to open the corresponding `/dev/bus/usb` node. A temporary ACL or a root-run invocation is enough; no persistent udev rule is required just to inspect a session.

```sh
python3 tools/read_usb_retry_stats.py --seconds 30
python3 tools/read_usb_parent_stats.py --seconds 30 > control-timing.jsonl
```

The first command prints counter snapshots and their 30-second deltas. The second prints time-stamped changes as newline-delimited JSON. It is useful while turning the wheel or pressing play/pause: `parent_accepted` tells you when an authenticated control message entered the receiver; `hid_queued` and `hid_sent` show whether Linux HID delivery followed it. If those rise together but the action was late, the delay came before the USB HID queue.

A few counters need careful interpretation:

- `submitted`, `audio_completed`, and `skipped` show whether the local 5 ms radio schedule kept up.
- `source_underflows`, `spi_timeouts`, `spi_crc_errors`, `playback_queue_drops`, `usb_playback_dropped_frames`, and `mic_dropped` indicate local pipeline trouble. Compare their **deltas**, since most are cumulative since boot.
- `retry_attempted` and `retry_completed` count second-transmission reservations. `no_stereo_ack` means neither authenticated reply confirmed stereo in its measured windows; it does **not** prove that the headset missed the audio.
- `control_repeat_allowed` and `control_repeat_held` count control-bearing audio frames admitted to or withheld from retransmission. In quiet playback, one in four 30 ms control polls is intentionally held to single-send, providing a control opportunity every 120 ms. A real control reply starts a five-second period of single-send control polls. Ordinary audio retransmissions continue.
- `parent_duplicates`, `parent_rejected`, and `parent_stale` identify distinct control-link failure modes. Their absolute values matter less than whether they rise during a problem.

For context, the tested two-board stack completed 3,003 of 3,003 audio frames over one quiet 15-second interval with zero local queue, SPI, USB, or mic drops and five frames without stereo ACK. This is a snapshot, not a packet-loss guarantee. If playback sounds bad, compare a healthy and bad interval under the same audio source before changing radio parameters.

The diagnostics are development interfaces, not part of the USB Audio or HID standards. Normal listening and microphone use do not require either script or elevated USB access.
