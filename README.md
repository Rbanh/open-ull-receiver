# Open ULL Receiver

Experimental, unofficial replacement receiver source for the **Razer BlackShark V2 HyperSpeed**. The working prototype uses two ESP32-S3 boards: a Supermini handles USB audio, controls, and the radio link; an S3-Zero with PSRAM handles LC3plus playback encoding and microphone decoding over SPI. The PC needs **one USB cable** to the Supermini after both boards are flashed.

On the tested headset, the prototype has provided live stereo playback, mono microphone capture, mic mute, volume wheel input, and play/pause. It is still experimental: radio quality and reconnect behavior can vary. This is the **full application source for the tested two-board design**, not a universal ready-made binary. Each owner must supply their own receiver identity and bond, and obtain the external codec source under its own terms.

## Build and provision

1. Obtain the official [ETSI TS 103 634 V1.7.1 LC3plus attachment](https://www.etsi.org/deliver/etsi_ts/103600_103699/103634/01.07.01_60/) and extract it locally. The reference codec is not in this repository. Install it and pinned TinyUSB with:

   ```sh
   python3 tools/setup_dependencies.py --lc3plus-dir /path/to/extracted/LC3plus
   ```

2. If your **own original receiver** is still recognized as `1532:0565` and has the supported diagnostic firmware build, make a private read-only dump outside this Git checkout:

   ```sh
   python3 tools/dump_original.py --out /private/path/original-flash.bin
   python3 tools/provision_from_dump.py inspect --flash /private/path/original-flash.bin
   ```

   Choose the bond, SIRK, and receiver-address record offsets that belong to the active pairing, then generate local headers. Multiple historical records are common; the parser does **not** determine the active bond automatically.

   ```sh
   python3 tools/provision_from_dump.py generate \
     --flash /private/path/original-flash.bin \
     --bond-offset 0xYOUR_OFFSET \
     --sirk-offset 0xYOUR_OFFSET \
     --receiver-offset 0xYOUR_OFFSET
   ```

3. Build with PlatformIO and the pinned `espressif32@6.12.0` platform:

   ```sh
   pio run -d firmware/codec
   pio run -d firmware/radio
   ```

   See [build and wiring instructions](docs/build.md) before flashing. The exact ROM hooks and timing have only been tested with the listed ESP32-S3 boards and platform version.

If the original receiver cannot enumerate and you have no earlier flash dump, this revision cannot generate a matching bond. **New headset pairing is not implemented.** A normal Bluetooth pairing is not a substitute for the HyperSpeed bond. See [provisioning details](docs/provisioning.md).

## What's here

- `firmware/radio`: USB Audio Class 2, HID controls, BLE control link, and the independent low-latency radio transport.
- `firmware/codec`: SPI-connected stereo encoder and mono decoder application.
- `tools`: owner-run, read-only original-receiver dump; offline private-header generator; local dependency installer; release audit.
- `patches`: local adaptations for the separately obtained ETSI LC3plus reference source.
- `tests`: small host-side protocol checks.

No Razer firmware, decompiled vendor source, original-dongle dump, audio capture, encryption key, or precompiled firmware is published here. The receiver's observed wire formats are documented and implemented for interoperability; no Razer USB VID/PID is used by the prototype. The source we wrote is MIT-licensed. See [license boundaries](docs/publication-scope.md) for the LC3plus patch and external dependencies.

Razer and BlackShark are trademarks of their respective owner. This independent project is not affiliated with or endorsed by Razer.
