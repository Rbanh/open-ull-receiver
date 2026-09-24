# Build and flash the two-board prototype

The tested combination is an ESP32-S3 Supermini with 8 MB flash for USB/radio and an ESP32-S3-Zero with PSRAM for the codec. The `platformio.ini` files pin `espressif32@6.12.0` and their flash mode/size. Building or flashing another board variant requires checking its flash and PSRAM characteristics first.

Before building, run the dependency installer and the private provisioner in the root README. Both projects should then build with:

```sh
pio run -d firmware/codec
pio run -d firmware/radio
```

The S3-Zero must have working PSRAM; the codec refuses startup without its decoder arena. The Supermini is the USB audio device. The USB audio interface uses a development VID/PID (`cafe:4011`) and is independent of Razer USB drivers.

The two boards are connected by the five SPI/control signals in [architecture](architecture.md), plus common GND and 5 V. **When their 5 V rails are tied, power only one USB port at a time.** Flash each board with its own USB cable while the other USB port is unplugged. After both are flashed, leave only the Supermini connected to the PC; it powers the Zero through the shared 5 V connection. Verify the exact board and serial port before uploading:

```sh
pio run -d firmware/codec -t upload --upload-port /dev/serial/by-id/YOUR_ZERO
pio run -d firmware/radio -t upload --upload-port /dev/serial/by-id/YOUR_SUPERMINI
```

Some boards need BOOT/RESET to enter the ROM downloader. The release does not contain private prebuilt binaries. If a build changes the controller or code layout, check the runtime and radio behavior before relying on it; the prototype uses version-sensitive ESP32-S3 controller hooks.
