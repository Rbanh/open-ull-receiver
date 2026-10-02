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

## Windows compatibility and validation

Windows 10 version 1703 and later, including Windows 11, ship Microsoft's [USB Audio 2.0 driver](https://learn.microsoft.com/en-us/windows-hardware/drivers/audio/usb-2-0-audio-drivers). A compatible receiver should load `usbaudio2.sys` automatically. Select **Headphones (HyperSpeed Research S3)** for playback and **Microphone (HyperSpeed Research S3)** for input.

The earlier firmware advertised a three-byte Full-Speed feedback endpoint and sent 10.14 feedback. Windows installed the correct audio driver but failed to start it with **Code 10**, logging `usbaudio2` event 37: it could not find a feedback endpoint for asynchronous playback. The receiver now advertises a four-byte explicit feedback endpoint and sends 16.16 feedback with TinyUSB's format correction disabled. TinyUSB 0.18.0 documents this combination as compatible with Linux and Windows. The USB device revision is `0x0103`.

On October 2, 2026, the updated owner-paired prototype was built with ESP-IDF 5.5.0 and the pinned PlatformIO platform. The compiled feedback descriptor, pairing data, partition table, and selected memory/ROM addresses were checked before an application-only Supermini flash at `0x10000`; the flash hash verified. After a normal reset, Windows reported the audio and HID interfaces as **Started**, exposed headphone and microphone endpoints, and the owner confirmed hearing Windows audio through the headset.

Windows microphone recording and headset control behavior remain unverified. Linux playback, microphone, and controls were tested before this feedback change; the four-byte update has not yet been retested on Linux. An automated Windows audio-stream check did not complete, so the playback result above is based on the owner's listening test.

To enter the Supermini downloader, hold **BOOT**, press and release **RESET**, then release **BOOT**. If it remains in download mode after flashing, press **RESET** without holding **BOOT**, or reconnect its USB cable. Keep the codec board's USB unplugged while the boards share power. Make a private backup of the connected receiver before updating; owner-specific binaries and pairing information should remain outside the public checkout.
