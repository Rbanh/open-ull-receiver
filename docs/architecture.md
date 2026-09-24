# Architecture

The USB-facing Supermini presents stereo 48 kHz/16-bit playback, mono 48 kHz/16-bit capture, and USB HID media controls. It maintains the headset's encrypted control connection and schedules 5 ms wireless audio frames. The S3-Zero takes a 1,280-byte SPI request, converts stereo 48 kHz PCM to 96 kHz LC3plus HR at 190 encoded bytes per frame, and converts 32 kHz mono microphone frames back to 48 kHz PCM. Its decoder workspace uses PSRAM; the Supermini does not require PSRAM.

Only the Supermini is USB-connected in normal operation. The boards share ground and 5 V. The tested SPI/control wiring is:

| Supermini | S3-Zero | Function |
|---|---|---|
| GPIO4 | GPIO10 | SPI clock |
| GPIO5 | GPIO9 | Master out / slave in |
| GPIO6 | GPIO8 | Master in / slave out |
| GPIO7 | GPIO7 | Chip select |
| GPIO2 | GPIO12 | Slave ready |

The headset's wheel and short power-button press arrive over the encrypted control link and are forwarded as USB HID consumer controls. The microphone switch controls capture mute. The headset may still beep at its own internal volume limit even though Linux has a separate volume range.

The software currently relies on a previously established original-receiver bond and the exact ESP32-S3 controller build exposed by PlatformIO `espressif32@6.12.0`. It is a reverse-engineered interoperability prototype, not a general BLE Audio receiver.

## Keeping audio and controls on the same air link

The Supermini schedules one audio event every 5 ms. It repeats eligible audio packets after a 1.42 ms subinterval using the same encrypted payload and follows the headset's channel hop. The second transmission provides another chance for reception, but an absent stereo acknowledgement is not the same thing as proven audible loss.

The headset's wheel, mic switch, and short power press use authenticated parent-control messages. A control-bearing audio frame is offered every 30 ms. Repeating every one of those frames made idle audio cleaner but could leave wheel actions arriving seconds late. The current policy repeats a control frame only when its encrypted packet was safely reused from the cache, reserves every fourth control poll for a single transmission, and keeps control polls single-send for five seconds after real control data arrives. Audio-only frames continue to retry. The [diagnostic counters](diagnostics.md) expose this tradeoff without logging packets or pairing data.
