# Publication and licensing boundary

Our independently written application, build helpers, tests, and documentation are offered under MIT. The public tree excludes the original Razer/headset firmware, decompiled vendor source, complete flash images, pairing credentials, packet/audio captures, and compiled images that would embed an owner's credentials.

The LC3plus codec is a separate ETSI reference implementation. It is **not** MIT-licensed by this repository, and its source is not included. The adaptation patch in `patches/` contains changes and limited context against ETSI TS 103 634 V1.7.1; treat that patch with the reference implementation's terms, not as a blanket MIT grant. ETSI notes that software copyright permissions and patent rights are distinct. Users obtain the official attachment themselves and are responsible for the applicable terms.

TinyUSB is obtained separately from its upstream MIT-licensed release. ESP-IDF and PlatformIO are external toolchain dependencies with their own licenses. The USB device descriptor uses the development VID/PID `cafe:4011`, not Razer's VID/PID.

`firmware/radio/private/`, the external codec and TinyUSB directories, and all binary build products are Git-ignored. Run `python3 tools/audit_release.py` before every public commit. A clean audit checks tracked paths, file types, and known secret patterns; it is not a legal opinion or a substitute for reviewing each contribution.
