# Owner-side receiver provisioning

The radio firmware does not create a new headset bond. It reuses the identity and encrypted bond from an owner's original BlackShark V2 HyperSpeed receiver. These values are unique to a pairing and must never be copied from another user or committed to Git.

`tools/dump_original.py` supports the observed `1532:0565` diagnostic HID interface and firmware build `2025/07/24 16:15:00 GMT +08:00`. It sends only information and flash-page-read commands. The resulting 4 MiB file contains proprietary firmware and secrets. Store it outside the repository with restricted access; never attach it to an issue.

`tools/provision_from_dump.py inspect` lists candidate NVDM record offsets without displaying keys. Historical records often coexist. The current extractor cannot reliably determine which stored bond is active from flash alone. `generate` therefore requires explicit bond, SIRK, and receiver-address offsets. It writes `bond.h` and `device.h` under the ignored `firmware/radio/private/` directory with mode `0600`. A failed selection can result in a build that searches but never authenticates; select and test against your own hardware.

A fully failed USB receiver without an earlier dump offers no readable bond to this workflow. Developing a fresh pairing flow is future work. Do not attempt to use another person's key or flash image.
