#!/usr/bin/env python3
"""Extract per-device receiver credentials from an owner's local flash dump.

This is an offline parser for the NVDM layout observed on one 1532:0565
receiver firmware. It never uploads a dump, prints keys, or changes a device.
Multiple records are normal; the caller must explicitly choose offsets.
"""
import argparse
import os
import re
import tempfile
from pathlib import Path

NVDM = b"AB15\x00"
BOND_NAME = b"1840\x00"
SIRK_NAME = b"1900\x00"
ADDRESS_NAME = re.compile(rb"[Ff]32[Dd]\x00")
MAX_FLASH = 8 * 1024 * 1024


def records(data, name, sizes):
    pattern = re.compile(re.escape(NVDM) + (name if isinstance(name, bytes) else name.pattern))
    for match in pattern.finditer(data):
        if match.start() < 16:
            continue
        header = data[match.start() - 16:match.start()]
        size = int.from_bytes(header[4:6], "little")
        if header[2:4] != b"\x05\x05" or size not in sizes:
            continue
        value = data[match.end():match.end() + size]
        if len(value) != size:
            continue
        yield match.start(), value


def scan(data):
    bonds = {}
    for offset, value in records(data, BOND_NAME, {161}):
        target = value[1:7]
        ltk = value[0x41:0x51]
        if value[0] == 0 and len(target) == 6 and any(target) and len(ltk) == 16 and any(ltk) and ltk != b"\xff" * 16:
            bonds[offset] = value
    sirks = {off: value[:16] for off, value in records(data, SIRK_NAME, {16, 18})
             if any(value[:16]) and value[:16] != b"\xff" * 16}
    addresses = {}
    for offset, value in records(data, ADDRESS_NAME, {12}):
        try:
            raw = bytes.fromhex(value.decode("ascii"))
        except (ValueError, UnicodeDecodeError):
            continue
        if len(raw) == 6 and raw[5] & 0xc0 == 0xc0:
            addresses[offset] = raw
    return bonds, sirks, addresses


def c_array(name, value):
    return f"static const uint8_t {name}[{len(value)}]={{" + ",".join(f"0x{x:02x}" for x in value) + "};\n"


def create_private(path, contents, force):
    if path.exists() and not force:
        raise FileExistsError(f"{path} exists; use --force to replace it")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=".provision-", dir=path.parent)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w") as stream:
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def choose(candidates, offset, kind):
    if offset is None:
        raise ValueError(f"Select --{kind}-offset from inspect output")
    if offset not in candidates:
        raise ValueError(f"No validated {kind} record at {offset:#x}")
    return candidates[offset]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("inspect", "generate"))
    parser.add_argument("--flash", type=Path, required=True, help="Private 4 MiB dump from your own receiver")
    parser.add_argument("--bond-offset", type=lambda s: int(s, 0))
    parser.add_argument("--sirk-offset", type=lambda s: int(s, 0))
    parser.add_argument("--receiver-offset", type=lambda s: int(s, 0))
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "firmware/radio/private")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.flash.stat().st_size != 0x400000 or args.flash.stat().st_size > MAX_FLASH:
        parser.error("Expected an exact 4 MiB receiver flash dump")
    bonds, sirks, addresses = scan(args.flash.read_bytes())
    if args.command == "inspect":
        print("Candidate offsets only; no keys are displayed. Test a candidate if multiple records exist.")
        print("bond:", " ".join(f"{offset:#x}" for offset in bonds) or "none")
        print("sirk:", " ".join(f"{offset:#x}" for offset in sirks) or "none")
        print("receiver:", " ".join(f"{offset:#x}" for offset in addresses) or "none")
        return
    try:
        bond = choose(bonds, args.bond_offset, "bond")
        sirk = choose(sirks, args.sirk_offset, "sirk")
        identity = choose(addresses, args.receiver_offset, "receiver")
    except ValueError as exc:
        parser.error(str(exc))
    target = bond[1:7]
    rand, ediv, ltk = bond[0x53:0x5b], bond[0x51:0x53], bond[0x41:0x51]
    bond_header = "// Generated locally from an owner-supplied receiver dump. Never commit.\n#include <stdint.h>\n"
    for name, value in (("saved_rand", rand), ("saved_ediv", ediv), ("saved_ltk", ltk), ("saved_sirk", sirk)):
        bond_header += c_array(name, value)
    device_header = "// Generated locally from an owner-supplied receiver dump. Never commit.\n#include <stdint.h>\n"
    device_header += c_array("target", target) + c_array("identity", identity)
    create_private(args.output / "bond.h", bond_header, args.force)
    create_private(args.output / "device.h", device_header, args.force)
    print(f"Wrote private headers in {args.output}; credentials withheld from stdout."
          " This does not establish which stored bond is active.")


if __name__ == "__main__":
    main()
