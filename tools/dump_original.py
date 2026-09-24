#!/usr/bin/env python3
"""Read a supported owner's original 1532:0565 receiver to a private flash file.

Known-build diagnostic HID only. The command whitelist has information-query
and flash-page-read; there are no erase, program, reset, or pairing commands.
The output contains vendor firmware and personal keys: keep it private.
"""
import argparse
import ctypes
import os
from pathlib import Path
import struct
import time

BUILD = b"2025/07/24 16:15:00 GMT +08:00"
HID_PREFIX = bytes.fromhex("0613ff0901a101150026ff00850609007508953d9102850709007508953d8102c0")
FLASH_BYTES = 0x400000
PAGE_BYTES = 256


class ReadOnlyReceiver:
    def __init__(self):
        nodes = []
        for node in Path("/sys/class/hidraw").glob("*"):
            try:
                attrs = dict(line.split("=", 1) for line in (node / "device/uevent").read_text().splitlines())
                descriptor = (node / "device/report_descriptor").read_bytes()
                if attrs.get("HID_ID") == "0003:00001532:00000565" and len(descriptor) == 191 and descriptor.startswith(HID_PREFIX):
                    nodes.append(node)
            except (OSError, ValueError):
                continue
        if len(nodes) != 1:
            raise RuntimeError(f"Expected one supported receiver diagnostic interface; found {len(nodes)}")
        self.lib = ctypes.CDLL("libhidapi-hidraw.so.0")
        self.lib.hid_open_path.argtypes = [ctypes.c_char_p]
        self.lib.hid_open_path.restype = ctypes.c_void_p
        self.lib.hid_write.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        self.lib.hid_get_input_report.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        self.lib.hid_close.argtypes = [ctypes.c_void_p]
        self.handle = self.lib.hid_open_path(("/dev/" + nodes[0].name).encode())
        if not self.handle:
            raise RuntimeError("Could not open receiver HID interface")
        self.pending = bytearray()

    def close(self):
        self.lib.hid_close(self.handle)

    def query(self, command, payload=b""):
        if command == 0x1e08:
            if payload:
                raise ValueError("Information query takes no payload")
        elif command == 0x0403:
            if len(payload) != 6 or payload[:2] != b"\x00\x01":
                raise ValueError("Only storage-zero page reads are supported")
            address = int.from_bytes(payload[2:], "little")
            if address % PAGE_BYTES or address + PAGE_BYTES > FLASH_BYTES:
                raise ValueError("Flash address outside supported 4 MiB range")
        else:
            raise ValueError("Unsupported command")
        packet = struct.pack("<BBHH", 5, 0x5a, len(payload) + 2, command) + payload
        report = (b"\x06" + struct.pack("<H", len(packet)) + packet).ljust(62, b"\0")
        if self.lib.hid_write(self.handle, ctypes.create_string_buffer(report, 62), 62) != 62:
            raise RuntimeError("HID read request failed")
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            time.sleep(0.001)
            buf = ctypes.create_string_buffer(b"\x07" + bytes(61), 62)
            received = self.lib.hid_get_input_report(self.handle, buf, 62)
            if received < 3:
                raise RuntimeError("HID response failed")
            raw = buf.raw[:received]
            size = int.from_bytes(raw[1:3], "little")
            if raw[0] != 7 or size > received - 3:
                raise RuntimeError("Invalid HID report framing")
            self.pending.extend(raw[3:3 + size])
            while len(self.pending) >= 6:
                head, kind, size, code = struct.unpack_from("<BBHH", self.pending)
                if head != 5 or kind not in (0x5a, 0x5b, 0x5c, 0x5d) or not 2 <= size <= 4096:
                    raise RuntimeError("Invalid diagnostic response framing")
                if len(self.pending) < size + 4:
                    break
                response = bytes(self.pending[6:size + 4])
                del self.pending[:size + 4]
                if code == command and kind in (0x5b, 0x5d):
                    return response
        raise TimeoutError("Receiver read timed out")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="Private file outside this Git checkout")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    out = args.out.expanduser().resolve()
    if out.is_relative_to(repo):
        parser.error("Dump must be saved outside the public repository")
    if out.exists():
        parser.error("Output already exists; choose a new private filename")
    out.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.umask(0o077)
    receiver = ReadOnlyReceiver()
    try:
        if BUILD not in receiver.query(0x1e08):
            raise RuntimeError("Unsupported original receiver firmware build")
        with out.open("xb") as stream:
            os.chmod(out, 0o600)
            for address in range(0, FLASH_BYTES, PAGE_BYTES):
                result = receiver.query(0x0403, b"\x00\x01" + struct.pack("<I", address))
                if len(result) != 264 or result[:2] != b"\0\0" or int.from_bytes(result[4:8], "little") != address:
                    raise RuntimeError(f"Malformed read at {address:#x}; partial output remains private")
                stream.write(result[8:])
                if (address + PAGE_BYTES) % 0x40000 == 0:
                    stream.flush()
                    print(f"Read {address + PAGE_BYTES:#x}/{FLASH_BYTES:#x} bytes", flush=True)
            stream.flush()
            os.fsync(stream.fileno())
        if BUILD not in receiver.query(0x1e08):
            raise RuntimeError("Receiver stopped responding after the read")
        print(f"Private dump complete: {out}; never upload this file")
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
