#!/usr/bin/env python3
"""Audit the exact Git index against a reviewed path manifest before publication."""
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "release-manifest.txt"
FORBIDDEN_PARTS = {"private", "reference", "decompiled", "captures", "flash", ".pio", "build", "__pycache__"}
FORBIDDEN_SUFFIXES = {".bin", ".elf", ".map", ".o", ".a", ".so", ".wav", ".pcap", ".pcapng", ".zip", ".log", ".jsonl", ".pem", ".key"}
PATTERNS = (
    rb"(?i)-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----",
    rb"(?i)gh[pousr]_[a-z0-9]{20,}",
    rb"(?i)AIza[a-z0-9_-]{20,}",
    rb"(?i)/home/[a-z0-9_.-]+/",
    rb"(?i)/mnt/[a-z0-9_.-]+/",
    rb"(?i)static\s+const\s+(?:unsigned\s+char|uint8_t)\s+saved_(?:ltk|sirk|ediv|rand)\s*\[\d+\]\s*=\s*\{",
)


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def main():
    expected = set(MANIFEST.read_text().splitlines())
    listed = git("ls-files", "-z").decode().split("\0")
    tracked = set(filter(None, listed))
    errors = []
    if tracked != expected:
        for name in sorted(tracked - expected):
            errors.append(f"unreviewed tracked path: {name}")
        for name in sorted(expected - tracked):
            errors.append(f"manifest path not staged: {name}")
    for name in sorted(tracked):
        path = Path(name)
        if path.is_absolute() or ".." in path.parts or FORBIDDEN_PARTS & set(path.parts) or path.suffix.lower() in FORBIDDEN_SUFFIXES:
            errors.append(f"forbidden path: {name}")
            continue
        entry = git("ls-files", "--stage", "--", name).decode().split()
        if not entry or entry[0] != "100644" and entry[0] != "100755":
            errors.append(f"non-regular Git file: {name}")
            continue
        data = git("show", f":{name}")
        if len(data) > 512 * 1024 or b"\0" in data:
            errors.append(f"oversize or binary file: {name}")
        for pattern in PATTERNS:
            if re.search(pattern, data):
                errors.append(f"secret/path pattern: {name}")
                break
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Staged release audit passed: {len(tracked)} reviewed text files; no excluded paths or obvious secrets")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
