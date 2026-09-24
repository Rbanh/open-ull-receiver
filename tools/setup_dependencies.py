#!/usr/bin/env python3
"""Install pinned build dependencies locally; third-party source stays untracked."""
import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TINYUSB_TAG = "0.18.0"
TINYUSB_COMMIT = "86ad6e56c1700e85f1c5678607a762cfe3aa2f47"
TINYUSB_URL = "https://github.com/hathach/tinyusb.git"
CODEC_PATCH = ROOT / "patches/lc3plus-v1.7.1.patch"
TINYUSB_DEST = ROOT / "firmware/radio/components/ull_usb/tinyusb-0.18.0"
CODEC_DEST = ROOT / "firmware/codec/components/ull_pcm_codec/reference/floating_point"


def codec_directory(path):
    path = path.expanduser().resolve()
    for candidate in (path, path / "src/floating_point", path / "floating_point"):
        if (candidate / "lc3plus.c").is_file() and (candidate / "license.h").is_file():
            return candidate
    raise ValueError("Expected extracted ETSI TS 103 634 V1.7.1 src/floating_point")


def install_codec(source):
    if CODEC_DEST.exists():
        raise FileExistsError(f"Remove old local codec copy first: {CODEC_DEST}")
    source = codec_directory(source)
    if "ETSI TS 103 634 V1.7.1" not in (source / "license.h").read_text():
        raise ValueError("Only the matching LC3plus V1.7.1 reference source is supported")
    CODEC_DEST.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".lc3plus-", dir=CODEC_DEST.parent) as temp:
        temp = Path(temp)
        shutil.copytree(source, temp / "floating_point")
        subprocess.run(["patch", "--batch", "--forward", "--silent", "-p1", "-i", str(CODEC_PATCH)],
                       cwd=temp, check=True)
        (temp / "floating_point").rename(CODEC_DEST)
    print("Installed local LC3plus reference source plus compatibility patch; excluded from Git")


def install_tinyusb(local):
    if TINYUSB_DEST.exists():
        raise FileExistsError(f"Remove old local TinyUSB copy first: {TINYUSB_DEST}")
    TINYUSB_DEST.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".tinyusb-", dir=TINYUSB_DEST.parent) as temp:
        temp = Path(temp)
        target = temp / "tinyusb"
        if local:
            source = local.expanduser().resolve()
            if not (source / "LICENSE").is_file() or not (source / "src/tusb.c").is_file():
                raise ValueError("Local TinyUSB source is incomplete")
            shutil.copytree(source, target, ignore=shutil.ignore_patterns(".git"))
        else:
            subprocess.run(["git", "clone", "--quiet", "--depth", "1", "--branch", TINYUSB_TAG,
                            TINYUSB_URL, str(target)], check=True)
            commit = subprocess.check_output(["git", "-C", str(target), "rev-parse", "HEAD"], text=True).strip()
            if commit != TINYUSB_COMMIT:
                raise RuntimeError("TinyUSB tag changed; refusing unpinned dependency")
            shutil.rmtree(target / ".git")
        target.rename(TINYUSB_DEST)
    print("Installed local TinyUSB 0.18.0 source; excluded from Git")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lc3plus-dir", type=Path, required=True,
                        help="Official ETSI TS 103 634 V1.7.1 source extracted by the user")
    parser.add_argument("--tinyusb-dir", type=Path,
                        help="Optional local TinyUSB 0.18.0 source; otherwise clone pinned upstream tag")
    args = parser.parse_args()
    install_codec(args.lc3plus_dir)
    install_tinyusb(args.tinyusb_dir)


if __name__ == "__main__":
    main()
