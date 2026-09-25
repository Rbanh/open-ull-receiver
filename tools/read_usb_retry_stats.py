#!/usr/bin/env python3
"""Read payload-free development receiver counters over EP0; never writes USB/NVS.

Run with access to /dev/bus/usb (for example, a temporary udev ACL or sudo).
The receiver must have the RF00/RF01/RF02 diagnostics build installed.
"""
import argparse
import ctypes as C
import json
import struct
import time

VID, PID = 0xCAFE, 0x4011
PAGE_NAMES = (
    ('phase', 'frame', 'submitted', 'skipped', 'valid_rx', 'rejected_rx',
     'audio_completed', 'ack_left', 'ack_right', 'retry_attempted',
     'retry_completed', 'retry_failures', 'retry_skips', 'retry_warmup', 'flags'),
    ('source_underflows', 'source_discarded', 'spi_exchanges', 'spi_timeouts',
     'spi_crc_errors', 'spi_sequence_errors', 'codec_errors',
     'playback_queue_drops', 'codec_resets', 'spi_exchange_us',
     'spi_max_exchange_us', 'mic_received', 'mic_decoded', 'mic_dropped',
     'usb_playback_dropped_frames'),
    ('repeated_audio_frames', 'first_reply_authenticated',
     'second_reply_authenticated', 'first_window_stereo_ack',
     'second_window_new_stereo_ack', 'no_stereo_ack',
     'fallback_attempted', 'fallback_sent', 'fallback_failed',
     'retry_schedule_conflicts', 'retry_deadline_expired'),
    tuple(f'retry_failure_{i}' for i in range(10)),
    tuple(f'retry_cache_{i}' for i in range(15)),
    *(tuple(f'ch_{(page * 15 + i) // 3}_{("sent", "reply", "stereo_ack")[(page * 15 + i) % 3]}'
             if page * 15 + i < 111 else f'unused_{page}_{i}'
             for i in range(15)) for page in range(8)),
    ('parent_active', 'parent_accepted', 'parent_duplicates', 'parent_empty',
     'parent_acked', 'parent_rejected', 'parent_stale', 'parent_header',
     'parent_eligibility', 'control_repeat_allowed', 'control_repeat_held',
     'control_data_rx', 'control_quiet_ms'),
    ('air_controls', 'acl_controls', 'known_controls', 'unknown_controls',
     'setup_hidden', 'unframed_air', 'last_unknown_meta',
     *(f'reserved_{i}' for i in range(7, 14)), 'schema_version'),
)
COUNTERS = (
    'frame', 'submitted', 'skipped', 'valid_rx', 'rejected_rx',
    'audio_completed', 'ack_left', 'ack_right', 'retry_attempted',
    'retry_completed', 'retry_failures', 'retry_skips',
    'source_underflows', 'source_discarded', 'spi_exchanges',
    'spi_timeouts', 'spi_crc_errors', 'spi_sequence_errors',
    'codec_errors', 'playback_queue_drops', 'codec_resets',
    'mic_received', 'mic_decoded', 'mic_dropped',
    'usb_playback_dropped_frames', 'repeated_audio_frames',
    'first_reply_authenticated', 'second_reply_authenticated',
    'first_window_stereo_ack', 'second_window_new_stereo_ack',
    'no_stereo_ack', 'fallback_attempted', 'fallback_sent', 'fallback_failed',
    'retry_schedule_conflicts', 'retry_deadline_expired',
)

class LibUSB:
    def __init__(self):
        self.lib = C.CDLL('libusb-1.0.so.0')
        l = self.lib
        l.libusb_init.argtypes = [C.POINTER(C.c_void_p)]
        l.libusb_init.restype = C.c_int
        l.libusb_open_device_with_vid_pid.argtypes = [C.c_void_p, C.c_uint16, C.c_uint16]
        l.libusb_open_device_with_vid_pid.restype = C.c_void_p
        l.libusb_control_transfer.argtypes = [C.c_void_p, C.c_uint8, C.c_uint8,
            C.c_uint16, C.c_uint16, C.POINTER(C.c_uint8), C.c_uint16, C.c_uint]
        l.libusb_control_transfer.restype = C.c_int
        l.libusb_close.argtypes = [C.c_void_p]
        l.libusb_exit.argtypes = [C.c_void_p]
        self.ctx = C.c_void_p()
        err = l.libusb_init(C.byref(self.ctx))
        if err: raise RuntimeError(f'libusb_init returned {err}')
        self.dev = l.libusb_open_device_with_vid_pid(self.ctx, VID, PID)
        if not self.dev:
            l.libusb_exit(self.ctx)
            raise PermissionError('Cannot open cafe:4011; check it is connected and grant temporary /dev/bus/usb access')

    def read(self, page):
        buf = (C.c_uint8 * 64)()
        ret = self.lib.libusb_control_transfer(self.dev, 0xC0, 0x5C, page, 0, buf, 64, 1000)
        if ret != 64: raise RuntimeError(f'RF{page:02d} control read returned {ret}')
        raw = bytes(buf)
        if raw[:4] not in (f'RF{page:02d}'.encode(), bytes((82, 70, 48, 48 + page)), b'R\x00\x00\x00'):
            raise ValueError(f'Unexpected page {page}: {raw[:4]!r}')
        # retry21 accidentally wrote byte tags through uint32_t indices; payload is intact.
        vals = struct.unpack_from('<15I', raw, 4)
        return dict(zip(PAGE_NAMES[page], vals))

    def close(self):
        self.lib.libusb_close(self.dev)
        self.lib.libusb_exit(self.ctx)

def sample(dev):
    out = {}
    for page in range(3): out.update(dev.read(page))
    for page in (3, 4):
        try: out.update(dev.read(page))
        except RuntimeError: break  # older diagnostics image has only RF00..RF02
    flags = out.pop('flags')
    out['retry_enabled'] = bool(flags & 1)
    out['retry_active'] = bool(flags & 2)
    out['retry_blocked'] = bool(flags & 4)
    out['retry_cooldown'] = bool(flags & 8)
    out['retry_stable_frames'] = (flags >> 8) & 255
    return out

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=30)
    a = p.parse_args()
    if not 1 <= a.seconds <= 120: p.error('--seconds must be 1..120')
    dev = LibUSB()
    try:
        before = sample(dev)
        start = time.monotonic()
        time.sleep(a.seconds)
        after = sample(dev)
    finally: dev.close()
    elapsed = time.monotonic() - start
    delta = {k: (after[k] - before[k]) & 0xffffffff for k in COUNTERS}
    repeated = delta['repeated_audio_frames']
    classified = sum(delta[k] for k in ('first_window_stereo_ack',
        'second_window_new_stereo_ack', 'no_stereo_ack'))
    result = {'seconds': round(elapsed, 3), 'start': before, 'end': after,
              'delta': delta, 'repeated_classification_consistent': classified == repeated}
    print(json.dumps(result, indent=2))

if __name__ == '__main__': main()
