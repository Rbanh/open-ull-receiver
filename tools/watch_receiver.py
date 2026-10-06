#!/usr/bin/env python3
"""Continuously record payload-free receiver counters through Windows inbox HID.

Requires the diagnostic HID collection (USB revision 0104). No third-party
Python packages, audio recording, USB writes, driver replacement, or RF changes.
"""
import argparse
import ctypes as C
from ctypes import wintypes as W
from datetime import datetime, timezone, timedelta
import hashlib
import json
import os
from pathlib import Path
import struct
import time
import uuid

from read_usb_retry_stats import PAGE_NAMES, COUNTERS, VID, PID

USB_FIELDS = ('usb_mounted', 'usb_playback_active', 'usb_capture_active',
              'usb_playback_packets', 'usb_playback_frames', 'usb_buffered_frames',
              'usb_consumed_frames', 'usb_dropped_frames', 'usb_malformed_packets',
              'usb_feedback_q16', 'usb_feedback_active', 'usb_capture_underflows',
              'usb_capture_dropped_frames', 'uptime_ms', 'usb_schema')
EXTRA_COUNTERS = ('usb_playback_packets', 'usb_playback_frames', 'usb_consumed_frames',
                  'usb_dropped_frames', 'usb_malformed_packets',
                  'usb_capture_underflows', 'usb_capture_dropped_frames',
                  'parent_accepted', 'parent_duplicates', 'parent_empty',
                  'parent_acked', 'parent_rejected', 'parent_stale',
                  'control_repeat_allowed', 'control_repeat_held', 'control_data_rx')
CORE_PAGES = (0, 1, 2, 13, 15)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def decode_page(page, raw):
    if len(raw) != 64 or raw[:4] != f'RF{page:02d}'.encode():
        raise ValueError(f'Invalid diagnostic report for page {page}')
    names = USB_FIELDS if page == 15 else PAGE_NAMES[page]
    values = dict(zip(names, struct.unpack_from('<15I', raw, 4)))
    if page == 15 and values['usb_schema'] != 1:
        raise ValueError('Unsupported USB diagnostic schema')
    return values


def compare(before, after, seconds):
    keys = set(COUNTERS + EXTRA_COUNTERS)
    keys.update(k for k in after if k.startswith(('ch_', 'retry_failure_', 'retry_cache_')))
    common = keys & before.keys() & after.keys()
    # Headset reconnects reset radio session counters without rebooting USB.
    # Keep lifetime-counter changes when uptime proves the Supermini continued.
    # Otherwise start a fresh baseline; legitimate uint32 wraps survive.
    reset_keys = {k for k in common if after[k] < before[k] and before[k] < 0xf0000000}
    continuous_uptime = ('uptime_ms' in before and 'uptime_ms' in after
                         and after['uptime_ms'] > before['uptime_ms'])
    if 'uptime_ms' in before and after['uptime_ms'] < before['uptime_ms'] < 0xf0000000:
        return {}, {}, ['receiver_counter_reset']
    if reset_keys and not continuous_uptime:
        return {}, {}, ['receiver_counter_reset']
    delta = {k: (after[k] - before[k]) & 0xffffffff for k in sorted(common - reset_keys)}
    metrics = {}
    anomalies = ['partial_counter_reset'] if reset_keys else []
    active = bool(after.get('usb_mounted') and after.get('usb_playback_active'))
    if seconds > 0:
        metrics['usb_frames_per_second'] = round(delta.get('usb_playback_frames', 0) / seconds, 1)
        metrics['consumed_frames_per_second'] = round(delta.get('usb_consumed_frames', 0) / seconds, 1)
    repeated = delta.get('repeated_audio_frames', 0)
    if repeated:
        metrics['unconfirmed_stereo_fraction'] = round(delta.get('no_stereo_ack', 0) / repeated, 4)
    if active:
        for k in ('spi_timeouts', 'spi_crc_errors', 'spi_sequence_errors', 'codec_errors',
                  'playback_queue_drops', 'codec_resets', 'usb_dropped_frames',
                  'usb_malformed_packets', 'source_underflows', 'skipped',
                  'retry_deadline_expired', 'retry_schedule_conflicts'):
            if delta.get(k, 0):
                anomalies.append(k)
        if repeated >= 50 and metrics.get('unconfirmed_stereo_fraction', 0) >= 0.05:
            anomalies.append('radio_stereo_reply_unconfirmed')
        if after.get('usb_buffered_frames', 960) < 240:
            anomalies.append('low_usb_playback_queue')
        if seconds < 5 and delta.get('usb_playback_packets', 0) < seconds * 800:
            anomalies.append('usb_packet_rate_low')
    return delta, metrics, anomalies


class GUID(C.Structure):
    _fields_ = [('a', W.DWORD), ('b', W.WORD), ('c', W.WORD), ('d', C.c_ubyte * 8)]


class Interface(C.Structure):
    _fields_ = [('size', W.DWORD), ('guid', GUID), ('flags', W.DWORD), ('reserved', C.c_size_t)]


class Attributes(C.Structure):
    _fields_ = [('size', W.ULONG), ('vid', W.USHORT), ('pid', W.USHORT), ('version', W.USHORT)]


class Caps(C.Structure):
    _fields_ = [(n, W.USHORT) for n in ('usage', 'usage_page', 'input_len', 'output_len', 'feature_len')]
    _fields_ += [('reserved', W.USHORT * 17)]
    _fields_ += [(n, W.USHORT) for n in ('link_count', 'input_buttons', 'input_values',
                   'input_indices', 'output_buttons', 'output_values', 'output_indices',
                   'feature_buttons', 'feature_values', 'feature_indices')]


class WindowsHID:
    def __init__(self):
        if os.name != 'nt':
            raise RuntimeError('This reader requires Windows')
        self.hid = C.WinDLL('hid', use_last_error=True)
        self.setup = C.WinDLL('setupapi', use_last_error=True)
        self.kernel = C.WinDLL('kernel32', use_last_error=True)
        self.handle = None
        self.hid.HidD_GetHidGuid.argtypes = [C.POINTER(GUID)]
        self.hid.HidD_GetAttributes.argtypes = [W.HANDLE, C.POINTER(Attributes)]
        self.hid.HidD_GetAttributes.restype = C.c_ubyte
        self.hid.HidD_GetPreparsedData.argtypes = [W.HANDLE, C.POINTER(C.c_void_p)]
        self.hid.HidD_GetPreparsedData.restype = C.c_ubyte
        self.hid.HidD_FreePreparsedData.argtypes = [C.c_void_p]
        self.hid.HidP_GetCaps.argtypes = [C.c_void_p, C.POINTER(Caps)]
        self.hid.HidP_GetCaps.restype = W.LONG
        self.hid.HidD_GetFeature.argtypes = [W.HANDLE, C.c_void_p, W.ULONG]
        self.hid.HidD_GetFeature.restype = C.c_ubyte
        self.setup.SetupDiGetClassDevsW.argtypes = [C.POINTER(GUID), W.LPCWSTR, W.HWND, W.DWORD]
        self.setup.SetupDiGetClassDevsW.restype = W.HANDLE
        self.setup.SetupDiEnumDeviceInterfaces.argtypes = [W.HANDLE, C.c_void_p, C.POINTER(GUID), W.DWORD, C.POINTER(Interface)]
        self.setup.SetupDiGetDeviceInterfaceDetailW.argtypes = [W.HANDLE, C.POINTER(Interface), C.c_void_p, W.DWORD, C.POINTER(W.DWORD), C.c_void_p]
        self.setup.SetupDiDestroyDeviceInfoList.argtypes = [W.HANDLE]
        self.kernel.CreateFileW.argtypes = [W.LPCWSTR, W.DWORD, W.DWORD, C.c_void_p, W.DWORD, W.DWORD, W.HANDLE]
        self.kernel.CreateFileW.restype = W.HANDLE
        self.kernel.CloseHandle.argtypes = [W.HANDLE]
        guid = GUID()
        self.hid.HidD_GetHidGuid(C.byref(guid))
        devices = self.setup.SetupDiGetClassDevsW(C.byref(guid), None, None, 0x12)
        invalid = C.c_void_p(-1).value
        if devices == invalid:
            raise C.WinError(C.get_last_error())
        try:
            index = 0
            while True:
                interface = Interface()
                interface.size = C.sizeof(interface)
                if not self.setup.SetupDiEnumDeviceInterfaces(devices, None, C.byref(guid), index, C.byref(interface)):
                    if C.get_last_error() != 259:
                        raise C.WinError(C.get_last_error())
                    break
                index += 1
                required = W.DWORD()
                self.setup.SetupDiGetDeviceInterfaceDetailW(devices, C.byref(interface), None, 0, C.byref(required), None)
                detail = C.create_string_buffer(required.value)
                C.cast(detail, C.POINTER(W.DWORD))[0] = 8 if C.sizeof(C.c_void_p) == 8 else 6
                if not self.setup.SetupDiGetDeviceInterfaceDetailW(devices, C.byref(interface), detail, required, None, None):
                    continue
                path = C.wstring_at(C.addressof(detail) + 4)
                if f'vid_{VID:04x}&pid_{PID:04x}' not in path.lower():
                    continue
                handle = self.kernel.CreateFileW(path, 0, 3, None, 3, 0, None)
                if handle == invalid:
                    continue
                accepted = False
                try:
                    attributes = Attributes()
                    attributes.size = C.sizeof(attributes)
                    if not self.hid.HidD_GetAttributes(handle, C.byref(attributes)) or (attributes.vid, attributes.pid) != (VID, PID):
                        continue
                    data = C.c_void_p()
                    if not self.hid.HidD_GetPreparsedData(handle, C.byref(data)):
                        continue
                    try:
                        caps = Caps()
                        result = self.hid.HidP_GetCaps(data, C.byref(caps))
                    finally:
                        self.hid.HidD_FreePreparsedData(data)
                    if result != 0x110000 or caps.usage_page != 0xff00 or caps.usage != 1 or caps.feature_len != 65:
                        continue
                    accepted = True
                    self.handle = handle
                    self.version = attributes.version
                    break
                finally:
                    if not accepted:
                        self.kernel.CloseHandle(handle)
        finally:
            self.setup.SetupDiDestroyDeviceInfoList(devices)
        if self.handle is None:
            raise RuntimeError('Receiver diagnostic HID collection unavailable; connect firmware USB revision 0104 or later')

    def read(self, page):
        report = (C.c_ubyte * 65)()
        report[0] = 0x10 + page
        if not self.hid.HidD_GetFeature(self.handle, report, 65):
            raise C.WinError(C.get_last_error())
        if report[0] != 0x10 + page:
            raise ValueError('Diagnostic report ID mismatch')
        return decode_page(page, bytes(report)[1:])

    def close(self):
        if self.handle is not None:
            self.kernel.CloseHandle(self.handle)
            self.handle = None


class Log:
    def __init__(self, root, days=7, max_mb=512):
        self.root = root
        self.days = days
        self.max_bytes = max_mb * 1024 * 1024
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = None
        self.last_cleanup = 0
        self.minute = None

    def aggregate(self, record):
        if record.get('type') != 'sample':
            return
        key = record['time'][:16]
        if self.minute is None or self.minute['time'][:16] != key:
            self.flush_minute()
            self.minute = dict(type='minute', time=key + ':00.000+00:00',
                               samples=0, active_samples=0,
                               active_counter_deltas={}, anomaly_sample_counts={})
        bucket = self.minute
        bucket['samples'] += 1
        values = record['counters']
        if not (values.get('usb_mounted') and values.get('usb_playback_active')):
            return
        bucket['active_samples'] += 1
        for name, count in record.get('delta', {}).items():
            totals = bucket['active_counter_deltas']
            totals[name] = totals.get(name, 0) + count
        for name in record.get('anomalies', []):
            counts = bucket['anomaly_sample_counts']
            counts[name] = counts.get(name, 0) + 1
        buffered = values['usb_buffered_frames']
        limits = bucket.setdefault('usb_buffered_min_max', [buffered, buffered])
        limits[0], limits[1] = min(limits[0], buffered), max(limits[1], buffered)

    def flush_minute(self):
        if self.minute is not None:
            path = self.root / ('minute-' + self.minute['time'][:10].replace('-', '') + '.jsonl')
            with path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(self.minute, separators=(',', ':')) + '\n')
            self.minute = None

    def write(self, record):
        self.aggregate(record)
        stamp = datetime.now(timezone.utc)
        if self.path is None or self.path.stat().st_size >= 16 * 1024 * 1024 or not self.path.name.startswith('capture-' + stamp.strftime('%Y%m%d')):
            self.path = self.root / ('capture-' + stamp.strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:6] + '.jsonl')
            self.path.touch()
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, separators=(',', ':')) + '\n')
        if time.monotonic() - self.last_cleanup > 300:
            files = sorted(self.root.glob('capture-*.jsonl'), key=lambda p: p.stat().st_mtime, reverse=True)
            total = 0
            cutoff = (stamp - timedelta(days=self.days)).timestamp()
            for file in files:
                total += file.stat().st_size
                if file != self.path and (file.stat().st_mtime < cutoff or total > self.max_bytes):
                    file.unlink()
            for file in self.root.glob('minute-*.jsonl'):
                if file.stat().st_mtime < (stamp - timedelta(days=30)).timestamp():
                    file.unlink()
            self.last_cleanup = time.monotonic()


def write_status(root, value):
    temp = root / 'status.tmp'
    temp.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
    temp.replace(root / 'status.json')


def summarize(root, hours=1):
    cutoff = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat()
    totals = {}
    anomalies = {}
    result = {'hours': hours, 'samples': 0, 'active_samples': 0, 'events': [], 'markers': []}
    buffered = []
    for file in sorted(root.glob('capture-*.jsonl')):
        if file.stat().st_mtime < (datetime.now(timezone.utc) - timedelta(hours=hours + 1)).timestamp():
            continue
        for row in read_records(file):
            if row.get('time', '') < cutoff:
                continue
            if row.get('type') != 'sample':
                if len(result['events']) < 30:
                    result['events'].append(row)
                continue
            result['samples'] += 1
            active = row['counters'].get('usb_playback_active') and row['counters'].get('usb_mounted')
            if active:
                result['active_samples'] += 1
                buffered.append(row['counters']['usb_buffered_frames'])
                for key, count in row.get('delta', {}).items():
                    totals[key] = totals.get(key, 0) + count
                for key in row.get('anomalies', []):
                    anomalies[key] = anomalies.get(key, 0) + 1
    marker_path = root / 'markers.jsonl'
    if marker_path.exists():
        for marker in read_records(marker_path):
            if marker.get('time', '') >= cutoff:
                result['markers'].append(marker)
    result.update(active_counter_deltas=totals, anomaly_sample_counts=anomalies)
    if buffered:
        result['usb_buffered_min_max'] = [min(buffered), max(buffered)]
    result['interpretation'] = 'Missing stereo replies indicate unconfirmed delivery, not proven audible packet loss. Compare local pipeline faults, queue/feedback trends, and user markers.'
    return result


def read_records(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue  # A live writer may not have finished its final line.


def run(args):
    kernel = C.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.argtypes = [C.c_void_p, W.BOOL, W.LPCWSTR]
    kernel.CreateMutexW.restype = W.HANDLE
    kernel.CloseHandle.argtypes = [W.HANDLE]
    name = 'Local\\OpenULLWatcher-' + hashlib.sha256(str(args.output.resolve()).lower().encode()).hexdigest()[:24]
    mutex = kernel.CreateMutexW(None, True, name)
    if not mutex:
        raise C.WinError(C.get_last_error())
    if C.get_last_error() == 183:
        kernel.CloseHandle(mutex)
        raise RuntimeError('Watcher already running for this output directory')
    log = Log(args.output, args.days, args.max_mb)
    stop = args.output / 'stop.request'
    stop.unlink(missing_ok=True)
    device = None
    previous = None
    previous_at = None
    last_detail = 0
    started = time.monotonic()
    last_error = None
    last_error_at = 0
    session = uuid.uuid4().hex
    log.write({'type': 'watcher_started', 'time': utc_now(), 'pid': os.getpid(), 'interval': args.interval})
    try:
        while not stop.exists() and (not args.seconds or time.monotonic() - started < args.seconds):
            tick = time.monotonic()
            try:
                if device is None:
                    device = WindowsHID()
                    previous = None
                    session = uuid.uuid4().hex
                    log.write({'type': 'receiver_connected', 'time': utc_now(), 'usb_revision': f'{device.version:04x}', 'session': session})
                counters = {}
                for page in CORE_PAGES:
                    counters.update(device.read(page))
                read_span = time.monotonic() - tick
                at = time.monotonic()
                delta, metrics, anomalies = compare(previous, counters, at - previous_at) if previous is not None else ({}, {}, [])
                if previous_at is not None and at - previous_at > args.interval * 1.5:
                    anomalies.append('host_sampling_gap')
                if read_span > 0.1:
                    anomalies.append('diagnostic_read_slow')
                if 'receiver_counter_reset' in anomalies or 'partial_counter_reset' in anomalies:
                    session = uuid.uuid4().hex
                row = {'type': 'sample', 'time': utc_now(), 'session': session,
                       'read_ms': round(read_span * 1000, 2), 'counters': counters,
                       'delta': delta, 'metrics': metrics, 'anomalies': anomalies}
                if at - last_detail >= 30:
                    detail = {}
                    for page in range(3, 13):
                        detail.update(device.read(page))
                    row['detail'] = detail
                    last_detail = at
                log.write(row)
                write_status(args.output, dict(row, state='collecting', pid=os.getpid(), interval=args.interval))
                previous, previous_at = counters, at
                last_error = None
            except (OSError, RuntimeError, ValueError) as exc:
                if device:
                    device.close()
                device = None
                previous = previous_at = None
                error = str(exc)
                if error != last_error or tick - last_error_at > 300:
                    log.write({'type': 'receiver_unavailable', 'time': utc_now(), 'error': error})
                    last_error_at = tick
                last_error = error
                write_status(args.output, {'state': 'waiting_for_receiver', 'time': utc_now(), 'pid': os.getpid(), 'error': error})
            deadline = tick + (args.interval if device else 5)
            while time.monotonic() < deadline and not stop.exists():
                time.sleep(min(0.2, max(0, deadline - time.monotonic())))
    finally:
        if device:
            device.close()
        log.write({'type': 'watcher_stopped', 'time': utc_now(), 'pid': os.getpid()})
        log.flush_minute()
        write_status(args.output, {'state': 'stopped', 'time': utc_now(), 'pid': os.getpid()})
        kernel.CloseHandle(mutex)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('receiver-watch'))
    parser.add_argument('--interval', type=float, default=1)
    parser.add_argument('--seconds', type=float, default=0, help='0 means run until stopped')
    parser.add_argument('--days', type=int, default=7)
    parser.add_argument('--max-mb', type=int, default=512)
    parser.add_argument('--probe', action='store_true')
    parser.add_argument('--summarize', action='store_true')
    parser.add_argument('--hours', type=float, default=1)
    parser.add_argument('--mark', metavar='DESCRIPTION')
    parser.add_argument('--stop', action='store_true')
    args = parser.parse_args()
    if args.interval < 1 or args.days < 1 or args.max_mb < 16 or args.hours <= 0 or args.seconds < 0:
        parser.error('interval >= 1, days >= 1, max-mb >= 16, hours > 0, seconds >= 0 required')
    if args.mark is not None:
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output / 'markers.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'type': 'user_marker', 'time': utc_now(), 'description': args.mark}) + '\n')
        print('Playback observation timestamp recorded.')
    elif args.stop:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / 'stop.request').touch()
        print('Watcher stop requested.')
    elif args.summarize:
        print(json.dumps(summarize(args.output, args.hours), indent=2))
    elif args.probe:
        device = WindowsHID()
        try:
            print(json.dumps({str(p): device.read(p) for p in CORE_PAGES}, indent=2))
        finally:
            device.close()
    else:
        run(args)


if __name__ == '__main__':
    main()
