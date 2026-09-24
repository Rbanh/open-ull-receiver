#!/usr/bin/env python3
"""Observe parent-control counters and HID dispatch without changing the receiver."""
import argparse
import ctypes as C
import json
import struct
import time
from read_usb_retry_stats import LibUSB

FIELDS=('parent_active','parent_accepted','parent_duplicates','parent_empty',
        'parent_acked','parent_rejected','parent_stale','parent_header',
        'parent_eligibility','control_repeat_allowed','control_repeat_held',
        'control_data_rx','control_quiet_ms')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=45)
    parser.add_argument('--interval',type=float,default=0.1)
    args=parser.parse_args()
    if not 1<=args.seconds<=120 or not 0.05<=args.interval<=1:
        parser.error('seconds must be 1..120 and interval 0.05..1')
    dev=LibUSB();start=time.monotonic();last=None
    try:
        while time.monotonic()-start<args.seconds:
            values=dev.read(13)
            buffer=(C.c_uint8*16)()
            n=dev.lib.libusb_control_transfer(dev.dev,0xC0,0x5B,0,0,
                                              buffer,16,1000)
            if n!=16: raise RuntimeError(f'HID health read returned {n}')
            raw=bytes(buffer)
            queued,sent=struct.unpack_from('<II',raw,4)
            state=tuple(values[k] for k in FIELDS)+(queued,sent)
            if state!=last:
                item={'t':round(time.monotonic()-start,3)}
                item.update((k,values[k]) for k in FIELDS)
                item.update(hid_queued=queued,hid_sent=sent,hid_backlog=queued-sent)
                print(json.dumps(item),flush=True)
                last=state
            time.sleep(args.interval)
    finally:
        dev.close()

if __name__=='__main__': main()
