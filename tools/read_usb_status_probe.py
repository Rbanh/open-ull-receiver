#!/usr/bin/env python3
"""Read authenticated-control metadata from RF14. No packet payloads or writes."""
import argparse
import json
import time
from read_usb_retry_stats import LibUSB

FIELDS = ('air_controls','acl_controls','known_controls','unknown_controls',
          'setup_hidden','unframed_air','control_only_frames',
          'control_only_proprietary','control_only_setup_hidden')

def decode(snapshot):
    meta = snapshot.pop('last_unknown_meta')
    control_only_meta = snapshot.pop('control_only_last_meta')
    snapshot['last_unknown'] = None if not meta else {
        'source': 'air' if meta >> 24 == 1 else 'acl' if meta >> 24 == 2 else 'other',
        'length': (meta >> 8) & 0xffff,
        'opcode': meta & 255,
    }
    snapshot['last_control_only_proprietary'] = None if not control_only_meta else {
        'length': control_only_meta >> 8,
        'opcode': control_only_meta & 255,
    }
    return {k:v for k,v in snapshot.items() if not k.startswith('reserved_')}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=10.0)
    a=p.parse_args()
    if not 0 <= a.seconds <= 120:p.error('--seconds must be 0..120')
    dev=LibUSB()
    try:
        start=dev.read(14)
        time.sleep(a.seconds)
        end=dev.read(14)
    finally:dev.close()
    if start['schema_version']!=2 or end['schema_version']!=2:
        raise ValueError('Unexpected RF14 schema')
    delta={k:(end[k]-start[k])&0xffffffff for k in FIELDS}
    print(json.dumps({'seconds':a.seconds,'delta':delta,'end':decode(end)},indent=2))

if __name__=='__main__':main()
