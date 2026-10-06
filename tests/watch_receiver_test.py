"""Exercise report validation and rollover/reboot/idle anomaly distinctions."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from watch_receiver import compare, decode_page, Log, summarize, utc_now


class DiagnosticsTest(unittest.TestCase):
    def state(self, **changes):
        value = dict(usb_mounted=1, usb_playback_active=1, uptime_ms=1000,
                     usb_playback_packets=1000, usb_playback_frames=48000,
                     usb_consumed_frames=48000, usb_buffered_frames=960,
                     source_underflows=0, repeated_audio_frames=200, no_stereo_ack=0)
        value.update(changes)
        return value

    def test_report_schema_and_tag(self):
        raw = b'RF15' + struct.pack('<15I', *range(14), 1)
        self.assertEqual(decode_page(15, raw)['usb_schema'], 1)
        with self.assertRaises(ValueError):
            decode_page(1, raw)
        with self.assertRaises(ValueError):
            decode_page(15, raw[:60])
        with self.assertRaises(ValueError):
            decode_page(15, raw[:-4] + struct.pack('<I', 2))

    def test_reboot_is_not_a_huge_loss_delta(self):
        delta, _, anomalies = compare(self.state(), self.state(uptime_ms=10, usb_playback_packets=10), 1)
        self.assertEqual(delta, {})
        self.assertEqual(anomalies, ['receiver_counter_reset'])

    def test_uint32_rollover(self):
        before = self.state(usb_playback_packets=0xfffffff0)
        after = self.state(usb_playback_packets=0x3d8)
        delta, _, _ = compare(before, after, 1)
        self.assertEqual(delta['usb_playback_packets'], 1000)

    def test_headset_reconnect_retains_usb_losses_without_reboot(self):
        before = self.state(frame=500000, usb_dropped_frames=0)
        after = self.state(frame=0, uptime_ms=2000, usb_dropped_frames=40000,
                           usb_playback_packets=2000, usb_playback_frames=96000)
        delta, metrics, anomalies = compare(before, after, 1)
        self.assertNotIn('frame', delta)
        self.assertEqual(delta['usb_dropped_frames'], 40000)
        self.assertEqual(delta['usb_playback_frames'], 48000)
        self.assertEqual(metrics['usb_frames_per_second'], 48000)
        self.assertIn('partial_counter_reset', anomalies)
        self.assertIn('usb_dropped_frames', anomalies)
        self.assertNotIn('receiver_counter_reset', anomalies)

    def test_reset_without_uptime_evidence_is_conservative(self):
        before = self.state(frame=500000)
        after = self.state(frame=0, usb_dropped_frames=40000)
        before.pop('uptime_ms')
        after.pop('uptime_ms')
        delta, _, anomalies = compare(before, after, 1)
        self.assertEqual(delta, {})
        self.assertEqual(anomalies, ['receiver_counter_reset'])

    def test_idle_does_not_produce_playback_faults(self):
        after = self.state(usb_playback_active=0, usb_buffered_frames=0, source_underflows=100)
        _, _, anomalies = compare(self.state(), after, 1)
        self.assertEqual(anomalies, [])

    def test_usb_and_radio_evidence_remain_distinct(self):
        after = self.state(usb_playback_packets=2000, usb_playback_frames=96000,
                           usb_consumed_frames=96000, uptime_ms=2000,
                           repeated_audio_frames=400, no_stereo_ack=20, source_underflows=1)
        delta, metrics, anomalies = compare(self.state(), after, 1)
        self.assertEqual(delta['source_underflows'], 1)
        self.assertEqual(metrics['usb_frames_per_second'], 48000)
        self.assertEqual(metrics['unconfirmed_stereo_fraction'], 0.1)
        self.assertIn('source_underflows', anomalies)
        self.assertIn('radio_stereo_reply_unconfirmed', anomalies)
        self.assertNotIn('usb_packet_rate_low', anomalies)

    def test_summary_tolerates_partial_live_line(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = Log(root)
            log.write(dict(type='sample', time=utc_now(), counters=self.state(),
                           delta={'no_stereo_ack': 2}, anomalies=['radio_stereo_reply_unconfirmed']))
            with log.path.open('a') as stream:
                stream.write('{"unfinished":')
            result = summarize(root)
            self.assertEqual(result['active_samples'], 1)
            self.assertEqual(result['active_counter_deltas']['no_stereo_ack'], 2)

    def test_long_term_minute_totals_exclude_idle_faults(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = Log(root)
            stamp = utc_now()
            for active, count in ((1, 2), (0, 100)):
                log.write(dict(type='sample', time=stamp,
                               counters=self.state(usb_playback_active=active),
                               delta={'source_underflows': count}, anomalies=['source_underflows']))
            log.flush_minute()
            row = json.loads(next(root.glob('minute-*.jsonl')).read_text())
            self.assertEqual(row['samples'], 2)
            self.assertEqual(row['active_samples'], 1)
            self.assertEqual(row['active_counter_deltas']['source_underflows'], 2)


if __name__ == '__main__':
    unittest.main()
