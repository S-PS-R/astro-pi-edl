import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import patch

from edl_flight_computer.shared.downlink import PROTOCOL_VERSION
from edl_flight_computer.shared.ground_station import GroundStation


class EarthDownlinkTests(TestCase):
    def test_thins_saved_samples_and_keeps_distinct_events_and_final_sample(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'ground.jsonl'
            with patch('edl_flight_computer.shared.ground_station.socket.socket'):
                receiver = GroundStation(log_path=path, echo=False)

            def send(seq, time_s, packet_type='telemetry', **fields):
                return receiver.ingest(json.dumps(dict(v=PROTOCOL_VERSION, type=packet_type,
                                                       mission='earth', run='drop', seq=seq,
                                                       time_s=time_s, **fields)))

            reading = dict(phase='landing', accel_x=0.2, ekf_displacement_m=-2.0,
                           ekf_vertical_velocity_m_s=-1.0, imu_fresh=True, imu_status='ok',
                           time_s=0.0, settling=True, faults=['temperature_pressure'])
            send(0, 0.0, data=reading)
            live = send(1, 0.05, data=reading)
            self.assertTrue(live['data']['imu_fresh'])
            send(2, 0.1, data=reading)
            send(3, 0.14, 'event', event='landing', event_id=1,
                 data=dict(flight_time_s=6.2, ekf_accel_bias_m_s2=0.1))
            send(4, 0.14, 'event', event='landing', event_id=1, data={})
            send(5, 0.14, data=dict(reading, phase='landed', ekf_vertical_velocity_m_s=0.0))
            receiver.close()
            saved = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]

        self.assertEqual([row['seq'] for row in saved], [0, 2, 3, 5])
        self.assertNotIn('received_at', saved[0])
        self.assertEqual(saved[0]['data'], dict(phase='landing', accel_x=0.2,
                                               ekf_displacement_m=-2.0, ekf_vertical_velocity_m_s=-1.0,
                                               faults=['temperature_pressure']))
        self.assertEqual(saved[2]['data'], {'flight_time_s': 6.2})
        self.assertEqual(saved[3]['data']['ekf_vertical_velocity_m_s'], 0.0)
