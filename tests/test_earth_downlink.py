from base64 import urlsafe_b64decode
from email import policy
from email.parser import BytesParser
import json
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from smtplib import SMTPServerDisconnected
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import Mock, patch

from edl_flight_computer.shared.downlink import PROTOCOL_VERSION
from edl_flight_computer.shared.ground_station import GroundStation, main, save_run_report
from edl_flight_computer.shared.landing_email import LandingEmail
from edl_flight_computer.shared.gmail_email import GmailEmail


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

    def test_only_confirmed_landings_mark_runs_for_email(self):
        with patch('edl_flight_computer.shared.ground_station.socket.socket'):
            receiver = GroundStation(echo=False)
        self.addCleanup(receiver.close)

        def send(run, mission='earth', **fields):
            return receiver.ingest(json.dumps(dict(v=PROTOCOL_VERSION, mission=mission, run=run,
                                                   time_s=1.0, **fields)))

        send('stopped', type='event', seq=0, event_id=1, event='stopped')
        send('impact', mission='mars', type='event', seq=0, event_id=1, event='impact')
        send('drop', type='telemetry', seq=0, data={'phase': 'landing'})
        self.assertTrue(receiver.runs['stopped'].finished)
        for run in ('stopped', 'impact', 'drop'):
            self.assertIsNone(receiver.runs[run].landing)

        touchdown = send('mars', mission='mars', type='telemetry', seq=0, data={'phase': 'landed'})
        self.assertIs(receiver.runs['mars'].landing, touchdown)
        mars_event = send('mars', mission='mars', type='event', seq=1, event_id=1, event='landing')
        self.assertIs(receiver.runs['mars'].landing, mars_event)

        final = send('drop', type='telemetry', seq=1, data={'phase': 'landed', 'final': True})
        self.assertIs(receiver.runs['drop'].landing, final)
        event = send('drop', type='event', seq=2, event_id=1, event='landing', data={})
        self.assertIs(receiver.runs['drop'].landing, event)
        self.assertIsNone(send('drop', type='event', seq=3, event_id=1, event='landing', data={}))
        self.assertIs(receiver.runs['drop'].landing, event)


class LandingReportTests(TestCase):
    def test_defaults_to_port_8765_without_email_configuration(self):
        with patch.dict('os.environ', {}, clear=True):
            self.assertIsNone(LandingEmail.from_settings('', []))
            with patch('edl_flight_computer.shared.ground_station.GroundStation') as station:
                station.return_value.poll.side_effect = KeyboardInterrupt
                main(['--no-log', '--no-plot'])
                station.assert_called_once_with(8765, None)

    def test_startup_uses_sender_and_recipients_from_run_settings(self):
        addresses = dict(sender='edl@example.test', recipients=['demo@example.test', 'team@example.test'])
        with (
            patch.dict('os.environ', {'EDL_SMTP_HOST': 'smtp.example.test'}, clear=True),
            patch('edl_flight_computer.run.LANDING_EMAIL', addresses),
            patch('edl_flight_computer.shared.ground_station.GroundStation'),
            patch('edl_flight_computer.shared.ground_station.ProcessPoolExecutor'),
            patch('matplotlib.pyplot.figure'),
            patch('matplotlib.pyplot.show'),
            redirect_stdout(StringIO()) as output,
        ):
            main(['--no-log'])
        self.assertIn('listening on UDP 8765', output.getvalue())
        self.assertIn('Landing email enabled for 2 recipient(s).', output.getvalue())

    def test_email_check_authenticates_without_sending_or_opening_receiver(self):
        addresses = dict(sender='edl@example.test', recipients=['demo@example.test'])
        settings = dict(EDL_SMTP_HOST='smtp.example.test',
                        EDL_SMTP_USER='operator', EDL_SMTP_PASSWORD='test-only')
        with (
            patch.dict('os.environ', settings, clear=True),
            patch('edl_flight_computer.run.LANDING_EMAIL', addresses),
            patch('edl_flight_computer.shared.ground_station.GroundStation') as station,
            patch('edl_flight_computer.shared.landing_email.smtplib.SMTP') as connect,
            redirect_stdout(StringIO()) as output,
        ):
            self.assertEqual(main(['--check-email']), 0)
        smtp = connect.return_value.__enter__.return_value
        self.assertIn('SMTP login succeeded. No email sent.', output.getvalue())
        smtp.login.assert_called_once_with('operator', 'test-only', initial_response_ok=True)
        smtp.send_message.assert_not_called()
        station.assert_not_called()

    def test_gmail_check_signs_in_without_submitting_email(self):
        addresses = dict(sender='edl@example.test', recipients=['demo@example.test'],
                         gmail_client_file='gmail-client.json')
        with (
            patch.dict('os.environ', {}, clear=True),
            patch('edl_flight_computer.run.LANDING_EMAIL', addresses),
            patch.object(GmailEmail, '_access_token', return_value='test-token') as access_token,
            patch('edl_flight_computer.shared.gmail_email.urlopen') as send_request,
            patch('edl_flight_computer.shared.ground_station.GroundStation') as station,
            redirect_stdout(StringIO()) as output,
        ):
            self.assertEqual(main(['--check-email']), 0)
        self.assertIn('Gmail API sign-in succeeded. No email sent.', output.getvalue())
        access_token.assert_called_once_with(interactive=True)
        send_request.assert_not_called()
        station.assert_not_called()

    def test_gmail_submits_the_plot_over_https_with_noninteractive_auth(self):
        email = GmailEmail(Path('gmail-client.json'), 'edl@example.test', ('demo@example.test',))
        landing = dict(type='event', mission='mars', run='sim', time_s=386.3)
        with TemporaryDirectory() as folder:
            plot = Path(folder) / 'landing.png'
            plot.write_bytes(b'PNG payload')
            with (
                patch.object(GmailEmail, '_access_token', return_value='test-token') as access_token,
                patch('edl_flight_computer.shared.gmail_email.urlopen') as send_request,
            ):
                send_request.return_value.__enter__.return_value.read.return_value = b'{"id":"message-123"}'
                self.assertEqual(email.send(landing, plot), {})
        access_token.assert_called_once_with()
        request, = send_request.call_args.args
        self.assertEqual(request.full_url, 'https://gmail.googleapis.com/gmail/v1/users/me/messages/send')
        self.assertEqual(request.get_method(), 'POST')
        self.assertEqual(request.get_header('Authorization'), 'Bearer test-token')
        message = BytesParser(policy=policy.default).parsebytes(urlsafe_b64decode(json.loads(request.data)['raw']))
        self.assertEqual(message['From'], email.sender)
        self.assertEqual(message['To'], 'demo@example.test')
        attachment, = message.iter_attachments()
        self.assertEqual(attachment.get_filename(), 'landing.png')
        self.assertEqual(attachment.get_payload(decode=True), b'PNG payload')

    def test_attaches_saved_plot_and_uses_confirmation_metrics(self):
        landing = dict(type='event', mission='earth', run='drop',
                       data=dict(flight_time_s=6.2, ekf_displacement_m=-2.4))
        settings = dict(EDL_SMTP_HOST='smtp.example.test',
                        EDL_SMTP_USER='operator', EDL_SMTP_PASSWORD='test-only')
        with patch.dict('os.environ', settings, clear=True):
            email = LandingEmail.from_settings('edl@example.test',
                                               ['demo@example.test', 'team@example.test', 'demo@example.test'])
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'landing.png'
            with patch('edl_flight_computer.shared.landing_email.smtplib.SMTP') as connect:
                smtp = connect.return_value.__enter__.return_value

                def check_message(message):
                    self.assertEqual(message['From'], 'edl@example.test')
                    self.assertEqual(message['To'], 'demo@example.test, team@example.test')
                    body = message.get_body().get_content()
                    self.assertIn('6.200 s', body)
                    self.assertIn('-2.400 m', body)
                    attachment, = message.iter_attachments()
                    self.assertEqual(attachment.get_content_type(), 'image/png')
                    self.assertEqual(attachment.get_payload(decode=True), path.read_bytes())
                    self.assertTrue(path.read_bytes().startswith(b'\x89PNG\r\n\x1a\n'))
                    return {}

                smtp.send_message.side_effect = check_message
                save_run_report(path, 'drop', 'earth', ((1.0, {'phase': 'landing'}),), landing, email)
                smtp.starttls.assert_called_once()
                smtp.login.assert_called_once_with('operator', 'test-only', initial_response_ok=True)
                smtp.send_message.assert_called_once()

    def test_mars_report_uses_simulated_touchdown_metrics(self):
        landing = dict(type='event', mission='mars', run='sim', time_s=386.3,
                       data=dict(velocity_m_s=2.5, landing_target_error_m=19.8))
        email = LandingEmail('smtp.example.test', 587, 'edl@example.test', ('demo@example.test',), '', '')
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'landing.png'
            path.write_bytes(b'PNG payload')
            with patch('edl_flight_computer.shared.landing_email.smtplib.SMTP') as connect:
                smtp = connect.return_value.__enter__.return_value
                email.send(landing, path)
                message, = smtp.send_message.call_args.args
        self.assertEqual(message['Subject'], 'Mars landing confirmed')
        body = message.get_body().get_content()
        self.assertIn('T+386.3 s', body)
        self.assertIn('2.50 m/s', body)
        self.assertIn('19.80 m', body)

    def test_reconnects_with_challenge_before_credentials_when_login_disconnects(self):
        landing = dict(type='event', mission='mars', run='sim', time_s=386.3)
        email = LandingEmail('smtp.example.test', 465, 'edl@example.test', ('demo@example.test',),
                             'operator', 'test-only')
        plot = Mock(spec=Path, name='plot')
        plot.name = 'landing.png'
        plot.read_bytes.return_value = b'PNG payload'
        with patch('edl_flight_computer.shared.landing_email.smtplib.SMTP_SSL') as connect:
            first = connect.return_value
            second = type(first)()
            connect.side_effect = [first, second]
            initial_smtp = first.__enter__.return_value
            retry_smtp = second.__enter__.return_value
            initial_smtp.login.side_effect = SMTPServerDisconnected('Connection unexpectedly closed')
            retry_smtp.send_message.return_value = {}

            self.assertEqual(email.send(landing, plot), {})

        initial_smtp.login.assert_called_once_with('operator', 'test-only', initial_response_ok=True)
        initial_smtp.send_message.assert_not_called()
        retry_smtp.login.assert_called_once_with('operator', 'test-only', initial_response_ok=False)
        retry_smtp.send_message.assert_called_once()

    def test_save_failure_prevents_email(self):
        email = Mock()
        with patch('edl_flight_computer.shared.ground_station.save_png', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                save_run_report(Path('landing.png'), 'drop', 'earth', (), {}, email)
        email.send.assert_not_called()
