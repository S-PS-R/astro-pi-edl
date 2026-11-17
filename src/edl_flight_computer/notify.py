"""Email a landing notification through Gmail SMTP (standard library only).

Set these environment variables before running (never commit them):
    EDL_SMTP_USER      Gmail address that sends the message
    EDL_SMTP_PASSWORD  16-character Google app password (not the account password)
    EDL_EMAIL_TO       Recipient(s), comma separated; defaults to EDL_SMTP_USER
Optional: EDL_SMTP_HOST (default smtp.gmail.com), EDL_SMTP_PORT (default 465).
"""

import os
import smtplib
import ssl
from email.message import EmailMessage
from threading import Thread


class LandingNotifier:
    def __init__(self, timeout_s=15.0):
        self.user = os.environ.get('EDL_SMTP_USER')
        self.password = os.environ.get('EDL_SMTP_PASSWORD')
        self.recipients = os.environ.get('EDL_EMAIL_TO') or self.user
        self.host = os.environ.get('EDL_SMTP_HOST', 'smtp.gmail.com')
        self.port = int(os.environ.get('EDL_SMTP_PORT', '465'))
        self.timeout_s = timeout_s
        self._thread = None

    @property
    def configured(self):
        return bool(self.user and self.password and self.recipients)

    def send(self, subject, body):
        """Send in the background so the network never delays the flight loop."""
        if not self.configured:
            print('Landing email skipped: set EDL_SMTP_USER and EDL_SMTP_PASSWORD.', flush=True)
            return
        message = EmailMessage()
        message['Subject'] = subject
        message['From'] = self.user
        message['To'] = self.recipients
        message.set_content(body)
        self._thread = Thread(target=self._send, args=(message,), name='EDL email')
        self._thread.start()

    def _send(self, message):
        try:
            context = ssl.create_default_context()
            with smtplib.SMTP_SSL(self.host, self.port, context=context,
                                  timeout=self.timeout_s) as server:
                server.login(self.user, self.password)
                server.send_message(message)
            print(f"Landing email sent to {message['To']}", flush=True)
        except (OSError, smtplib.SMTPException) as error:
            # The Pi may be offline after a drop; the mission must not fail because of it.
            print(f'Landing email failed: {error}', flush=True)

    def wait(self):
        """Give an in-flight send time to finish before the program exits."""
        if self._thread is not None:
            self._thread.join(timeout=self.timeout_s + 1)


def landing_message(planet_name, mission_time_s, velocity_m_s, peaks):
    lines = [f'{planet_name.upper()} touchdown at T+{mission_time_s:.1f} s, '
             f'{velocity_m_s:.1f} m/s.', '']
    for name, peak in peaks.items():
        if peak.get('confirmed'):
            lines.append(f"{name}: {peak['value']:.2f} {peak['units']} at {peak['time_s']:.1f} s")
    return '\n'.join(lines)
