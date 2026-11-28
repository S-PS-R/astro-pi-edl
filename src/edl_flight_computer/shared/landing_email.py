"""Configure landing report delivery and build the shared email message."""

from contextlib import contextmanager
from dataclasses import dataclass, field
from email.errors import HeaderParseError
from email.headerregistry import Address
from email.message import EmailMessage
import os
from pathlib import Path
import smtplib
import ssl


@dataclass(frozen=True)
class LandingEmail:
    host: str
    port: int
    sender: str
    recipients: tuple[str, ...]
    username: str
    password: str = field(repr=False)

    @classmethod
    def from_settings(cls, sender, recipients, gmail_client_file=''):
        settings = os.environ
        sender = sender.strip()
        recipients = tuple(dict.fromkeys(address.strip() for address in recipients))
        if not sender and not recipients:
            return None
        if not sender or not recipients:
            raise ValueError('Set both sender and recipients in run.py to enable landing email.')
        for name, addresses in (('sender', (sender,)), ('recipients', recipients)):
            try:
                for address in addresses:
                    Address(addr_spec=address)
            except (ValueError, HeaderParseError):
                raise ValueError(f'Invalid email address in LANDING_EMAIL {name} in run.py.') from None
        if gmail_client_file.strip():
            from .gmail_email import GmailEmail

            return GmailEmail(Path(gmail_client_file).expanduser().resolve(), sender, recipients)

        host = settings.get('EDL_SMTP_HOST', '').strip()
        if not host:
            raise ValueError('Set EDL_SMTP_HOST to enable landing email.')
        port = int(settings.get('EDL_SMTP_PORT', '587'))
        if not 1 <= port <= 65535:
            raise ValueError('EDL_SMTP_PORT must be between 1 and 65535.')
        username = settings.get('EDL_SMTP_USER', '')
        password = settings.get('EDL_SMTP_PASSWORD', '')
        if bool(username) != bool(password):
            raise ValueError('Set EDL_SMTP_USER and EDL_SMTP_PASSWORD together.')
        return cls(host, port, sender, recipients, username, password)

    def send(self, landing, plot_path):
        message = build_message(self.sender, self.recipients, landing, plot_path)
        with self.connect() as smtp:
            return smtp.send_message(message)

    def check(self):
        with self.connect():
            return 'SMTP login succeeded.' if self.username else 'TLS connected. No SMTP credentials configured.'

    @contextmanager
    def connect(self):
        """Open a TLS connection and authenticate if credentials are configured."""
        context = ssl.create_default_context()
        for initial_response_ok in (True, False):
            if self.port == 465:
                connection = smtplib.SMTP_SSL(self.host, self.port, timeout=10, context=context)
            else:
                connection = smtplib.SMTP(self.host, self.port, timeout=10)
            with connection as smtp:
                if self.port != 465:
                    smtp.starttls(context=context)
                if self.username:
                    try:
                        smtp.login(self.username, self.password, initial_response_ok=initial_response_ok)
                    except smtplib.SMTPServerDisconnected as error:
                        # Reconnect once and wait for the server's AUTH challenge.
                        # Retry only before submitting the message.
                        if initial_response_ok:
                            continue
                        raise smtplib.SMTPServerDisconnected(
                            'SMTP connection closed during sign-in on both attempts.'
                        ) from error
                yield smtp
                return


def build_message(sender, recipients, landing, plot_path):
    data = landing.get('data', {})
    mission = landing['mission'].title()
    message = EmailMessage()
    message['From'] = sender
    message['To'] = ', '.join(recipients)
    message['Subject'] = f'{mission} landing confirmed'
    body = [f'{mission} landing confirmed for run {landing["run"]}.']
    if landing['mission'] == 'earth':
        flight = data.get('flight_time_s' if landing['type'] == 'event' else 'flight_elapsed_s')
        displacement = data.get('ekf_displacement_m')
        duration = f'{flight:.3f} s' if flight is not None else 'unavailable'
        distance = f'{displacement:+.3f} m' if displacement is not None else 'unavailable'
        body.extend([
            f'Flight time from release through landing confirmation (including settling): {duration}',
            f'Estimated vertical displacement from release (EKF): {distance}',
        ])
    else:
        body.append(f'Simulated touchdown time: T+{landing["time_s"]:.1f} s')
        speed = data.get('ground_speed_m_s', data.get('velocity_m_s'))
        if speed is not None:
            body.append(f'Touchdown speed: {speed:.2f} m/s')
        target_error = data.get('landing_target_error_m')
        if target_error is not None:
            body.append(f'Distance from landing target: {target_error:.2f} m')
    if data.get('csv'):
        body.append(f'Flight CSV: {data["csv"]}')
    message.set_content('\n'.join(body) + '\n')
    message.add_attachment(plot_path.read_bytes(), maintype='image', subtype='png',
                           filename=plot_path.name)
    return message
