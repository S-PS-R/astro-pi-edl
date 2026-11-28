"""Send landing reports through the Gmail API over HTTPS."""

from base64 import urlsafe_b64encode
from dataclasses import dataclass
from email.policy import SMTP
import json
import os
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from .landing_email import build_message

GMAIL_SCOPE = 'https://www.googleapis.com/auth/gmail.send'
SCOPES = (GMAIL_SCOPE, 'openid', 'https://www.googleapis.com/auth/userinfo.email')
SEND_MAIL_URL = 'https://gmail.googleapis.com/gmail/v1/users/me/messages/send'


@dataclass(frozen=True)
class GmailEmail:
    client_file: Path
    sender: str
    recipients: tuple[str, ...]

    def _access_token(self, *, interactive=False):
        if os.name != 'nt':
            raise ValueError('Gmail API sign-in uses Windows Credential Manager on the downlink PC.')
        try:
            from google.auth.exceptions import GoogleAuthError, RefreshError
            from google.auth.transport.requests import Request as GoogleRequest
            from google.oauth2.credentials import Credentials
            from google.oauth2.id_token import verify_oauth2_token
            from google_auth_oauthlib.flow import InstalledAppFlow
            from keyring.backends.Windows import WinVaultKeyring
            from keyring.errors import KeyringError
            from oauthlib.oauth2 import OAuth2Error
        except ImportError as error:
            raise ValueError('Install Gmail API support with python -m pip install -e ".[gmail]".') from error

        try:
            with self.client_file.open(encoding='utf-8') as file:
                client_config = json.load(file)
            config = client_config['installed']
            client_id = config['client_id']
            client_secret = config['client_secret']
            token_uri = config['token_uri']
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('Use the downloaded Google OAuth client JSON for a Desktop app.') from error

        vault = WinVaultKeyring()
        service = f'astro-pi-edl Gmail {client_id}'
        account = self.sender.casefold()
        request = GoogleRequest()
        try:
            refresh_token = vault.get_password(service, account)
            if refresh_token:
                credentials = Credentials(None, refresh_token=refresh_token, token_uri=token_uri,
                                          client_id=client_id, client_secret=client_secret, scopes=SCOPES)
                try:
                    credentials.refresh(request)
                except RefreshError as error:
                    if not interactive:
                        raise ValueError('Gmail authorization expired or was revoked. Run --check-email again.') from error
                else:
                    if credentials.refresh_token != refresh_token:
                        vault.set_password(service, account, credentials.refresh_token)
                    return credentials.token

            if not interactive:
                raise ValueError('Run the ground station with --check-email to sign in to Gmail before the demo.')
            flow = InstalledAppFlow.from_client_config(client_config, scopes=SCOPES, autogenerate_code_verifier=True)
            credentials = flow.run_local_server(port=0, login_hint=self.sender, prompt='consent')
            if not credentials.id_token or not credentials.refresh_token:
                raise ValueError('Google did not return a verified sign-in and refresh token. Run --check-email again.')
            identity = verify_oauth2_token(credentials.id_token, request, client_id)
            if not identity.get('email_verified') or identity.get('email', '').casefold() != account:
                raise ValueError(f'Sign in to Google using the sender configured in run.py ({self.sender}).')
            vault.set_password(service, account, credentials.refresh_token)
            return credentials.token
        except Warning as error:
            raise ValueError('Grant Gmail sending and email address access in the Google consent screen.') from error
        except (GoogleAuthError, KeyringError, OAuth2Error) as error:
            raise ValueError(f'Gmail authorization failed ({type(error).__name__}).') from error

    def check(self):
        self._access_token(interactive=True)
        return 'Gmail API sign-in succeeded.'

    def send(self, landing, plot_path):
        token = self._access_token()
        message = build_message(self.sender, self.recipients, landing, plot_path)
        raw = urlsafe_b64encode(message.as_bytes(policy=SMTP)).decode('ascii')
        request = Request(
            SEND_MAIL_URL, data=json.dumps({'raw': raw}).encode('utf-8'), method='POST',
            headers={'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'},
        )
        try:
            with urlopen(request, timeout=15) as response:
                result = json.load(response)
        except HTTPError as error:
            if error.code == 403:
                raise OSError('Gmail API denied sending. Check that the API is enabled and Gmail sending was approved.') from error
            raise
        if not result.get('id'):
            raise OSError('Gmail did not return a message ID.')
        return {}
