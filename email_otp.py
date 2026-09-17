"""
ROS Ops Center — Email OTP sender (signup verification only).
Sends a 6-digit code via Gmail SMTP. Credentials come from environment
variables and are NEVER hardcoded:

    export ROS_SMTP_USER="you@gmail.com"
    export ROS_SMTP_PASS="<16-char Gmail App Password, no spaces>"

If these are not set, is_configured() returns False and the server refuses
signup with a clear message (rather than creating unverified accounts).
"""
import os, smtplib, ssl
from email.message import EmailMessage

SMTP_HOST = "smtp.gmail.com"
SMTP_PORT = 587

def _user() -> str:
    return os.environ.get("ROS_SMTP_USER", "").strip()

def _pass() -> str:
    return os.environ.get("ROS_SMTP_PASS", "").replace(" ", "").strip()

def is_configured() -> bool:
    return bool(_user() and _pass())

def send_otp(to_email: str, code: str) -> tuple:
    """Send the OTP code. Returns (ok, message)."""
    if not is_configured():
        return False, "Email service not configured on the server."
    msg = EmailMessage()
    msg["Subject"] = "Your ROS Ops verification code"
    msg["From"] = _user()
    msg["To"] = to_email
    msg.set_content(
        f"Your ROS Ops Center verification code is: {code}\n\n"
        f"It expires in 1 minute. If you didn't request this, ignore this email."
    )
    msg.add_alternative(
        f"""
        <div style="font-family:Arial,sans-serif;max-width:420px;margin:auto">
          <h2 style="color:#0a7">ROS Ops Center</h2>
          <p>Your verification code is:</p>
          <div style="font-size:30px;font-weight:bold;letter-spacing:6px;
                      background:#f3f4f6;padding:14px;text-align:center;border-radius:8px">
            {code}
          </div>
          <p style="color:#777;font-size:13px">This code expires in 1 minute.
          If you didn't request this, you can ignore this email.</p>
        </div>
        """,
        subtype="html",
    )
    try:
        ctx = ssl.create_default_context()
        with smtplib.SMTP(SMTP_HOST, SMTP_PORT, timeout=20) as server:
            server.starttls(context=ctx)
            server.login(_user(), _pass())
            server.send_message(msg)
        return True, "sent"
    except smtplib.SMTPAuthenticationError:
        return False, "Email login failed (check ROS_SMTP_USER / app password)."
    except Exception as e:
        return False, f"Could not send email: {e}"
