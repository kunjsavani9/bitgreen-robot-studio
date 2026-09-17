---

## Email (Gmail) setup for signup OTP

New users verify their email with a 6-digit code sent over Gmail SMTP. Until this
is configured, **the signup page refuses new accounts**. Existing accounts can
still log in.

Gmail doesn't accept your normal password here. You need an **App Password**.

### 1. Turn on 2-Step Verification

App Passwords only exist on accounts with 2-Step Verification enabled.

1. Open https://myaccount.google.com/security
2. Under **How you sign in to Google**, click **2-Step Verification**.
3. Follow the prompts to turn it on.

### 2. Create an App Password

1. Open https://myaccount.google.com/apppasswords
2. Enter an app name, e.g. `BITGREEN Robot Studio`.
3. Click **Create**.
4. Copy the 16-character password shown, e.g. `abcd efgh ijkl mnop`.
   Google shows it only once.

> Can't find the App Passwords page? It is hidden when 2-Step Verification is
> off, and some work or school Google accounts have it disabled by the admin.
> Use a personal Gmail account instead.

### 3. Add it to `.env`

```bash
nano .env
```

```dotenv
ROS_SMTP_USER=yourname@gmail.com
ROS_SMTP_PASS=abcdefghijklmnop
```

Spaces in the password are stripped automatically, so either form works.
Never commit `.env`; it is already in `.gitignore`.

### 4. Test it before starting the server

```bash
source venv/bin/activate
set -a; source .env; set +a
python -c "import email_otp; print(email_otp.send_otp('yourname@gmail.com', '123456'))"
```

| Output | Meaning |
|---|---|
| `(True, 'sent')` | Working. Check your inbox (and spam) for the code |
| `(False, 'Email service not configured on the server.')` | `.env` wasn't loaded. Run `set -a; source .env; set +a` in the same terminal |
| `(False, 'Email login failed ...')` | Wrong address or App Password. Create a new App Password and try again |
| `(False, 'Could not send email: ...')` | Network issue. Port 587 may be blocked (common on college Wi-Fi); try a hotspot |

### 5. Restart the server

The server reads these variables only at startup:

```bash
pkill -f server.py
set -a; source .env; set +a
python server.py
```

Now http://localhost:8000/signup will email a code to new users. The code expires
after 1 minute; use **Resend** on the signup page if it runs out.
