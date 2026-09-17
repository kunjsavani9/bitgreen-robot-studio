"""
ROS Ops Center — Authentication module (Stage 1)
SQLite user store + bcrypt password hashing + session tokens.

Admin is the reserved pair set by ROS_ADMIN_USERNAME / ROS_ADMIN_EMAIL (see .env.example).
Everyone else is a 'client'. One signup per email; usernames are unique and
constrained to a safe Linux-username pattern.
"""
import os, sqlite3, secrets, time, re, threading
from pathlib import Path

try:
    import bcrypt
except ImportError:  # pragma: no cover
    raise SystemExit("bcrypt is required: pip install bcrypt")

# ── Configuration ─────────────────────────────────────────────────────────────
DB_PATH = Path(__file__).parent / "ros_ops.db"

ADMIN_USERNAME = os.environ.get("ROS_ADMIN_USERNAME", "admin").strip()
ADMIN_EMAIL    = os.environ.get("ROS_ADMIN_EMAIL", "admin@example.com").strip()

# Safe as a Linux username AND a path segment: starts with a letter, 3-32 chars,
# lowercase letters / digits / underscore only.
USERNAME_RE = re.compile(r"^[a-z][a-z0-9_]{2,31}$")
# Stronger email check: local@domain.tld with a real-looking TLD (2+ letters),
# no consecutive dots, no leading/trailing dots in local or domain parts.
EMAIL_RE    = re.compile(
    r"^[A-Za-z0-9._%+\-]+@[A-Za-z0-9.\-]+\.[A-Za-z]{2,}$"
)

# Obvious placeholder / fake domains we reject outright.
_FAKE_EMAIL_DOMAINS = {
    "test.com", "test.test", "example.com", "example.org", "example.net",
    "email.com", "mail.com", "fake.com", "none.com", "no.com", "a.com",
    "abc.com", "xyz.com", "asdf.com", "qwerty.com", "domain.com",
}

def _email_ok(email: str) -> bool:
    email = (email or "").strip().lower()
    if not EMAIL_RE.match(email):
        return False
    if ".." in email:
        return False
    try:
        local, domain = email.rsplit("@", 1)
    except ValueError:
        return False
    if not local or local.startswith(".") or local.endswith("."):
        return False
    if domain.startswith(".") or domain.endswith(".") or domain.startswith("-"):
        return False
    labels = domain.split(".")
    if any(len(lbl) == 0 for lbl in labels):
        return False
    tld = labels[-1]
    if len(tld) < 2 or not tld.isalpha():
        return False
    if domain in _FAKE_EMAIL_DOMAINS:
        return False
    return True

SESSION_TTL = 7 * 24 * 3600  # 7 days

_lock = threading.Lock()


# ── DB setup ──────────────────────────────────────────────────────────────────
def _conn():
    c = sqlite3.connect(str(DB_PATH))
    c.row_factory = sqlite3.Row
    c.execute("PRAGMA foreign_keys = ON")
    return c


def init_db():
    with _lock, _conn() as c:
        c.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                username   TEXT UNIQUE NOT NULL,
                email      TEXT UNIQUE NOT NULL,
                pw_hash    TEXT NOT NULL,
                role       TEXT NOT NULL DEFAULT 'client',
                created_at INTEGER NOT NULL
            )
        """)
        c.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                token      TEXT PRIMARY KEY,
                username   TEXT NOT NULL,
                role       TEXT NOT NULL,
                created_at INTEGER NOT NULL,
                expires_at INTEGER NOT NULL,
                FOREIGN KEY(username) REFERENCES users(username) ON DELETE CASCADE
            )
        """)
        # Pending signups awaiting OTP verification. No real account exists until
        # the code is verified, so unverifiable emails never create accounts.
        c.execute("""
            CREATE TABLE IF NOT EXISTS pending_signups (
                email       TEXT PRIMARY KEY,
                username    TEXT NOT NULL,
                pw_hash     TEXT NOT NULL,
                role        TEXT NOT NULL DEFAULT 'client',
                code_hash   TEXT NOT NULL,
                expires_at  INTEGER NOT NULL,
                attempts    INTEGER NOT NULL DEFAULT 0,
                last_sent   INTEGER NOT NULL
            )
        """)
        c.commit()


# ── Helpers ───────────────────────────────────────────────────────────────────
def _hash_pw(password: str) -> str:
    return bcrypt.hashpw(password.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")


def _verify_pw(password: str, pw_hash: str) -> bool:
    try:
        return bcrypt.checkpw(password.encode("utf-8"), pw_hash.encode("utf-8"))
    except Exception:
        return False


def validate_signup(email: str, username: str, password: str):
    """Returns None if OK, else an error string."""
    email = (email or "").strip().lower()
    username = (username or "").strip().lower()
    if not USERNAME_RE.match(username):
        return "Username must be 3-32 chars: lowercase letters, digits, underscore; start with a letter."
    if not _email_ok(email):
        return "Please enter a valid email address."
    if len(password or "") < 6:
        return "Password must be at least 6 characters."
    # Protect the reserved admin identity from being taken by anyone else.
    if username == ADMIN_USERNAME and email != ADMIN_EMAIL:
        return "That username is reserved."
    if email == ADMIN_EMAIL and username != ADMIN_USERNAME:
        return "That email is reserved for the admin account."
    return None


def role_for(username: str, email: str) -> str:
    if username == ADMIN_USERNAME and email == ADMIN_EMAIL:
        return "host"
    return "client"


# ── Public API ────────────────────────────────────────────────────────────────
def signup(email: str, username: str, password: str):
    """Create a user. Returns (ok: bool, result: dict|str).
    On success result = {'username','email','role'}."""
    email = (email or "").strip().lower()
    username = (username or "").strip().lower()
    err = validate_signup(email, username, password)
    if err:
        return False, err
    role = role_for(username, email)
    pw_hash = _hash_pw(password)
    with _lock, _conn() as c:
        # Uniqueness checks (clear messages instead of raw IntegrityError)
        if c.execute("SELECT 1 FROM users WHERE username=?", (username,)).fetchone():
            return False, "That username is already taken."
        if c.execute("SELECT 1 FROM users WHERE email=?", (email,)).fetchone():
            return False, "An account already exists for that email."
        c.execute(
            "INSERT INTO users(username,email,pw_hash,role,created_at) VALUES(?,?,?,?,?)",
            (username, email, pw_hash, role, int(time.time())),
        )
        c.commit()
    return True, {"username": username, "email": email, "role": role}


# ── OTP signup verification ───────────────────────────────────────────────────
import secrets as _secrets, email_otp

OTP_TTL          = 60    # seconds (1 minute)
OTP_MAX_ATTEMPTS = 3     # wrong tries before the pending signup is invalidated
OTP_RESEND_AFTER = 30    # seconds before a resend is allowed

def _gen_code() -> str:
    return f"{_secrets.randbelow(1_000_000):06d}"

def begin_signup(email: str, username: str, password: str):
    """Validate inputs, create a PENDING signup, email a 6-digit code.
    No real account is created until verify_otp() succeeds.
    Returns (ok, result|error). On success result = {'email','username'}."""
    email = (email or "").strip().lower()
    username = (username or "").strip().lower()
    err = validate_signup(email, username, password)
    if err:
        return False, err

    # Admin is exempt from OTP — create directly.
    if username == ADMIN_USERNAME and email == ADMIN_EMAIL:
        ok, res = signup(email, username, password)
        if ok:
            return True, {"admin": True, "username": username, "email": email}
        return False, res

    if not email_otp.is_configured():
        return False, "Email verification is not available right now. Please try later."

    pw_hash = _hash_pw(password)
    now = int(time.time())
    code = _gen_code()
    code_hash = _hash_pw(code)
    role = role_for(username, email)

    with _lock, _conn() as c:
        # Block if a real account already exists.
        if c.execute("SELECT 1 FROM users WHERE username=?", (username,)).fetchone():
            return False, "That username is already taken."
        if c.execute("SELECT 1 FROM users WHERE email=?", (email,)).fetchone():
            return False, "An account already exists for that email."
        # Block if username is pending under a DIFFERENT email.
        row = c.execute("SELECT email FROM pending_signups WHERE username=?", (username,)).fetchone()
        if row and row["email"] != email:
            return False, "That username is already taken."
        # Upsert the pending signup (re-signup with same email refreshes it).
        c.execute("DELETE FROM pending_signups WHERE email=? OR username=?", (email, username))
        c.execute(
            "INSERT INTO pending_signups(email,username,pw_hash,role,code_hash,expires_at,attempts,last_sent) "
            "VALUES(?,?,?,?,?,?,?,?)",
            (email, username, pw_hash, role, code_hash, now + OTP_TTL, 0, now),
        )
        c.commit()

    sent_ok, msg = email_otp.send_otp(email, code)
    if not sent_ok:
        with _lock, _conn() as c:
            c.execute("DELETE FROM pending_signups WHERE email=?", (email,))
            c.commit()
        return False, msg
    return True, {"email": email, "username": username}

def verify_otp(email: str, code: str):
    """Verify the OTP for a pending signup and create the real account.
    Returns (ok, result|error). On success result = {'username','email','role'}."""
    email = (email or "").strip().lower()
    code = (code or "").strip()
    now = int(time.time())
    with _lock, _conn() as c:
        row = c.execute("SELECT * FROM pending_signups WHERE email=?", (email,)).fetchone()
        if not row:
            return False, "No pending signup for that email. Please sign up again."
        if row["expires_at"] < now:
            c.execute("DELETE FROM pending_signups WHERE email=?", (email,))
            c.commit()
            return False, "Code expired. Please request a new one."
        if row["attempts"] >= OTP_MAX_ATTEMPTS:
            c.execute("DELETE FROM pending_signups WHERE email=?", (email,))
            c.commit()
            return False, "Too many incorrect attempts. Please sign up again."
        if not _verify_pw(code, row["code_hash"]):
            c.execute("UPDATE pending_signups SET attempts=attempts+1 WHERE email=?", (email,))
            c.commit()
            left = OTP_MAX_ATTEMPTS - (row["attempts"] + 1)
            if left <= 0:
                c.execute("DELETE FROM pending_signups WHERE email=?", (email,))
                c.commit()
                return False, "Too many incorrect attempts. Please sign up again."
            return False, f"Incorrect code. {left} attempt(s) left."
        # Correct — create the real account, clear the pending row.
        username = row["username"]; pw_hash = row["pw_hash"]; role = row["role"]
        # Final uniqueness re-check (in case something registered meanwhile).
        if c.execute("SELECT 1 FROM users WHERE username=?", (username,)).fetchone():
            c.execute("DELETE FROM pending_signups WHERE email=?", (email,)); c.commit()
            return False, "That username was just taken. Please sign up again."
        if c.execute("SELECT 1 FROM users WHERE email=?", (email,)).fetchone():
            c.execute("DELETE FROM pending_signups WHERE email=?", (email,)); c.commit()
            return False, "An account already exists for that email."
        c.execute(
            "INSERT INTO users(username,email,pw_hash,role,created_at) VALUES(?,?,?,?,?)",
            (username, email, pw_hash, role, now),
        )
        c.execute("DELETE FROM pending_signups WHERE email=?", (email,))
        c.commit()
    return True, {"username": username, "email": email, "role": role}

def resend_otp(email: str):
    """Resend a fresh code for a pending signup (rate-limited). Returns (ok, msg)."""
    email = (email or "").strip().lower()
    now = int(time.time())
    with _lock, _conn() as c:
        row = c.execute("SELECT * FROM pending_signups WHERE email=?", (email,)).fetchone()
        if not row:
            return False, "No pending signup for that email. Please sign up again."
        if now - row["last_sent"] < OTP_RESEND_AFTER:
            wait = OTP_RESEND_AFTER - (now - row["last_sent"])
            return False, f"Please wait {wait}s before requesting another code."
        code = _gen_code()
        c.execute(
            "UPDATE pending_signups SET code_hash=?, expires_at=?, attempts=0, last_sent=? WHERE email=?",
            (_hash_pw(code), now + OTP_TTL, now, email),
        )
        c.commit()
    sent_ok, msg = email_otp.send_otp(email, code)
    if not sent_ok:
        return False, msg
    return True, "A new code has been sent."

def purge_expired_pending():
    """Housekeeping: drop expired pending signups."""
    now = int(time.time())
    with _lock, _conn() as c:
        c.execute("DELETE FROM pending_signups WHERE expires_at < ?", (now - 3600,))
        c.commit()


def login(username: str, password: str):
    """Verify credentials and issue a session token.
    Returns (ok, result). On success result = {'token','username','role'}."""
    username = (username or "").strip().lower()
    with _lock, _conn() as c:
        row = c.execute("SELECT * FROM users WHERE username=?", (username,)).fetchone()
        if not row or not _verify_pw(password, row["pw_hash"]):
            return False, "Invalid username or password."
        token = secrets.token_urlsafe(32)
        now = int(time.time())
        c.execute(
            "INSERT INTO sessions(token,username,role,created_at,expires_at) VALUES(?,?,?,?,?)",
            (token, username, row["role"], now, now + SESSION_TTL),
        )
        c.commit()
    return True, {"token": token, "username": username, "role": row["role"]}


def validate_token(token: str):
    """Returns {'username','role'} if the token is valid and unexpired, else None."""
    if not token:
        return None
    now = int(time.time())
    with _lock, _conn() as c:
        row = c.execute("SELECT * FROM sessions WHERE token=?", (token,)).fetchone()
        if not row:
            return None
        if row["expires_at"] < now:
            c.execute("DELETE FROM sessions WHERE token=?", (token,))
            c.commit()
            return None
    return {"username": row["username"], "role": row["role"]}


def logout(token: str):
    with _lock, _conn() as c:
        c.execute("DELETE FROM sessions WHERE token=?", (token,))
        c.commit()


def all_users():
    """Used later (Stage 2) for the startup reconcile step."""
    with _lock, _conn() as c:
        return [dict(r) for r in c.execute(
            "SELECT username,email,role,created_at FROM users").fetchall()]


def delete_user(username: str):
    """Permanently delete a user account and all their sessions from the DB.
    The admin account is protected. Returns (ok, message)."""
    username = (username or "").strip().lower()
    if username == ADMIN_USERNAME:
        return False, "The admin account cannot be deleted."
    with _lock, _conn() as c:
        row = c.execute("SELECT 1 FROM users WHERE username=?", (username,)).fetchone()
        if not row:
            return False, "No such user."
        c.execute("DELETE FROM sessions WHERE username=?", (username,))
        c.execute("DELETE FROM users WHERE username=?", (username,))
        c.commit()
    return True, "deleted"


# Initialise the DB as soon as the module is imported.
init_db()
