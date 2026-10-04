"""Shared TLS discovery, without importing TeleVuer/Isaac or reading key contents."""
import os
from pathlib import Path
import socket


def detect_host_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 9))
            return probe.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())


def certificate_paths():
    cert, key = os.getenv("XR_TELEOP_CERT"), os.getenv("XR_TELEOP_KEY")
    if bool(cert) != bool(key):
        raise RuntimeError("Set both XR_TELEOP_CERT and XR_TELEOP_KEY, or unset both")
    if cert:
        pair = Path(cert).expanduser(), Path(key).expanduser()
    else:
        folder = Path.home() / ".config/xr_teleoperate"
        pair = folder / "cert.pem", folder / "key.pem"
        if not all(p.is_file() for p in pair):
            # Preserve installed legacy wrapper discovery only as a fallback.
            import inspect
            from televuer.televuer import TeleVuer
            folder = Path(inspect.getfile(TeleVuer)).resolve().parents[2]
            pair = folder / "cert.pem", folder / "key.pem"
    if not all(p.is_file() for p in pair):
        raise FileNotFoundError(f"SSL certificate/key missing: {pair}. See README_TELEOP.md")
    return tuple(str(p) for p in pair)
