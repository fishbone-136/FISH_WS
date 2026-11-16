#!/usr/bin/python3
"""Share the active local X11 seat, including GDM, and advertise Wi-Fi DNS."""
import argparse
import fcntl
import os
from pathlib import Path
import pwd
import signal
import socket
import struct
import subprocess
import time

import dbus


def log(message):
    print(message, flush=True)


def command(*args):
    return subprocess.check_output(args, text=True, timeout=5).strip()


def identity_options(uid):
    if os.geteuid() == 0:
        account = pwd.getpwuid(uid)
        return dict(user=uid, group=account.pw_gid, extra_groups=[])
    if uid != os.getuid():
        raise PermissionError("The active seat belongs to another account")
    return {}


def active_session():
    session = command("/usr/bin/loginctl", "show-seat", "seat0",
                      "--property=ActiveSession", "--value")
    if not session:
        return None
    properties = dict(line.split("=", 1) for line in command(
        "/usr/bin/loginctl", "show-session", session,
        "--property=User", "--property=Type").splitlines())
    if properties.get("Type") != "x11":
        return None
    uid = int(properties["User"])
    authority = Path(f"/run/user/{uid}/gdm/Xauthority")
    stat = authority.stat()
    return session, uid, str(authority), stat.st_ino, stat.st_mtime_ns


def find_display(uid, authority):
    account = pwd.getpwuid(uid)
    environment = {"PATH": "/usr/bin:/bin", "HOME": account.pw_dir,
                   "USER": account.pw_name, "XAUTHORITY": authority}
    for endpoint in sorted(Path("/tmp/.X11-unix").glob("X[0-9]*")):
        display = ":" + endpoint.name[1:]
        probe = subprocess.run(
            ["/usr/bin/xdpyinfo", "-display", display], env=environment,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=4, **identity_options(uid))
        if probe.returncode == 0:
            environment["DISPLAY"] = display
            return environment
    raise RuntimeError("No accessible X11 display for the active seat")


class WifiName:
    def __init__(self, interface, name):
        self.interface, self.name = interface, name
        self.bus = None
        self.group = None
        self.address = None

    def reset(self):
        if self.group is not None:
            try:
                self.group.Free()
            except dbus.DBusException:
                pass
        self.group = None
        self.bus = None
        self.address = None

    def update(self):
        try:
            index = socket.if_nametoindex(self.interface)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                value = fcntl.ioctl(sock.fileno(), 0x8915,
                                    struct.pack("256s", self.interface.encode()))
            address = socket.inet_ntoa(value[20:24])
        except OSError:
            if self.group is not None:
                self.reset()
                log("Wi-Fi disconnected; withdrew Wi-Fi hostname")
            return
        if self.group is not None and self.address == address:
            if int(self.group.GetState()) in (1, 2):
                return
        self.reset()
        self.bus = dbus.SystemBus(private=True)
        server = dbus.Interface(self.bus.get_object("org.freedesktop.Avahi", "/"),
                                "org.freedesktop.Avahi.Server")
        if int(server.GetState()) != 2:
            return
        path = server.EntryGroupNew()
        self.group = dbus.Interface(
            self.bus.get_object("org.freedesktop.Avahi", path),
            "org.freedesktop.Avahi.EntryGroup")
        # Publish only on Wi-Fi and do not replace Avahi's reverse DNS record.
        self.group.AddAddress(dbus.Int32(index), dbus.Int32(0), dbus.UInt32(16),
                              self.name, address)
        self.group.Commit()
        self.address = address
        log(f"Wi-Fi hostname: {self.name} -> {address}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--wifi", default="wlP1p1s0")
    parser.add_argument("--name", default="fishbone-wifi.local")
    args = parser.parse_args()
    runtime = Path(args.runtime)
    runtime.mkdir(mode=0o755, parents=True, exist_ok=True)
    password = Path(args.password).read_bytes()
    if len(password) != 8:
        raise RuntimeError("Expected an eight-byte VNC password file")
    wifi = WifiName(args.wifi, args.name)
    child = None
    current = None
    last_error = None
    stopping = False

    def stop_signal(_number, _frame):
        nonlocal stopping
        stopping = True

    def stop_child():
        nonlocal child
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        child = None

    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)
    try:
        while not stopping:
            try:
                wifi.update()
            except (dbus.DBusException, OSError) as error:
                wifi.reset()
                log(f"Wi-Fi hostname retry: {error}")
            try:
                selected = active_session()
                if selected != current or (child is not None and child.poll() is not None):
                    stop_child()
                    current = None
                if selected is not None and child is None:
                    session, uid, authority, *_ = selected
                    environment = find_display(uid, authority)
                    password_path = runtime / f"passwd-{uid}"
                    fd = os.open(password_path, os.O_WRONLY | os.O_CREAT |
                                 os.O_TRUNC | os.O_NOFOLLOW, 0o600)
                    with os.fdopen(fd, "wb") as output:
                        output.write(password)
                        os.fchmod(output.fileno(), 0o600)
                        if os.geteuid() == 0:
                            os.fchown(output.fileno(), uid, pwd.getpwuid(uid).pw_gid)
                    child = subprocess.Popen(
                        [args.server, "-display", environment["DISPLAY"],
                         "-rfbport", "5900", "-localhost=0",
                         "-SecurityTypes", "TLSVnc", "-PasswordFile", str(password_path),
                         "-AlwaysShared=1", "-desktop", "fishbone desktop"],
                        env=environment, **identity_options(uid))
                    current = selected
                    log(f"Sharing seat0 session={session} uid={uid} display={environment['DISPLAY']}")
                last_error = None
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
                stop_child()
                current = None
                if str(error) != last_error:
                    log(f"Waiting for local X11 login screen or desktop: {error}")
                    last_error = str(error)
            time.sleep(2)
    finally:
        stop_child()
        wifi.reset()


if __name__ == "__main__":
    main()
