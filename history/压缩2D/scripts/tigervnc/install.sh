#!/bin/bash
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
    echo 'Run this installer with sudo.' >&2
    exit 1
fi
setup_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if ! grep -Eq '^[[:space:]]*WaylandEnable=false[[:space:]]*$' /etc/gdm3/custom.conf; then
    echo 'This installer requires GDM to use X11 (WaylandEnable=false).' >&2
    exit 1
fi
/usr/bin/apt-get install -y tigervnc-scraping-server tigervnc-tools x11-utils python3-dbus avahi-daemon
/usr/bin/install -d -m 0755 /usr/local/libexec /etc/tigervnc
/usr/bin/install -m 0755 "$setup_dir/console.py" /usr/local/libexec/tigervnc-console.py
/usr/bin/install -m 0644 "$setup_dir/tigervnc-console.service" /etc/systemd/system/tigervnc-console.service
if [[ -f /etc/tigervnc/console.passwd ]]; then
    /usr/bin/cp -p /etc/tigervnc/console.passwd "/etc/tigervnc/console.passwd.backup.$(date +%Y%m%d%H%M%S)"
fi
umask 077
printf '%s\n' '123' | /usr/bin/tigervncpasswd -f > /etc/tigervnc/console.passwd
/usr/bin/chmod 0600 /etc/tigervnc/console.passwd
if [[ -f /home/fishbone/.config/systemd/user/tigervnc-console.service ]]; then
    /usr/sbin/runuser -u fishbone -- /usr/bin/env XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus /usr/bin/systemctl --user disable --now tigervnc-console.service || true
fi
/usr/bin/systemctl daemon-reload
/usr/bin/systemctl enable --now avahi-daemon.service ssh.service
/usr/bin/systemctl enable tigervnc-console.service
/usr/bin/systemctl restart tigervnc-console.service
/usr/bin/systemctl --no-pager --full status tigervnc-console.service
echo 'TigerVNC installed. Viewer: fishbone-wifi.local::5900; VNC password: 123'
echo 'SSH: ssh fishbone@fishbone-wifi.local (use the existing Ubuntu account password)'
