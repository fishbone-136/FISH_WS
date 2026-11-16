#!/bin/sh
# NetworkManager runs this as root after activation, DHCP renewal, or resume.
# Also enforce the kernel setting if a wireless driver resets it on reconnect.
set -eu
interface=${1:-}
action=${2:-}
case "$action" in
    up|dhcp4-change|dhcp6-change|reapply) ;;
    *) exit 0 ;;
esac
[ -d "/sys/class/net/$interface/wireless" ] || exit 0
/usr/sbin/iw dev "$interface" set power_save off
