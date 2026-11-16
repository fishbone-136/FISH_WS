#!/bin/bash
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
    echo 'Run this script with sudo.' >&2
    exit 1
fi
wifi_interface=wlP1p1s0
# Ubuntu's default-wifi-powersave-on.conf sorts AFTER numeric filenames.
# Use a name that sorts last, including for networks created in the future.
config_path=/etc/NetworkManager/conf.d/zz-remote-wifi-powersave.conf
setup_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
dispatcher_path=/etc/NetworkManager/dispatcher.d/90-remote-wifi-powersave
umask 077
backup_dir=$(mktemp -d /var/backups/tigervnc-wifi.XXXXXXXX)
chmod 0700 "$backup_dir"
for path in "$config_path" "$dispatcher_path"; do
    if [[ -f "$path" ]]; then
        cp -p "$path" "$backup_dir/$(basename -- "$path")"
    fi
done
cp -a /etc/NetworkManager/system-connections "$backup_dir/"
# Save the individual values as well so they can be restored with nmcli.
while IFS=: read -r connection_uuid connection_type; do
    [[ "$connection_type" == 802-11-wireless ]] || continue
    original_value=$(nmcli -g 802-11-wireless.powersave connection show uuid "$connection_uuid")
    printf '%s %s\n' "$connection_uuid" "$original_value" >> "$backup_dir/powersave-before.txt"
done < <(nmcli -t -f UUID,TYPE connection show)
printf '[connection]\nwifi.powersave=2\n' > "$config_path"
chmod 0644 "$config_path"
install -o root -g root -m 0755 "$setup_dir/wifi-powersave-dispatcher.sh" "$dispatcher_path"
while read -r connection_uuid _; do
    nmcli connection modify uuid "$connection_uuid" 802-11-wireless.powersave 2
done < "$backup_dir/powersave-before.txt"
nmcli general reload conf
iw dev "$wifi_interface" set power_save off
iw dev "$wifi_interface" get power_save
echo "Original configuration backed up to: $backup_dir"
echo 'Wi-Fi power saving disabled for saved and future networks; connection was not restarted.'
