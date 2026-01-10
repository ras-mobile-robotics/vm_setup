#!/bin/bash
# The "Golden Image" Prep Script - Run as root/sudo
# Run this on your master VM right before you export it to an OVA. This script configures Netplan to use MAC addresses and wipes the unique IDs so they regenerate for the students.

# 1. Dynamically find the ethernet interface (excluding loopback 'lo')
INTERFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -v 'lo' | head -n 1)
echo "Detected interface: $INTERFACE"

# 2. Configure Netplan to use MAC address as DHCP ID (backup incase machine id doesn't work with some routers)
# This creates a fresh config that applies to ANY interface name found
cat <<EOF | sudo tee /etc/netplan/01-netcfg.yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    $INTERFACE:
      dhcp4: true
      dhcp-identifier: mac
EOF

sudo netplan apply

# 3. Wipe machine-id so it regenerates on student boot
sudo truncate -s 0 /etc/machine-id
sudo rm /var/lib/dbus/machine-id
sudo ln -s /etc/machine-id /var/lib/dbus/machine-id

# 4. Remove SSH Host keys
# This ensures each robot generates its own unique SSH identity.
sudo rm /etc/ssh/ssh_host_*

# 5. Clear Shell History and logs
history -c
sudo find /var/log -type f -exec truncate -s 0 {} \;

echo "Master VM Prepared. Power off and export now."
