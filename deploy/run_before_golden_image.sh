#!/bin/bash
# The "Golden Image" Prep Script - Run as root/sudo
# Run this on your master VM right before you export it to an OVA. This script configures Netplan to use MAC addresses and wipes the unique IDs so they regenerate for the students.

# 1. Dynamically find the ethernet interface (excluding loopback 'lo')
INTERFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -v 'lo' | head -n 1)
echo "Detected interface: $INTERFACE"

# 2. Configure Netplan to use MAC address as DHCP ID (backup incase machine id doesn't work with some routers)
# This creates a fresh config that applies to ANY interface name found
echo "--> Configuring universal Netplan..."
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
# Using 'uninitialized' allows the system to generate a truly fresh ID on boot
echo "--> Wiping unique Machine-IDs..."
echo "uninitialized" | sudo tee /etc/machine-id
sudo rm -f /var/lib/dbus/machine-id
sudo ln -sf /etc/machine-id /var/lib/dbus/machine-id

# 4. Remove SSH Host keys
# This ensures each robot generates its own unique SSH identity.
echo "--> Removing SSH host keys..."
sudo rm /etc/ssh/ssh_host_*

# 5. CLOUD-INIT REMOVAL (Replaces 'cloud-init clean') ---
echo "--> Permanently removing Cloud-Init..."
sudo apt-get purge -y cloud-init
sudo rm -rf /etc/cloud/
sudo rm -rf /var/lib/cloud/
sudo rm -f /var/log/cloud-init*

# 6. Clear Shell History, logs and APT cache to save space
echo "--> Shrinking image size (logs and cache)..."
history -c && history -w && cat /dev/null > ~/.bash_history
sudo apt-get clean
sudo find /var/log -type f -exec truncate -s 0 {} \;

echo "====================================================="
echo "DONE: Master VM Prepared."
echo "ACTION: Power off NOW. Do not reboot before exporting."
echo "====================================================="
