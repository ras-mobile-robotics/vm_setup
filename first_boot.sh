#!/bin/bash

# --- TEACHER CONFIGURATION ---
# Update these before distributing the VM to students
CLASS_SSID="Your_Router_SSID"
CLASS_PASS="Your_Router_Password"
# -----------------------------

clear
echo "====================================================="
echo "        TURTLEBOT 4 STUDENT VM SETUP TOOL"
echo "====================================================="

# 1. Get Student Number (Validation: 0-99)
while true; do
    read -p "Enter your assigned student number (0-99): " ST_NUM
    if [[ "$ST_NUM" =~ ^[0-9]+$ ]] && [ "$ST_NUM" -ge 0 ] && [ "$ST_NUM" -le 99 ]; then
        break
    else
        echo "Error: Please enter a whole number between 0 and 99."
    fi
done

# 2. Set Hostname (e.g., ubuntuvm42)
# can be letters (a-z), digits (0-9), and hyphens (-)
# cannot have underscores, special characters, start or end with a hyphen
NEW_HOSTNAME="ubuntuvm$ST_NUM"
echo "--> Setting identity to $NEW_HOSTNAME..."
sudo hostnamectl set-hostname "$NEW_HOSTNAME"
sudo sed -i "s/127.0.1.1.*/127.0.1.1\t$NEW_HOSTNAME/g" /etc/hosts

# # 3. Get ROS_DOMAIN_ID (Validation: 0-101)
# while true; do
#     read -p "Enter your assigned ROS_DOMAIN_ID (0-101): " DOMAIN_ID
#     if [[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] && [ "$DOMAIN_ID" -ge 0 ] && [ "$DOMAIN_ID" -le 101 ]; then
#         break
#     else
#         echo "Error: ROS_DOMAIN_ID must be a number between 0 and 101."
#     fi
# done

# Apply ROS_DOMAIN_ID to .bashrc (removes old entries first)
# sed -i '/ROS_DOMAIN_ID/d' /home/eva/.bashrc
# echo "export ROS_DOMAIN_ID=$DOMAIN_ID" >> /home/eva/.bashrc

# 4. Connect to WiFi
echo "--> Attempting to connect to WiFi: $CLASS_SSID..."
# Ensure the wifi is on
sudo nmcli radio wifi on
sudo nmcli device wifi rescan
sudo nmcli device wifi connect "$CLASS_SSID" password "$CLASS_PASS"

# 5. Regenerate SSH Host Keys
echo "--> Regenerating SSH Host Keys..."
sudo ssh-keygen -A

# 6. Connection Heartbeat (Wait for IP)
echo "--> Verifying network connection..."
SUCCESS=false
for i in {1..15}; do
    MY_IP=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -n 1)
    if [ -n "$MY_IP" ]; then
        echo "    Connected! Your IP address is: $MY_IP"
        SUCCESS=true
        break
    fi
    echo "    Waiting for IP address... ($i/15)"
    sleep 2
done

if [ "$SUCCESS" = false ]; then
    echo "!! WARNING: Could not obtain an IP. Check your WiFi settings or router."
fi

# 7. Final Summary and Reboot
echo ""
echo "====================================================="
echo "SETUP SUMMARY:"
echo "  - Hostname:   $NEW_HOSTNAME"
echo "  - ROS Domain: $DOMAIN_ID"
echo "  - Network:    $CLASS_SSID"
echo "====================================================="
echo "The VM will now reboot to finalize your identity."
read -p "Press [ENTER] to reboot now..."

sudo reboot
