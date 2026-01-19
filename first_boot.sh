#!/bin/bash

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

# 3. Remove any ROS_DOMAIN_ID and "source /etc/turtlebot4_discovery/setup.bash"(setup by configure_discover.sh)
# source ~/vm_setup/robot_env_switcher.sh will take care of that
sed -i '/ROS_DOMAIN_ID/d' /home/eva/.bashrc
sed -i.bak '/source \/etc\/turtlebot4_discovery\/setup.bash/d' ~/.bashrc

# 4. Regenerate SSH Host Keys
echo "--> Regenerating SSH Host Keys..."
sudo ssh-keygen -A

# 5. Final Summary and Reboot
echo ""
echo "====================================================="
echo "SETUP SUMMARY:"
echo "  - Hostname:   $NEW_HOSTNAME"
echo "  - ROS Domain: $ST_NUM"
echo "====================================================="
echo "The VM will now reboot to finalize your identity."
read -p "Press [ENTER] to reboot now..."

sudo reboot
