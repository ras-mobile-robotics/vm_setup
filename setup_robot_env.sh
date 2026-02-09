#!/bin/bash

# 1. Alias Setup
# Add "source ~/vm_setup/alias.zsh" to bashrc if it does not exist
grep -qxF 'source ~/vm_setup/alias.sh' ~/.bashrc || echo 'source ~/vm_setup/alias.sh' >> ~/.bashrc

# 2. SSH Key Generation
# Only generate if the key doesn't already exist to avoid overwriting
if [ ! -f ~/.ssh/id_ed25519 ]; then
    echo "--> Generating SSH keys..."
    ssh-keygen -t ed25519 -C "" -N "" -f ~/.ssh/id_ed25519
fi

# 3. Get Student/Robot Number for Automation
# Loop until a valid 2-digit number is provided
while true; do
    read -p "Enter the ROBOT_ID (exactly 2 digits, e.g., 05): " ROBOT_ID

    # Check if input is exactly two digits (0-9)
    if [[ $ROBOT_ID =~ ^[0-9]{2}$ ]]; then
        break
    else
        echo "Error: Invalid input. Please enter a two-digit number (e.g., 05, 07, 12)."
    fi
done

# Assign variables based on input
ROBOT_NAME="turtlebot"
ROBOT_IP="192.168.50.2$ROBOT_ID"

# 4. Add Robot to /etc/hosts
# This lets you type 'ssh ubuntu@robot05' instead of the IP
echo "--> Mapping $ROBOT_NAME to $ROBOT_IP in /etc/hosts..."
sudo sed -i "/$ROBOT_NAME/d" /etc/hosts
echo -e "$ROBOT_IP\t$ROBOT_NAME" | sudo tee -a /etc/hosts > /dev/null

# 5. Set ROS Domain ID and Discovery Server in System Config
echo "--> Configuring ROS 2 Environment in /etc/turtlebot4_discovery/setup.bash..."
TB4_SETUP="/etc/turtlebot4_discovery/setup.bash"

# Update ROS_DOMAIN_ID
sudo sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$ROBOT_ID/" $TB4_SETUP

# Update ROS_DISCOVERY_SERVER
# Use | as a delimiter in sed here because the IP contains dots and colons
sudo sed -i "s|^export ROS_DISCOVERY_SERVER=.*|export ROS_DISCOVERY_SERVER=\"$ROBOT_IP:11811\"|" $TB4_SETUP

echo "System config updated successfully."


# 6. Set Robot Env in VM
shopt -s expand_aliases
source ~/.bashrc
set-ros-env robot
ros2 daemon stop; sleep 2; ros2 daemon start

# 7. Final Polish
source ~/.bashrc

echo "================= SETUP COMPLETE ===================="
echo "Your ROS_DOMAIN_ID is set to: $ROBOT_ID"
echo "You can now connect using: ssh ubuntu@$ROBOT_NAME"
echo "====================================================="