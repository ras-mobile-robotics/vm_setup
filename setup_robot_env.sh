#!/bin/bash

# 1. Alias Setup
# Add "source ~/vm_setup/alias.zsh" to bashrc if it does not exist
grep -qxF 'source ~/vm_setup/alias.zsh' ~/.bashrc || echo 'source ~/vm_setup/alias.zsh' >> ~/.bashrc

# 2. SSH Key Generation
# Only generate if the key doesn't already exist to avoid overwriting
if [ ! -f ~/.ssh/id_ed25519 ]; then
    echo "--> Generating SSH keys..."
    ssh-keygen -t ed25519 -C "" -N "" -f ~/.ssh/id_ed25519
fi

# 3. Get Student/Robot Number for Automation
# We use this to map the correct Robot IP and Domain
ST_NUM=$(hostname | grep -o '[0-9]\+')
ROBOT_NAME="robot$ST_NUM"
ROBOT_IP="192.168.50.20$ST_NUM" # Assuming .203, .205, etc. matches ID

# 4. Add Robot to /etc/hosts
# This lets you type 'ssh ubuntu@robot05' instead of the IP
echo "--> Mapping $ROBOT_NAME to $ROBOT_IP in /etc/hosts..."
sudo sed -i "/$ROBOT_NAME/d" /etc/hosts
echo -e "$ROBOT_IP\t$ROBOT_NAME" | sudo tee -a /etc/hosts > /dev/null

# 5. Set ROS Domain ID and Discovery Server in System Config
echo "--> Configuring ROS 2 Environment in /etc/turtlebot4/setup.bash..."
TB4_SETUP="/etc/turtlebot4/setup.bash"

# Update ROS_DOMAIN_ID
sudo sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$ST_NUM/" $TB4_SETUP

# Update ROS_DISCOVERY_SERVER
# Use | as a delimiter in sed here because the IP contains dots and colons
sudo sed -i "s|^export ROS_DISCOVERY_SERVER=.*|export ROS_DISCOVERY_SERVER=\"$ROBOT_IP:11811\"|" $TB4_SETUP

echo "System config $TB4_SETUP updated successfully."

# 6. Final Polish
source ~/.bashrc
echo "====================================================="
echo "SETUP COMPLETE"
echo "Your ROS_DOMAIN_ID is set to: $ST_NUM"
echo "You can now connect using: ssh ubuntu@$ROBOT_NAME"
echo "====================================================="