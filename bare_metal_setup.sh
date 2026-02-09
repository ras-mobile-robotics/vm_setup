#!/bin/bash

# 1. Alias Setup
grep -qxF 'source ~/vm_setup/alias.sh' ~/.bashrc || echo 'source ~/vm_setup/alias.sh' >> ~/.bashrc

# 2. SSH Key Generation
if [ ! -f ~/.ssh/id_ed25519 ]; then
    echo "--> Generating SSH keys..."
    ssh-keygen -t ed25519 -C "" -N "" -f ~/.ssh/id_ed25519
fi

# 3. Get Student/Robot Number
while true; do
    read -p "Enter the ROBOT_ID (exactly 2 digits, e.g., 05): " ROBOT_ID
    if [[ $ROBOT_ID =~ ^[0-9]{2}$ ]]; then
        break
    else
        echo "Error: Invalid input. Please enter a two-digit number (e.g., 05, 07, 12)."
    fi
done

# Assign variables
ROBOT_NAME="turtlebot"
ROBOT_IP="192.168.50.2$ROBOT_ID"

# 4. Add Robot to /etc/hosts
echo "--> Mapping $ROBOT_NAME to $ROBOT_IP in /etc/hosts..."
sudo sed -i "/$ROBOT_NAME/d" /etc/hosts
echo -e "$ROBOT_IP\t$ROBOT_NAME" | sudo tee -a /etc/hosts > /dev/null

### 4.1 Create /etc/turtlebot4_discovery/setup.bash
# Ensure the directory exists first
sudo mkdir -p /etc/turtlebot4_discovery

echo "--> Creating /etc/turtlebot4_discovery/setup.bash..."
sudo bash -c "cat << EOF > /etc/turtlebot4_discovery/setup.bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=$ROBOT_ID
export ROS_DISCOVERY_SERVER=\"$ROBOT_IP:11811\"
EOF"

echo "================= SETUP COMPLETE ===================="
echo "Your ROS_DOMAIN_ID is set to: $ROBOT_ID"
echo "Your Discovery Server is: $ROBOT_IP:11811"
echo "You can now connect using: ssh ubuntu@$ROBOT_NAME"
echo "====================================================="

echo ">>>>>>>>>>>>> RUN: source ~/.bashrc"