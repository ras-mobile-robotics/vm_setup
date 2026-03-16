#!/bin/bash

# 1. Alias Setup
grep -qxF 'source ~/vm_setup/alias.sh' ~/.bashrc || echo 'source ~/vm_setup/alias.sh' >> ~/.bashrc

grep -qxF 'source /etc/turtlebot4_discovery/setup.bash' ~/.bashrc || echo 'source /etc/turtlebot4_discovery/setup.bash' >> ~/.bashrc

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


# 5. FastDDS Discovery Server Service
SERVICE_FILE="/etc/systemd/system/fastdds-discovery.service"
# Captures the user who invoked the script, even if run with sudo
LOGGED_IN_USER=$(whoami)

if [ ! -f "$SERVICE_FILE" ]; then
    echo -e "$INFO Configuring FastDDS Discovery Server for user: $LOGGED_IN_USER..."

    sudo bash -c "cat <<EOF > $SERVICE_FILE
[Unit]
Description=FastDDS Discovery Server for ROS 2 Simulation
After=network.target

[Service]
User=$LOGGED_IN_USER
Type=simple
ExecStart=/bin/bash -c \"source /opt/ros/jazzy/setup.bash && /opt/ros/jazzy/bin/fast-discovery-server -i 0 -p 11811\"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF"

    sudo systemctl daemon-reload
    sudo systemctl enable fastdds-discovery.service
    echo -e "$OK FastDDS Discovery Server service started for $LOGGED_IN_USER"
else
    echo -e "$INFO FastDDS Discovery Server service already exists. Skipping configuration."
fi

echo "================= SETUP COMPLETE ===================="
echo "Your ROS_DOMAIN_ID is set to: $ROBOT_ID"
echo "Your Discovery Server is: $ROBOT_IP:11811"
echo "You can now connect using: ssh ubuntu@$ROBOT_NAME"
echo "====================================================="

echo ">>>>>>>>>>>>> RUN: source ~/.bashrc"