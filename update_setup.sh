#!/bin/bash

# Define files
SOURCE_FILE="/etc/turtlebot4_discovery/setup.bash"
DEST_FILE="$HOME/.domain_id"
STATE_FILE="$HOME/.vm_setup_state"

# Check if the source file exists
if [ -f "$SOURCE_FILE" ]; then

    # 1. Fix Domain ID by precedding precedding(ex: 03 -> 3) to prevent issues with RViz
    sudo sed -i 's/ROS_DOMAIN_ID=0\([0-9]\)/ROS_DOMAIN_ID=\1/g' $SOURCE_FILE

    # 2. Extract IDs and Server info
    # We use 'source' in a subshell to get the variables without affecting this script's environment
    DOMAIN_ID=$(grep "ROS_DOMAIN_ID=" "$SOURCE_FILE" | sed 's/.*ROS_DOMAIN_ID=\([0-9]*\).*/\1/')
    DISCOVERY_SERVER=$(grep "ROS_DISCOVERY_SERVER=" "$SOURCE_FILE" | sed -e 's/.*export ROS_DISCOVERY_SERVER="//' -e 's/".*//')
    
    # Save the ID to the hidden file as before
    echo "$DOMAIN_ID" > "$DEST_FILE"
    
    # 3. Get Git Version and Timestamp
    GIT_VERSION=$(git rev-parse --short HEAD 2>/dev/null || echo "not_a_repo")
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")

    # 4. Copy new tmux file
    cp ~/.tmux.conf ~/.tmux_$(date +%Y%m%d_%H%M%S) 
    rm ~/.tmux.conf
    ln -sf $HOME/vm_setup/tmux.conf ~/.tmux.conf

    # 5. Append the comma-separated line to the state file
    # Format: hash, timestamp, bot_id, domain_id, discovery_server
    echo "$GIT_VERSION, $TIMESTAMP, $DOMAIN_ID, $DISCOVERY_SERVER" >> "$STATE_FILE"


    echo "========== Updated =========="
fi