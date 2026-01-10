# ROS 2 Environment Switcher between Turtlebot4 Real and Sim
function set-ros-env() {
    local STATE_FILE="$HOME/.ros_env_mode_state"

    # Prompt the user for confirmation
    read -p "Are you sure you want to kill all tmux tabs? (y/n): " choice

    # If it's NOT 'y', exit immediately.
    if [[ "$choice" != "y" ]]; then
        echo "Operation cancelled."
        exit 1
    fi

    # ROS2 CLEANUP FUNCTION
    function _ros_full_cleanup() {
        echo -n "Stopping local discovery server and all ROS 2 nodes... "
        
        # Stop the system service if we are switching away from SIM
        sudo systemctl stop fastdds-discovery.service > /dev/null 2>&1
        sleep 1
        
        # Standard kills
        pkill -f "_ros2_daemon" > /dev/null 2>&1
        pkill -f "ros2 " > /dev/null 2>&1
        pkill -f "ros" > /dev/null 2>&1

        # ROS2 Daemon
        ros2 daemon stop > /dev/null 2>&1
        
        sleep 1
        echo "Stopped!"
    }

    # Argument Handling
    if [[ "$1" == "sim" || "$1" == "robot" ]]; then
        _ros_full_cleanup
        echo "$1" >| "$STATE_FILE"
        echo ">>> ROS ENV Mode set to [ $1 ]."
        _apply_ros_env "$1"

        # Kill all tmux tabs
        tmux kill-server

        echo "-------------------------------------------"
        echo " ROS2 ENV MODE: [ $1 ]"
        echo " ROS2 DISCOVERY SERVER:  $ROS_DISCOVERY_SERVER"
        echo "-------------------------------------------"
        echo -e "\e[1;31mCLOSE ALL TERMINAL TABS TO MAKE SURE THE CORRECT ROS ENV MODE (sim/robot) IS REFLECTED ACROSS ALL TERMINALS\e[0m"

    else
        echo "Usage: set-ros-env [sim|robot]"
        echo "Current ROS ENV Mode: $1"
    fi
}

# INTERNAL APPLY LOGIC (Run on every new terminal)
function _apply_ros_env() {
    local MODE=$1

    # Base Configs
    if [ "$MODE" == "sim" ]; then
        source /opt/ros/jazzy/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        [ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
        export ROS_DOMAIN_ID=0
        export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
        
        # Start Discovery Server ONLY if it isn't already running
        if ! systemctl is-active --quiet fastdds-discovery.service; then
            sudo systemctl start fastdds-discovery.service
            echo " [SIM] Discovery Service started via systemd."
        fi

    elif [ "$MODE" == "robot" ]; then
        # Source the TurtleBot setup for its specific IP/Config
        # setup using ~/setup/configure_discovery.sh
        if [ -f /etc/turtlebot4_discovery/setup.bash ]; then
            source /etc/turtlebot4_discovery/setup.bash
        fi
    fi
    
    ros2 daemon start > /dev/null 2>&1
}

# AUTO-LOAD ON BOOT / NEW TERMINAL
# Default to 'sim' if file doesn't exist
if [ ! -f "$HOME/.ros_env_mode_state" ]; then
    echo "sim" >| "$HOME/.ros_env_mode_state"
fi

# Load the saved state silently
_apply_ros_env $(cat "$HOME/.ros_env_mode_state")
