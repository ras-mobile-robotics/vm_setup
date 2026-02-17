# Function to update TurtleBot Discovery settings
switch-robot() {
    if [ -z "$1" ]; then
        echo "Usage: set_robot <ID_1_to_15>"
        return 1
    fi

    # Format the ID with zero padding (e.g., 5 -> 05)
    local XX=$(printf "%02d" $1)
    local FILE="/etc/turtlebot4_discovery/setup.bash"

    echo "Setting Robot ID to $1 (IP suffix .2$XX)..."

    # Use sudo to update the file
    sudo sed -i "s/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$1/" $FILE
    sudo sed -i "s/export ROS_DISCOVERY_SERVER=.*/export ROS_DISCOVERY_SERVER=\"192.168.50.2$XX:11811\"/" $FILE

    # Write to domain ID file
    echo $1 >| ~/.domain_id

    # Source
    source $FILE

    # Show the result
    grep -E "ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER" $FILE
    echo "====================================="
}