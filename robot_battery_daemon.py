#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import os
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus

class RobotStatusDaemon(Node):
    def __init__(self):
        # Get robot namespace from stored file
        self.robot_id = self.get_robot_id()
        namespace = f"robot_{self.robot_id}"
        super().__init__('robot_status_daemon', namespace=namespace)

        # --- Path for TMUX Status ---
        self.status_file = "/tmp/robot_status"

        # --- State Variables ---
        self.current_battery_percentage = 0.0 
        self.is_docked = False
        
        # --- Subscriptions ---
        # Remapping /tf is handled via CLI or Launch, but these topics 
        # follow the node namespace 'robot_XX' automatically here.
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)
        
        self.dock_sub = self.create_subscription(
            DockStatus, 'dock_status', self.dock_callback, 10)

        self.get_logger().info(f'Status Daemon [{namespace}] Active: Monitoring Battery & Dock...')

    def get_robot_id(self):
        """Reads the ID from the local hidden file or environment and pads single digits."""
        id_file = os.path.expanduser("~/.domain_id")
        
        if os.path.exists(id_file):
            with open(id_file, 'r') as f:
                robot_id = f.read().strip()
        else:
            robot_id = os.environ.get('ROBOT_ID', 'XX')

        # If the ID is a digit (e.g., "3"), pad it to two characters (e.g., "03")
        if robot_id.isdigit():
            return robot_id.zfill(2)
        
        return robot_id

    def update_status_file(self):
        """Generates the string for the TMUX status bar and writes it to a file."""
        try:
            percent = int(self.current_battery_percentage * 100)
            
            # Select icon based on charge level
            if percent >= 90: batt_icon = "󰁹"
            elif percent >= 70: batt_icon = "󰂀"
            elif percent >= 50: batt_icon = "󰁾"
            elif percent >= 30: batt_icon = "󰁼"
            else: batt_icon = "󰂃"
            
            # Charging vs Mobile status
            dock_str = "󱐋 Charging" if self.is_docked else "󰑭 Mobile"
            t_stamp = time.strftime("%H:%M")
            
            # Format: [ID] < Icon 85% │ Status (12:00) >
            full_str = f"[{self.robot_id}] < {batt_icon} {percent}% │ {dock_str} ({t_stamp}) >\n"
            
            with open(self.status_file, 'w') as f:
                f.write(full_str)
        except Exception:
            pass

    def dock_callback(self, msg):
        self.is_docked = msg.is_docked
        self.update_status_file()

    def battery_callback(self, msg):
        self.current_battery_percentage = msg.percentage
        self.update_status_file()

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusDaemon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()