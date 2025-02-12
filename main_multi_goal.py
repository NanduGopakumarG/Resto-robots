from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String

class TableSubscriber(Node):
    def __init__(self, navigator):
        super().__init__('table_subscriber')
        self.navigator = navigator
        self.goal_poses = []

        # Initialize goal poses
        self.goal_home = PoseStamped()
        self.goal_kitchen = PoseStamped()
        self.goal_table1 = PoseStamped()
        self.goal_table2 = PoseStamped()
        self.goal_table3 = PoseStamped()
        self.goal_table4 = PoseStamped()

        # Subscribe to the user input topic
        self.create_subscription(String, '/user_input_topic', self.table_callback, 10)

        # Set the goal poses (locations for home, kitchen, and tables)
        self.set_default_goal_poses()

    def table_callback(self, msg):
        user_command = msg.data
        self.get_logger().info(f"Received command: {user_command}")
        
        if user_command == "order":
            # Move from home to kitchen
            self.navigator.goToPose(self.goal_kitchen)
        elif user_command.startswith('table'):
            # Check the table number and navigate to that table
            table_number = user_command.split()[-1]
            self.update_goal_pose_for_table(table_number)
            self.navigator.goToPose(self.goal_poses[2])  # goal_pose2 will be updated
        elif user_command == "home":
            # Move from table back to home
            self.navigator.goToPose(self.goal_home)
        else:
            self.get_logger().warn(f"Unknown command: {user_command}")

    def set_default_goal_poses(self):
        """Set initial positions for all goal poses."""
        # Goal for home position
        self.goal_home.header.frame_id = 'map'
        self.goal_home.pose.position.x = 0.0
        self.goal_home.pose.position.y = 0.0
        self.goal_home.pose.orientation.w = 1.0
        self.goal_home.pose.orientation.z = 0.0

        # Goal for kitchen position
        self.goal_kitchen.header.frame_id = 'map'
        self.goal_kitchen.pose.position.x = -1.0  # Example position
        self.goal_kitchen.pose.position.y = -2.0  # Example position
        self.goal_kitchen.pose.orientation.w = 1.0
        self.goal_kitchen.pose.orientation.z = 0.0

        # Default table positions (to be updated on callback)
        self.goal_table1.header.frame_id = 'map'
        self.goal_table2.header.frame_id = 'map'
        self.goal_table3.header.frame_id = 'map'
        self.goal_table4.header.frame_id = 'map'

        # Add all goal poses to the list (positions will be updated dynamically)
        self.goal_poses = [self.goal_home, self.goal_kitchen, self.goal_table1, self.goal_table2, self.goal_table3, self.goal_table4]

    def update_goal_pose_for_table(self, table_number):
        """Update the goal pose for the specified table."""
        table_positions = {
            '1': (1.0, -1.5),  # Example positions for tables
            '2': (2.0, -2.0),
            '3': (3.0, -2.5),
            '4': (4.0, -3.0)
        }

        if table_number in table_positions:
            x, y = table_positions[table_number]
            self.goal_poses[2].pose.position.x = x
            self.goal_poses[2].pose.position.y = y
            self.goal_poses[2].header.stamp = self.navigator.get_clock().now().to_msg()
            self.get_logger().info(f"Updated goal for table {table_number}: ({x}, {y})")
        else:
            self.get_logger().warn(f"Unknown table number: {table_number}")

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    table_subscriber = TableSubscriber(navigator)

    try:
        rclpy.spin(table_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        table_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

