import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(String, 'user_input_topic', 10)
        self.get_logger().info("User Input Publisher Node has started. Type 'order' to move from home to kitchen, 'table <n>' to deliver food to table <n>, or 'home' to return home.")

        # Store the current valid input
        self.current_input = None

        # Run the input loop
        self.user_input_loop()

    def user_input_loop(self):
        while rclpy.ok():
            if self.current_input:
                # If there is a valid input, keep publishing it
                msg = String()
                msg.data = self.current_input
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {self.current_input}")
                time.sleep(1)  # Delay for 1 second before publishing again

            # Ask for new user input when the previous input is None or changed
            user_input = input("Enter 'order', 'table <n>' (where <n> is 1-4), or 'home': ").strip().lower()

            # Validate the input
            if user_input == 'order':
                self.current_input = 'move from home to kitchen for collecting food'
                self.get_logger().info(f"Changed input to: {self.current_input}")
            elif user_input.startswith('table') and user_input[5:].strip().isdigit():
                table_number = user_input[6:].strip()
                if table_number in ['1', '2', '3', '4']:
                    self.current_input = f"move to table {table_number} to deliver food"
                    self.get_logger().info(f"Changed input to: {self.current_input}")
                else:
                    self.get_logger().warn("Invalid table number. Please enter 'table 1' to 'table 4'.")
            elif user_input == 'home':
                self.current_input = 'move from table to home position'
                self.get_logger().info(f"Changed input to: {self.current_input}")
            else:
                self.get_logger().warn("Invalid input. Please enter 'order', 'table <n>' (1-4), or 'home'.")

def main(args=None):
    rclpy.init(args=args)

    user_input_publisher = UserInputPublisher()

    try:
        rclpy.spin(user_input_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        user_input_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

