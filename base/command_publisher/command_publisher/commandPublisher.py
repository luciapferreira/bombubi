import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Function to get key presses
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_pub')
        self.publisher_ = self.create_publisher(Twist, 'robot_commands', 10)
        self.get_logger().info('Command Publisher node has been started.')
        self.run_manual_control()

    def publish_manual_command(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)
        self.get_logger().info(f'Manual command: linear_x={linear_x}, angular_z={angular_z}')

    def run_manual_control(self):
        try:
            while rclpy.ok():
                key = get_key()
                if key == 'w':
                    self.publish_manual_command(0.5, 0.0)  # Forward
                elif key == 's':
                    self.publish_manual_command(-0.5, 0.0)  # Backward
                elif key == 'a':
                    self.publish_manual_command(0.0, 0.5)  # Rotate left
                elif key == 'd':
                    self.publish_manual_command(0.0, -0.5)  # Rotate right
                elif key == 'q':
                    break
                else:
                    self.publish_manual_command(0.0, 0.0)  # Stop
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
