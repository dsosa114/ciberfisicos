import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircularTrajectoryNode(Node):
    def __init__(self):
        super().__init__("turtlesim_circular")
        self.get_logger().info("Circular trajectory node has started.")
        self._publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        cmd = Twist()

        cmd.linear.x = 1.0
        cmd.angular.z = 0.5 

        self._publisher.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()