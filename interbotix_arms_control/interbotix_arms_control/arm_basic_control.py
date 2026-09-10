import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import TorqueEnable
from std_msgs.msg import Empty, Bool


class BasicController(Node):
    def __init__(self):
        super().__init__('interbotix_arms_basic_controller_node')

        self.declare_parameter('joint_group', 'arm') # <--- declare a parameter for the joint group
        self.declare_parameter('home_position', [0.0, 0.0, 0.0, 0.0, 0.0]) # <--- declare a parameter for the home position

        self.home_subscriber_ = self.create_subscription(Empty, 'home', self.home_callback, 10) # <--- create a subscriber to the 'home' topic
        self.activate_motors_subscriber_ = self.create_subscription(Bool, 'activate_motors', self.activate_motors_callback, 10) # <--- create a subscriber to the 'activate_motors' topic
        self.torque_client_ = self.create_client(TorqueEnable, 'torque_enable') # <--- create a client for the 'torque_enable' service
        self.publisher_ = self.create_publisher(JointGroupCommand, 'commands/joint_group', 10) # <--- create a publisher to the 'joint_group_command' topic
        self.get_logger().info('Node has been started') # <--- log the start of the node

    def home_callback(self, msg):
        joint_group = self.get_parameter('joint_group').get_parameter_value().string_value # <--- get the joint group parameter
        home_position = self.get_parameter('home_position').get_parameter_value().double_array_value # <--- get the home position parameter

        command_msg = JointGroupCommand() # <--- create a new JointGroupCommand message
        command_msg.name = joint_group # <--- set the joint group name
        command_msg.cmd = list(home_position) # <--- set the command to the home position

        self.publisher_.publish(command_msg) # <--- publish the command message
        self.get_logger().info(f'Published home position for {joint_group}') # <--- log the published command

    def activate_motors_callback(self, msg):             
        while not self.torque_client_.wait_for_service(timeout_sec=1.0): # <--- wait for the reset service to be available
                self.get_logger().info('Waiting for reset service...')

        request = TorqueEnable.Request() # <--- create a new TorqueEnable request
        request.cmd_type = 'group'
        request.name = self.get_parameter('joint_group').get_parameter_value().string_value # <--- set the joint group name
        request.enable = msg.data # <--- set the torque_enable field to the received message data

        future = self.torque_client_.call_async(request) # <--- call the service asynchronously
        future.add_done_callback(self.service_response_callback) # <--- add a callback for when the service response is received

    def service_response_callback(self, future):
        try:
            response = future.result() # <--- get the service response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}') # <--- log any exceptions that occur during the service call

def main(args=None):
    rclpy.init(args=args) # <--- initialize ROS 2 communication
    bc_node = BasicController()
    rclpy.spin(bc_node) # <--- keep the node running until interrupted
    bc_node.destroy_node() # <--- clean up and destroy the node
    rclpy.shutdown() # <--- shut down ROS 2 communication

if __name__ == '__main__':
    main()