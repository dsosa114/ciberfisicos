import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import random
from custom_interfaces.msg import Request, FinishedJob, Delivery
from std_msgs.msg import Header

class TableNode(Node):
    def __init__(self):
        super().__init__('table_node')
        
        # Declare ROS 2 parameters for node name and table_id
        self.declare_parameter('table_id', 'default_table')
        
        # Get parameter value and set internal state
        self.table_id = self.get_parameter('table_id').get_parameter_value().string_value
        
        self.product_type = f'product_{self.table_id}'
        self.material_type = f'material_{self.table_id}'
        self.max_jobs = 10
        self.jobs_in_pile = 0
        self.is_request_sent = False
        
        # Log the current node parameters for verification
        self.get_logger().info(f'Table node initialized with ID: {self.table_id} and Product Type: {self.product_type}')

        # QoS profile for publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.request_publisher = self.create_publisher(Request, '/request', qos_profile)
        self.finished_job_publisher = self.create_publisher(FinishedJob, '/finished_job', qos_profile)
        
        # Subscribers
        self.delivery_subscriber = self.create_subscription(
            Delivery,
            '/delivery',
            self.delivery_callback,
            qos_profile
        )
        
        # Timers
        self.production_timer = self.create_timer(1.0, self.production_loop)
        
        self.get_logger().info(f'Table {self.table_id} is ready.')

    def delivery_callback(self, msg):
        # Check if the delivery is for this table
        if msg.dropoff_location == self.table_id:
            self.get_logger().info(f'Received delivery of material {msg.product_id} at {msg.dropoff_location}.')
            self.is_request_sent = False

    def production_loop(self):
        # Request material if pile is low and no request is pending
        if self.jobs_in_pile < self.max_jobs and not self.is_request_sent:
            self.request_material()
        
        # Simulate production
        if random.random() < 0.05:  # 5% chance of finishing a job per second
            if self.jobs_in_pile < self.max_jobs:
                self.jobs_in_pile += 1
                self.get_logger().info(f'Table {self.table_id} finished a job. Pile size: {self.jobs_in_pile}.')
                self.finished_job_publisher.publish(self.create_finished_job_message())

    def request_material(self):
        msg = Request()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.material_id = self.material_type
        msg.destination_id = self.table_id
        
        self.request_publisher.publish(msg)
        self.is_request_sent = True
        self.get_logger().info(f'Requesting material {self.material_type} for table {self.table_id}.')

    def create_finished_job_message(self):
        msg = FinishedJob()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.product_id = self.product_type
        msg.location_id = self.table_id
        return msg
        
def main(args=None):
    rclpy.init(args=args)
    table_node = TableNode()
    rclpy.spin(table_node)
    table_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()