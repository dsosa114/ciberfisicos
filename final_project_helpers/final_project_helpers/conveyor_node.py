import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
from custom_interfaces.msg import Request, Supply
from std_msgs.msg import Header

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        self.supply_in_transit = False
        
        # QoS profile for publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.supply_publisher = self.create_publisher(Supply, '/supply', qos_profile)
        
        # Subscribers
        self.request_subscriber = self.create_subscription(
            Request,
            '/request',
            self.request_callback,
            qos_profile
        )
        
        self.get_logger().info('Conveyor is ready.')

    def request_callback(self, msg):
        if not self.supply_in_transit:
            self.get_logger().info(f'Received request for material {msg.material_id}. Starting transport from warehouse.')
            self.supply_in_transit = True
            self.get_logger().info('Transporting material...')
            # Simulate transport time
            time.sleep(45)
            self.publish_supply(msg)
            self.supply_in_transit = False

    def publish_supply(self, request_msg):
        supply_msg = Supply()
        supply_msg.header = Header()
        supply_msg.header.stamp = self.get_clock().now().to_msg()
        supply_msg.material_id = request_msg.material_id
        supply_msg.pickup_location = 'banda'
        
        self.supply_publisher.publish(supply_msg)
        self.get_logger().info(f'Material {supply_msg.material_id} is ready for pickup at {supply_msg.pickup_location}.')
        
def main(args=None):
    rclpy.init(args=args)
    conveyor_node = ConveyorNode()
    rclpy.spin(conveyor_node)
    conveyor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()