import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
import time

class BatteryModel(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryState, '/battery_state', 10)
        self.subscriber_ = self.create_subscription(Bool, '/is_docked', self.isDocked_callback, 10)
        self.time_step_s = 0.5
        self.timer = self.create_timer(self.time_step_s, self.timer_callback)

        self.design_capacity = 40.  # Battery design capacity in Ah
        self.current_charge_ah = self.design_capacity * 0.95  # Initial charge in Ah
        self.soc_initial = self.current_charge_ah / self.design_capacity  # Initial SoC (0.0 to 1.0)
        
        self.charging_state = 0
        self.discharge_current = 20.
        self.charge_current = -15.
        self.current_soc = self.soc_initial

        self.battery = BatteryState()
        self.battery.voltage = 24.0
        self.battery.temperature = 25.
        self.battery.current = -1*self.discharge_current
        self.battery.charge = self.current_charge_ah
        self.battery.capacity = self.design_capacity
        self.battery.design_capacity =  self.design_capacity
        self.battery.percentage = self.soc_initial
        self.battery.power_supply_health = 1
        self.battery.power_supply_status = 2
        self.battery.power_supply_technology = 2
        self.battery.present = True

         
        self.get_logger().info('Battery Model Node has been started.')

    def timer_callback(self):
            
        if self.charging_state:
            # Calculate SoC change for this time step
            delta_charge_ah = (self.charge_current * self.time_step_s) / 3600
            self.battery.current = -1*self.charge_current
        else:
            # Calculate SoC change for this time step
            delta_charge_ah = (self.discharge_current * self.time_step_s) / 3600
            self.battery.current = -1*self.discharge_current

        self.current_charge_ah -= delta_charge_ah # Note: current is positive for discharge
        self.battery.charge = self.current_charge_ah
        current_soc = self.current_charge_ah / self.design_capacity
        
        # Clamp SoC between 0 and 1
        self.current_soc = max(0, min(1, current_soc))
        self.battery.percentage = self.current_soc
        # Calculate a simplified voltage based on SoC
        # This is a basic linear approximation for demonstration
        self.battery.voltage = 24. - (4.2 - 4.2 * self.current_soc)
        self.battery.header.stamp = self.get_clock().now().to_msg()
        self.battery.header.frame_id = "battery_state"
        self.publisher_.publish(self.battery)
        # self.get_logger().info('Publishing video frame') # Uncomment for debugging
    
        # self.get_logger().warn('Could not read frame from camera.')

    def isDocked_callback(self, msg):
         self.charging_state = msg.data
         

def main(args=None):
    rclpy.init(args=args)
    battery_model = BatteryModel()
    rclpy.spin(battery_model)
    battery_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()