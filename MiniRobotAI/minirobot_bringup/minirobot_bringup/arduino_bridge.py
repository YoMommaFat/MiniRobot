import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import serial
import json
import threading
import time
import math

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # --- Kinematic Parameters ---
        self.wheel_radius = 0.020        # 20mm
        self.wheel_separation = 0.092    # 92mm
        self.ticks_per_rev = 692.0
        # PWM Mapping: How much m/s is PWM 100? (Default guess: 0.35 m/s)
        self.max_speed_at_100_pwm = 0.35   
        
        # --- Serial Config ---
        self.declare_parameter('serial_port', '/dev/ttyAMA0') 
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # --- ROS 2 Interfaces ---
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.sensor_pub = self.create_publisher(String, '/arduino_sensors', 10)
        
        # --- Internal State ---
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_th = 0.0
        
        # Start background thread for reading serial data
        if self.ser:
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

    def cmd_vel_callback(self, msg):
        """
        Converts Linear/Angular velocity to Left/Right PWM [-100, 100]
        """
        v = msg.linear.x
        w = msg.angular.z
        
        # 1. Calculate individual wheel velocities (m/s)
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)
        
        # 2. Map m/s to PWM (-100 to 100)
        pwm_l = int((v_left / self.max_speed_at_100_pwm) * 100)
        pwm_r = int((v_right / self.max_speed_at_100_pwm) * 100)
        
        # 3. Constrain to limits
        pwm_l = max(min(pwm_l, 100), -100)
        pwm_r = max(min(pwm_r, 100), -100)
        
        # 4. Send JSON to Arduino
        payload = {"cmd": "mot", "val": [pwm_l, pwm_r]}
        self.send_json(payload)

    def send_json(self, data):
        if self.ser and self.ser.is_open:
            try:
                line = json.dumps(data) + "\n"
                self.ser.write(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"Failed to send serial data: {e}")

    def serial_read_loop(self):
        """
        Continuously reads JSON from Arduino and publishes to ROS topics
        """
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if not line: continue
                    
                    data = json.loads(line)
                    cmd = data.get("cmd")
                    val = data.get("val")
                    
                    if cmd == "pwr":
                        self.publish_battery(val)
                    elif cmd == "enc":
                        self.process_odometry(val)
                    
                    if cmd in ["bump", "cliff", "dist", "pwr"]:
                        msg = String()
                        msg.data = json.dumps({"type": cmd, "val": val})
                        self.sensor_pub.publish(msg)
                        
                except Exception as e:
                    # Ignore malformed lines
                    pass
            else:
                time.sleep(0.01)

    def publish_battery(self, val):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Akkumulátor feszültség:
        # V_bat = ADC * (2.56V / 1023) * 6 (mivel 1/6-os feszosztó van az OPAMP előtt)
        msg.voltage = float(val[0]) * (2.56 / 1023.0) * 6.0
        self.battery_pub.publish(msg)

    def process_odometry(self, val):
        # Odometry calculation logic will be added here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
