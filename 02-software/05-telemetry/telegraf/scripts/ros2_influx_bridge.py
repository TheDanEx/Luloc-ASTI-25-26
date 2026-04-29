import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import re
import time

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.subscription = self.create_subscription(
            String,
            '/microROS/esp_diag_time',
            self.listener_callback,
            10)
        
        self.udp_ip = "127.0.0.1"
        self.udp_port = 8094
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.get_logger().info('ROS 2 to InfluxDB Bridge Started - FINAL VERSION')

    def listener_callback(self, msg):
        data = msg.data
        try:
            # Parsear métricas (Lat, Off, Jit)
            lat_match = re.search(r'Lat: ([\d.]+)ms', data)
            off_match = re.search(r'Off: ([\d.-]+)ms', data)
            jit_match = re.search(r'Jit: ([\d.]+)ms', data)
            
            if lat_match and off_match and jit_match:
                latency = float(lat_match.group(1))
                offset = float(off_match.group(1))
                jitter = float(jit_match.group(1))
                
                timestamp_ns = int(time.time() * 1e9)
                line = f"esp_diagnostics,host=esp32p4 latency={latency},offset={offset},jitter={jitter} {timestamp_ns}"
                
                self.sock.sendto(line.encode(), (self.udp_ip, self.udp_port))
                # Log discreto para confirmación
                print(f"[SUCCESS] Metrics sent to Influx: Lat={latency}ms")
                
        except Exception as e:
            self.get_logger().error(f"Error parsing: {e}")

def main(args=None):
    rclpy.init(args=args)
    bridge = TelemetryBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
