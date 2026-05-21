import rclpy, json, socket, time
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = ("127.0.0.1", 8094)
        self.robot = "Luloc"
        self.create_subscription(String, '/sumo5/diag', self.cb_diag, 10)
        self.create_subscription(String, '/sumo5/sensors', self.cb_sensors, 10)
        self.create_subscription(String, '/sumo5/motors', self.cb_motors, 10)
        self.create_subscription(String, '/sumo5/status', self.cb_status, 10)
        self.get_logger().info('Bridge sumo_5 ready')

    def send(self, line):
        try: self.sock.sendto(line.encode(), self.udp_addr)
        except: pass

    def cb_diag(self, msg):
        d = json.loads(msg.data)
        ts = int(time.time() * 1e9)
        self.send(f'esp_diagnostics,host=esp32p4 latency={d["lat"]},offset={d["off"]},jitter={d["jit"]} {ts}')

    def cb_sensors(self, msg):
        d = json.loads(msg.data)
        ts = int(time.time() * 1e9)
        r = self.robot
        self.send(f'line_sensor,robot={r} s0={d["n0"]},s1={d["n1"]},s2={d["n2"]},s3={d["n3"]},s4={d["n4"]},s5={d["n5"]},s6={d["n6"]},s7={d["n7"]} {ts}')
        self.send(f'odometry,robot={r} velIZ={d["sl"]},posIZ={d["dl"]},velDR={d["sr"]},posDR={d["dr"]} {ts}')

    def cb_motors(self, msg):
        d = json.loads(msg.data)
        ts = int(time.time() * 1e9)
        r = self.robot
        self.send(f'motor_cal,robot={r} target_l={d["tl"]},target_r={d["tr"]},actual_l={d["al"]},actual_r={d["ar"]} {ts}')

    def cb_status(self, msg):
        d = json.loads(msg.data)
        ts = int(time.time() * 1e9)
        self.send(f'system,robot={self.robot} uptime_sec={d["up"]} {ts}')

def main():
    rclpy.init()
    node = TelemetryBridge()
    time.sleep(2)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
