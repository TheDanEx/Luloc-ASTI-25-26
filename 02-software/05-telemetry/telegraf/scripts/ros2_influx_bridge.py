import rclpy, json, socket, time
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = ("127.0.0.1", 8094)
        self.robot = "Luloc"
        self.create_subscription(String, '/microROS/esp_diag_time', self.cb_diag, 10)
        self.create_subscription(String, '/sensors', self.cb_sensors, 10)
        self.create_subscription(String, '/motors', self.cb_motors, 10)
        self.create_subscription(String, '/status', self.cb_status, 10)
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
        self.send(f'line_sensor,robot={r} s0={d["n0"]},s1={d["n1"]},s2={d["n2"]},s3={d["n3"]},s4={d["n4"]},s5={d["n5"]},s6={d["n6"]},s7={d["n7"]},'
                  f'raw0={d.get("r0",0)},raw1={d.get("r1",0)},raw2={d.get("r2",0)},raw3={d.get("r3",0)},raw4={d.get("r4",0)},raw5={d.get("r5",0)},raw6={d.get("r6",0)},raw7={d.get("r7",0)},'
                  f'min0={d.get("min0",0)},min1={d.get("min1",0)},min2={d.get("min2",0)},min3={d.get("min3",0)},min4={d.get("min4",0)},min5={d.get("min5",0)},min6={d.get("min6",0)},min7={d.get("min7",0)},'
                  f'max0={d.get("max0",0)},max1={d.get("max1",0)},max2={d.get("max2",0)},max3={d.get("max3",0)},max4={d.get("max4",0)},max5={d.get("max5",0)},max6={d.get("max6",0)},max7={d.get("max7",0)} {ts}')
        self.send(f'odometry,robot={r} velIZ={d["sl"]},posIZ={d["dl"]},velDR={d["sr"]},posDR={d["dr"]} {ts}')

    def cb_motors(self, msg):
        d = json.loads(msg.data)
        ts = int(time.time() * 1e9)
        r = self.robot
        self.send(f'motor_cal,robot={r} target_l={d["tl"]},target_r={d["tr"]},actual_l={d["al"]},actual_r={d["ar"]},'
                  f'ff_l={d["ffl"]},p_l={d["pl"]},i_l={d["il"]},d_l={d["dl"]},'
                  f'ff_r={d["ffr"]},p_r={d["pr"]},i_r={d["ir"]},d_r={d["dr"]} {ts}')

    def cb_status(self, msg):
        d = json.loads(msg.data)
        ts = int(time.time() * 1e9)
        self.send(f'system,robot={self.robot} uptime_sec={d["up"]},'
                  f'cycle_mean_us={d.get("cyc_m",0)},cycle_max_us={d.get("cyc_x",0)},'
                  f'cycle_min_us={d.get("cyc_n",0)},cycle_overruns={d.get("cyc_o",0)} {ts}')

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
