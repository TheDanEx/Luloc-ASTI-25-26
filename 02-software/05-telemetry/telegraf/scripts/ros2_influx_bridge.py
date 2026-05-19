import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32
import socket
import re
import time
import json

SENSOR_IDX = {
    'motor_speed_left': 0, 'motor_speed_right': 1,
    'motor_distance_left': 2, 'motor_distance_right': 3,
    'battery_voltage': 4, 'robot_current': 5,
    'line_position': 6, 'line_detected': 7, 'line_is_calibrated': 8,
    'norm0': 9, 'norm1': 10, 'norm2': 11, 'norm3': 12,
    'norm4': 13, 'norm5': 14, 'norm6': 15, 'norm7': 16,
}

MOTOR_IDX = {
    'target_l': 0, 'target_r': 1,
    'actual_l': 2, 'actual_r': 3,
}

STATUS_IDX = {
    'uptime_sec': 0, 'active_mode': 1,
    'cpu0_usage': 2, 'cpu1_usage': 3,
}

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.udp_ip = "127.0.0.1"
        self.udp_port = 8094
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_tag = "Luloc"

        self.sub_diag = self.create_subscription(
            String, '/microROS/esp_diag_time', self.diag_callback, 10)
        self.sub_sensors = self.create_subscription(
            Float32MultiArray, '/robot/sensors', self.sensors_callback, 10)
        self.sub_motors = self.create_subscription(
            Float32MultiArray, '/robot/motors', self.motors_callback, 10)
        self.sub_status = self.create_subscription(
            Float32MultiArray, '/robot/status', self.status_callback, 10)

        self.get_logger().info('ROS2 to InfluxDB Bridge started (sumo_5)')

    def _send_ilp(self, line: str):
        try:
            self.sock.sendto(line.encode(), (self.udp_ip, self.udp_port))
        except Exception as e:
            self.get_logger().warn(f'UDP send error: {e}')

    def diag_callback(self, msg):
        data = msg.data
        try:
            lat_match = re.search(r'"lat":([\d.]+)', data)
            off_match = re.search(r'"off":([\d.-]+)', data)
            jit_match = re.search(r'"jit":([\d.]+)', data)
            if lat_match and off_match and jit_match:
                ts = int(time.time() * 1e9)
                line = f'esp_diagnostics,host=esp32p4 latency={lat_match.group(1)},offset={off_match.group(1)},jitter={jit_match.group(1)} {ts}'
                self._send_ilp(line)
        except Exception as e:
            self.get_logger().error(f'Diag parse error: {e}')

    def sensors_callback(self, msg):
        d = msg.data
        if len(d) < 17:
            return
        ts = int(time.time() * 1e9)
        robot = self.robot_tag

        line_sensor = (
            f'line_sensor,robot={robot} '
            f's0={d[SENSOR_IDX["norm0"]]:.3f},s1={d[SENSOR_IDX["norm1"]]:.3f},'
            f's2={d[SENSOR_IDX["norm2"]]:.3f},s3={d[SENSOR_IDX["norm3"]]:.3f},'
            f's4={d[SENSOR_IDX["norm4"]]:.3f},s5={d[SENSOR_IDX["norm5"]]:.3f},'
            f's6={d[SENSOR_IDX["norm6"]]:.3f},s7={d[SENSOR_IDX["norm7"]]:.3f} '
            f'{ts}'
        )
        self._send_ilp(line_sensor)

        odometry = (
            f'odometry,robot={robot} '
            f'velIZ={d[SENSOR_IDX["motor_speed_left"]]:.4f},'
            f'posIZ={d[SENSOR_IDX["motor_distance_left"]]:.4f},'
            f'velDR={d[SENSOR_IDX["motor_speed_right"]]:.4f},'
            f'posDR={d[SENSOR_IDX["motor_distance_right"]]:.4f} '
            f'{ts}'
        )
        self._send_ilp(odometry)

    def motors_callback(self, msg):
        d = msg.data
        if len(d) < 4:
            return
        ts = int(time.time() * 1e9)
        robot = self.robot_tag

        motor_cal = (
            f'motor_cal,robot={robot} '
            f'target_l={d[MOTOR_IDX["target_l"]]:.4f},'
            f'target_r={d[MOTOR_IDX["target_r"]]:.4f},'
            f'actual_l={d[MOTOR_IDX["actual_l"]]:.4f},'
            f'actual_r={d[MOTOR_IDX["actual_r"]]:.4f} '
            f'{ts}'
        )
        self._send_ilp(motor_cal)

        line_follower = (
            f'line_follower,robot={robot} '
            f'target_l={d[MOTOR_IDX["target_l"]]:.4f},'
            f'target_r={d[MOTOR_IDX["target_r"]]:.4f},'
            f'actual_l={d[MOTOR_IDX["actual_l"]]:.4f},'
            f'actual_r={d[MOTOR_IDX["actual_r"]]:.4f} '
            f'{ts}'
        )
        self._send_ilp(line_follower)

    def status_callback(self, msg):
        d = msg.data
        if len(d) < 2:
            return
        ts = int(time.time() * 1e9)
        system_ilp = (
            f'system,robot={self.robot_tag} '
            f'uptime_sec={d[STATUS_IDX["uptime_sec"]]:.0f} '
            f'{ts}'
        )
        self._send_ilp(system_ilp)

def main(args=None):
    rclpy.init(args=args)
    bridge = TelemetryBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
