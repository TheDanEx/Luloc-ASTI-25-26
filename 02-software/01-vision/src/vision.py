#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response
import cv2
import threading

bridge = CvBridge()
app = Flask(__name__)
latest_frame = None

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('usb_cam_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global latest_frame
        latest_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Añadir un cuadrado de prueba
        cv2.rectangle(latest_frame, (50,50), (150,150), (0,0,255), 3)

def generate():
    """Función para streaming MJPEG"""
    global latest_frame
    while True:
        if latest_frame is None:
            time.sleep(0.01)  # espera 10ms si no hay frame
            continue
        ret, jpeg = cv2.imencode('.jpg', latest_frame)
        if not ret:
            continue
        frame = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=5123, threaded=True, debug=False, use_reloader=False)

def main():
    rclpy.init()
    node = ImageSubscriber()
    threading.Thread(target=flask_thread, daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()