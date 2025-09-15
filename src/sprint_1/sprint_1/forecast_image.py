#!/home/krita/mujoco-env/bin/python3

import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String, Bool
from utils.ollama_helper import ImageAnalyzer
from sensor_msgs.msg import Image



class ForecastFeedback(Node):
    def __init__(self):
        super().__init__('forecast_feedback_node')
        self.get_logger().info("Init Node Forecast")
        self.fail2getframe = False
        self.image_temp = Image()
        self.subscription_fail = self.create_subscription(
            Bool,
            '/graph/state',
            self.listener_callback,
            10
        )
        self.correction = self.create_publisher(
            String,
            '/node_corrections',
            10
        )
        self.subscription_image_frag = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.capture_frame,
            10
        )
        self.image_analyzer = ImageAnalyzer()
    
    def listener_callback(self, msg):
        pass
    
    def capture_frame(self):
        if not self.fail2getframe:
            pass
            self.fail2getframe = False


def main(args=None):

    rclpy.init(args=args)
    node = ForecastFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrar...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node dead")


if __name__ == '__main__':
    main()
        