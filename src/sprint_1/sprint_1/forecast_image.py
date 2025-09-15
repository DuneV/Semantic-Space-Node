#!/home/krita/mujoco-env/bin/python3

import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String, Bool
from utils.ollama_helper import ImageAnalyzer
from sensor_msgs.msg import Image
import cv2
import numpy as np


class ForecastFeedback(Node):
    def __init__(self):
        super().__init__('forecast_feedback_node')
        self.get_logger().info("Init Node Forecast")
        
        self.fail2getframe = False
        self.latest_image_msg = None

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
        self.fail2getframe = msg.data
        self.get_logger().debug(f"State changed to: {self.fail2getframe}")

        if self.fail2getframe and self.latest_image_msg is not None:
            self.process_latest_image()

    def capture_frame(self, msg):
        self.latest_image_msg = msg
        self.get_logger().debug("New frame captured.")

    def process_latest_image(self):
        """Decode image, analyze with Ollama, and publish correction."""
        if not self.latest_image_msg:
            self.get_logger().warn("No image available to analyze!")
            return

        try:
            if self.latest_image_msg.encoding == 'bgr8':
                img_array = np.frombuffer(self.latest_image_msg.data, dtype=np.uint8)
                img_cv = img_array.reshape((self.latest_image_msg.height, self.latest_image_msg.width, 3))
            elif self.latest_image_msg.encoding == 'rgb8':
                img_array = np.frombuffer(self.latest_image_msg.data, dtype=np.uint8)
                img_cv = img_array.reshape((self.latest_image_msg.height, self.latest_image_msg.width, 3))
                img_cv = cv2.cvtColor(img_cv, cv2.COLOR_RGB2BGR)  # Convert to BGR for OpenCV
            else:
                self.get_logger().error(f"Unsupported image encoding: {self.latest_image_msg.encoding}")
                return

            img_pil = Image.fromarray(cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB))
            analysis_result = self.image_analyzer.analyze_image(img_pil)

            correction_msg = String()
            correction_msg.data = f"Analysis result: {analysis_result}"
            self.correction_publisher.publish(correction_msg)
            self.get_logger().info(f"Published correction: {correction_msg.data}")

            # Optional: no es necesario guardar
            # img_pil.save("/tmp/failed_frame.jpg")
            # self.get_logger().info("Saved failed frame to /tmp/failed_frame.jpg")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

        finally:
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
        