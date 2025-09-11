import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
import json

class Unzipper(Node):
    def __init__(self):
        super().__init__('unzipper_node')
        self.get_logger().info("Init Node Unzipp")
        self.subscription = self.create_subscription(
            String,                   
            '/yolo/detection_results',         
            self.listener_callback,     
            10            
        )
        self.publisher = self.create_publisher(
            String,
            '/processed_detections',
            10
        )
        self.subscription

    def listener_callback(self, msg):
        try:
            detection_data = json.loads(msg.data)
            detections = detection_data.get("detections", [])
            total_detections = detection_data.get("total_detections", len(detections))
            detection_strings = []
            for d in detections:
                detection_id = d.get("detection_id", "N/A")
                class_name = d.get("class_name", "unknown")
                confidence = d.get("confidence", 0.0)
                detection_strings.append(f"Detection {detection_id}: {class_name}, confidence {confidence:.2f}")
            phrase = f"Total objects: {total_detections}, {', '.join(detection_strings)}"
            # self.get_logger().info(f"Publishing phrase: {phrase}")
            msg_out = String()
            msg_out.data = phrase
            self.publisher.publish(msg_out)

            # self.get_logger().info(f"Parsed detection: {json.dumps(detection_data, indent=2)}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing detection: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Unzipper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()