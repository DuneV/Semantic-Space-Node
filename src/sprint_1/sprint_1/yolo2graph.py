#!/home/krita/mujoco-env/bin/python3

import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
from utils.processor import PromptProcessor


class Yolo2GraphNode(Node):
    def __init__(self):
        super().__init__('yolo2graph_node')
        self.get_logger().info("Init Node JC")
        self.get_logger().info("Created NLP")
        """ self.subscription = self.create_subscription(
            String,                   
            '/yolo/detection_results',         
            self.listener_callback,     
            10            
        ) """
        # self.get_logger().info(os.path.dirname(__file__))
        self.processor = PromptProcessor(lang='en', json_file='src/sprint_1/data/json_embedding/last_analysis.json')
        # self.subscription

    def listener_callback(self, msg):
        pass

    def publicar_accion(self, sujeto, verbo, objeto):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Yolo2GraphNode()

    try:
        while rclpy.ok():
            frase = input("Enter the task: ")
            if frase.strip(): 
                node.processor.process(frase)
            rclpy.spin_once(node, timeout_sec=0.1) 
    except KeyboardInterrupt:
        node.get_logger().info("Cerrar...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Nodo terminado.")

if __name__ == '__main__':
    main()