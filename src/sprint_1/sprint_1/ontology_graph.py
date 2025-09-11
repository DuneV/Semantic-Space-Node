import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import networkx as nx
from typing import List, Dict, Tuple, Any
import time
import json
import re

class OntologyNode(Node):
    def __init__(self):
        super().__init__('ontology_node')
        self.get_logger().info("Initializing Ontology Node")
        self.first_message_received = False 
        self.graph = self.build_ontology()
        self.concept_to_classes_cache = {}
        self.subscription = self.create_subscription(
            String,
            '/processed_detections',
            self.listener_callback,
            10
        )
        self.task = ""
        self.models = {
            0:['NOUN','VERB', 'ADP', 'NOUN'], 
            1:['VERB','NOUN'], 
            2:['NOUN', 'VERB', 'ADP', 'NOUN', 'VERB', 'ADP', 'NOUN']}
        self.subscription

    def read_task(self, path):
        task = ""

        with open(path, 'r', encoding='utf-8') as file:
            data = json.load(file)
    
        model_match = data.get("model_match")
        nouns = data.get("nouns", [])
        verbs = data.get("verbs", [])
        adps = data.get("adps", [])
        pattern = self.models.get(model_match, [])
        if not pattern:
            return ""
        idx_n, idx_v, idx_a = 0, 0, 0
        words = []
        for pos in pattern:
            if pos == "NOUN" and idx_n < len(nouns):
                words.append(nouns[idx_n])
                idx_n += 1
            elif pos == "VERB" and idx_v < len(verbs):
                words.append(verbs[idx_v])
                idx_v += 1
            elif pos == "ADP" and idx_a < len(adps):
                words.append(adps[idx_a])
                idx_a += 1
            else:
                words.append(f"<{pos}>")

        task = " ".join(words)
        return task


    def build_ontology(self) -> nx.MultiDiGraph:
        G = nx.MultiDiGraph()
        
        # Conceptos generales del grafo

        concepts = [
            "ElectronicDevice", "OfficeTool", "Furniture", "PrintedMaterial"
        ]

        # Se definen como conceptos
        for concept in concepts:
            G.add_node(concept, type="concept")

        G.add_edge("ElectronicDevice", "OfficeTool", key="subclass_of", relation="subclass_of")
        G.add_edge("Furniture", "OfficeTool", key="subclass_of", relation="subclass_of")
        G.add_edge("PrintedMaterial", "OfficeTool", key="subclass_of", relation="subclass_of")

        # Compatibilidades aqui no hay compatibilidades

        """ for k in ["Fruta","Lácteo","Endulzante","Hielo"]:
            G.add_edge(k,"Bebida", key="compatible_with", relation="compatible_with")
            G.add_edge("Bebida",k, key="compatible_with", relation="compatible_with") """

        classes = {
            "mouse": "ElectronicDevice",
            "keyboard": "ElectronicDevice",
            "tv": "ElectronicDevice",
            "desk": "Furniture",
            "chair": "Furniture",
            "monitor": "ElectronicDevice",
            "laptop": "ElectronicDevice",
            "book": "PrintedMaterial"
        }

        for cls, parent in classes.items():
            G.add_node(cls, type="class")
            G.add_edge(cls, parent, key="is_a", relation="is_a")

        # Actions
        actions = ["use_computer", "display_content", "sit", "move", "put"] # usar move

        # Here is not containers
        """ for container in ["vaso","jarra"]:
            G.add_edge(container, "Bebida", key="affords_contain", relation="affords", affordance="contener") """

        for action in actions:
            G.add_node(action, type="action")

        # Affordances
        G.add_edge("mouse", "use_computer", key="affords", relation="affords", affordance="use_computer")
        G.add_edge("keyboard", "use_computer", key="affords", relation="affords", affordance="use_computer")
        G.add_edge("tv", "display_content", key="affords", relation="affords", affordance="display_content")
        G.add_edge("monitor", "display_content", key="affords", relation="affords", affordance="display_content")
        G.add_edge("laptop", "use_computer", key="affords", relation="affords", affordance="use_computer")
        G.add_edge("chair", "sit", key="affords", relation="affords", affordance="sit")
        G.add_edge("book", "move", key="affords", relation="affords", affordance="move")

        # Object relationships
        for device in ["mouse", "keyboard", "monitor", "laptop", "book"]:
            G.add_edge(device, "desk", key="used_on", relation="used_on")

        return G

    def make_instance_id(self, label: str) -> str:
        return f"{label}#{int(time.time()*1000)%10_000_000}"

    def update_scene(self, detections: List[Dict[str, Any]], world_frame: str = "world") -> None:
        if not self.graph.has_node(world_frame):
            self.graph.add_node(world_frame, type="frame")

        # Eliminar instancias previas
        nodes_to_remove = [n for n, d in self.graph.nodes(data=True) if d.get("type") == "instance"]
        self.graph.remove_nodes_from(nodes_to_remove)

        # Insertar las nuevas detecciones
        for det in detections:
            label = det["label"]
            if not self.graph.has_node(label):
                self.graph.add_node(label, type="class")
            inst = self.make_instance_id(label)
            self.graph.add_node(inst, type="instance", class_of=label, conf=det.get("conf", 1.0))
            self.graph.add_edge(inst, label, key="instance_of", relation="instance_of")
            self.graph.add_edge(inst, world_frame, key="located_at", relation="located_at")


    def parse_intent(self, text: str) -> Dict[str, Any]:
        text = text.lower()
        if "computer" in text or "work" in text:
            return {
                "goal_concept": "ElectronicDevice",
                "constraints": {"use_computer": True},
                "preferred_tools": ["mouse", "keyboard", "laptop"],
                "required_surface": "desk"
            }
        if "move" in text or "mouse" in text:
            return {
                "goal_concept": "ElectronicDevice",
                "constraints": {"use_computer": False},
                "preferred_tools": ["mouse"],
                "required_surface": "keyboard"
            }
        return {"goal_concept": None}

    def find_instances(self, class_name: str) -> List[str]:
        instances = []
        for node, data in self.graph.nodes(data=True):
            if data.get("type") == "instance":
                for _, cls, key in self.graph.out_edges(node, keys=True):
                    if key == "instance_of" and cls == class_name:
                        instances.append(node)
        return instances

    def find_instances_of_concept(self, concept: str) -> List[Tuple[str, str]]:
        if concept not in self.concept_to_classes_cache:
            candidate_classes = set()
            for node, data in self.graph.nodes(data=True):
                if data.get("type") == "class":
                    seen = set([node])
                    frontier = [node]
                    reachable = False
                    while frontier:
                        current = frontier.pop()
                        for _, target, key, edge_data in self.graph.out_edges(current, keys=True, data=True):
                            if edge_data.get("relation") in ("is_a", "subclass_of") and target not in seen:
                                if target == concept:
                                    reachable = True
                                seen.add(target)
                                frontier.append(target)
                    if reachable:
                        candidate_classes.add(node)
            self.concept_to_classes_cache[concept] = candidate_classes

        results = []
        for cls in self.concept_to_classes_cache[concept]:
            for inst in self.find_instances(cls):
                results.append((inst, cls))
        return results

    def plan_move_mouse(self, intent: Dict[str, Any]) -> Dict[str, Any]:
        if intent.get("goal_concept") != "ElectronicDevice":
            return {"feasible": False, "reason": "Unsupported intent"}

        keyboard = self.find_instances("keyboard")
        if not keyboard:
            return {"feasible": False, "reason": "No keyboard available in the scene"}

        tools = []
        for tool in intent.get("preferred_tools", []):
            tools += self.find_instances(tool)

        if not tools:
            return {"feasible": False, "reason": "No tools available (mouse, keyboard, laptop)"}

        steps = [{"action": "move", "using": tools, "on": keyboard[0]}]
        return {
            "feasible": True,
            "surface": keyboard[0],
            "tools": tools,
            "steps": steps
        }


    def listener_callback(self, msg):
        if self.first_message_received:
            return
        
        self.first_message_received = True
        
        try:
            match = re.match(r"Total objects: (\d+), (.*)", msg.data)
            if not match:
                self.get_logger().error(f"Invalid message format: {msg.data}")
                return

            total_objects = int(match.group(1))
            detections_str = match.group(2)
            detections = []
            for det_match in re.finditer(r"Detection (\d+): (\w+), confidence ([\d.]+)", detections_str):
                detection_id = int(det_match.group(1))
                class_name = det_match.group(2)
                confidence = float(det_match.group(3))
                detections.append({"label": class_name, "conf": confidence})

            self.get_logger().info(f"Parsed detections: {detections}")
            self.update_scene(detections)

            intent = self.parse_intent(self.read_task(path='src/sprint_1/data/json_embedding/last_analysis.json'))
            plan = self.plan_move_mouse(intent)
            self.get_logger().info(f"Plan: {plan}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OntologyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()