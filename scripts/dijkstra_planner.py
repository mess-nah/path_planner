import rclpy
from rclpy.node import Node 
from std_msgs.msg import Int32MultiArray
import heapq
from std_msgs.msg import String


class DijkstraPlanner(Node):

    def __init__(self):
        super().__init__('dijkstra_planner')

        self.publisher = self.create_publisher(Int32MultiArray, '/planned_path', 10)
        self.graph= {
            '2': [('5', 1000), ('1', 1), ('3', 2)],
            '1': [('4', 2), ('2', 2)],
            '3': [('6', 50), ('2', 2)],
            '4': [('5', 1000), ('7', 50)],
            '6': [('5', 1000), ('9', 1)],
            '9': [('12', 2), ('8', 2)],
            '8': [('11', 1), ('9', 2), ('7', 50)],
            '11': [('12', 2), ('10', 50), ('8', 2)],
            '7': [('10', 50), ('8', 2), ('4', 2)],
            '5': [('6', 50), ('4', 2), ('8', 2), ('2', 2)],
            '10': [('7', 50), ('11', 1)],
            '12': [('9', 1), ('11', 1)]
        }

        self.start_node = '2'
        self.goal_nodes= ['12','10' ]

        self.timer = self.create_timer(1.0, self.publish_path)
    
    def dijkstra(self,graph, start):
        distances = {node: float('inf') for node in graph}
        distances[start] = 0
        predecessors = {node: None for node in graph}
        priority_queue = [(0, start)]
    
        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
                
            if current_distance > distances[current_node]:
                    continue
                
            for neighbor, weight in graph[current_node]:
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    predecessors[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))
        
        return distances, predecessors

    def reconstruct_paths(self, predecessors, goals):
        all_paths = {}
        for goal in goals:
            path = []
            current = goal
            while current is not None:
                path.insert(0, current)
                current = predecessors[current]
            all_paths[goal] = path
        return all_paths


    
    def publish_path(self):
        distances, predecessors = self.dijkstra(self.graph, self.start_node)
        paths = self.reconstruct_paths(predecessors, self.goal_nodes)

        valid_goals = [g for g in self.goal_nodes if g in paths and len(paths[g])>0]
        if not valid_goals:
            self.get_logger().warn("No valid path to any goal")
            return

        goal = min(valid_goals, key= lambda x:distances.get(x, float('inf')))
        
        
        path = paths[goal]

        msg = Int32MultiArray()
        msg.data = [int(node) for node in path]

        self.publisher.publish(msg)
        self.get_logger().info(f"Published path: {path}")

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    node.destroy_npde()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

