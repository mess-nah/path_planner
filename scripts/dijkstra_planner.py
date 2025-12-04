import rclpy
from rclpy.node import Node 
from std_msgs.msg import Int32MultiArray
import heapq


class DijkstraPlanner(Node, _ar_boxes, _mr_boxes, _fake_box):

    def __init__(self):
        super().__init__('dijkstra_planner')

        self.W_AR = 20
        self.W_MR = 50
        self.W_FAKE = 999
        self.W_NORMAL = 1
 

        self.ar_nodes = None
        self.mr_nodes = None
        self.fake_nodes = None





        
        self.graph= {
            '2': [('5', 1), ('1', 1), ('3',1)],
            '1': [('4', 1), ('2', 1)],
            '3': [('6', 1), ('2', 1)],
            '4': [('5', 1), ('7', 1)],
            '6': [('5', 1), ('9', 1)],
            '9': [('12', 1), ('8', 1)],
            '8': [('11', 1), ('9', 1), ('7', 1)],
            '11': [('12', 1), ('10', 1)],
            '7': [('10', 1), ('8', 1)],
            '5': [('6', 1), ('4', 1), ('8', 1)],
            '10': [('11', 1)],
            '12': [('11', 1)]
        }

        self.start_node = '2'
        self.goal_nodes= ['12','10' ]
        self.publisher = self.create_publisher(Int32MultiArray, '/planned_path', 10)
        self.create_subscription(Int32MultiArray, '/ar_nodes', self.update_ar,10)
        self.create_subscription(Int32MultiArray, '/mr_nodes', self.update_mr, 10)
        self.create_subscription(Int32MultiArray, '/fake_nodes', self.update_fake, 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def callback_ar(self, msg):
        self.ar_nodes = msg.data
        self.compute()

    def callback_mr(self, msg):
        self.mr_nodes = msg.data
        self.compute()
    
    def callback_fake(self, msg):
        self.fake_nodes = msg.data
        self.compute()

    def compute(self):
        if self.ar_nodes is None or self.mr_nodes is None or self.fake_nodes is None:
            return    
        self.get_logger.info("Received full cv data .")

        self.apply_weights()

        dist, prev = self.dijkstra(self.start)
        best_goal = min(self.goals, key=lambda g: dist[g])
        path = self.build_path(prev, best_goal)

        msg = Int32MultiArray()
        msg.data = path
        self.publisher.publish(msg)

        self.get_logger().info(f"Published final path: {path}")

    def apply_weights(self):
        self.graoh - [[(nbr, w)for nbr, w in node]for node in self.base_graph]

        for node in range(len(self.graph)):
            new_edges = []
            for nbr, w in self.graph[node]:
                if nbr in self.graph[node]:
                    w = self.W_AR

                elif nbr in self.mr_nodes:
                    w = self.W_MR
                
                elif nbr in self.fake_nodes:
                    w = self.W_FAKE
                new_edges.append((nbr, w))
            self.graph[node] = new_edges
    

        
    def dijkstra(self, start):
        distances = {node: float('inf') for node in graph}
        distances[start] = 0
        predecessors = {node: None for node in graph}
        priority_queue = [(0, start)]
    
        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
                
            if current_distance > distances[current_node]:
                    continue
                
            for nbr, w in graph[current_node]:
                distance = current_distance + w
                if distance < distances[nbr]:
                    distances[nbr] = distance
                    predecessors[nbr] = current_node
                    heapq.heappush(priority_queue, (distance, nbr))
        
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


    
    
def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

