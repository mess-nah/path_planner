import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


class Controller(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.subscription = self.create_subscriber (Int32MultiArray, '/planned_path',self.path_callback, 10)

    def path_callback(self, msg):

        self.current_path = list(msg.data)
        self.index = 0
        
        self.get_logger().info(f"received path: {self.current_path}")


    def control_loop(self, node):

        if not self.current_path:
            return

        if self.index >= len(self.current_path):
            return

        node_point = self.current_path[self.index]

        if node_point == 1:
            command.data = TURN_RIGHT

        elif node_point == 3:
            command.data = TURN_LEFT

        elif node_point == 5:
            command.data = GO_AHEAD
            
        
        





def main(args=None):
    rclpy.init(args=args)
    node= Controller()
    rclpy.spin(node)
    node.destroy_npde()
    rclpy.shutdown()

if __name__=="__main__":
    main()