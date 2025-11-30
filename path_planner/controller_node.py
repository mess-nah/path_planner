import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray 
import math



class Controller(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.dijkstra_sub = self.create_subscription(
             Int32MultiArray, '/planned_path',self.path_callback, 10)
        
        self.imu_sub = self.create_subscription(
             Float32MultiArray, '/imu_data_received', self.imu_callback, 10)
        
        self.distance_sub = self.create_subscription(
             Float32MultiArray, '/distance_data_received', self.distance_callback, 10)
        
        self.prox_sub = self.create_subscription(
             Float32MultiArray, '/proximity_data_received', self.proximity_callback, 10)
        
        self.deadwheel_sub = self.create_subscription(
             Float32MultiArray, '/deadwheel_count_received',self.deadwheel_callback, 10)

        self.climb_pub = self.create_publisher(String, '/climb_cmd', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_path = []
        self.index = 0

        self.current_yaw = 0.0
        self.distance_data = None
        self.proximity_data = None
        self.deadwheel_data = None

        self.node_height = {
            1: 40, 2: 20, 3: 40, 4: 60,
            5: 40, 6: 20, 7: 40, 8: 60,
            9: 40, 10: 20, 11: 40, 12: 20
        }


        self.timer = self.create_timer(0,1, self.control_loop)
        
    
    def path_callback(self, msg):
            self.current_path = list(msg.data)
            self.index = 0
            self.get_logger().info(f"received path: {self.current_path}")

    def imu_callback(self, imu_msg):
         self.current_yaw = imu_msg.data[0]
        
    def distance_callback(self, msg):
         if len(msg.data) > 0:
              self.latest_distance = msg.data[0]
              self.get_logger.info(f"Distance = {self.latest_distance}")

    def proximity_callback(self, msg):
         self.proximity_data = msg.data

    def deadwheel_callback(self, msg):
        self.deadwheel_data = sg.data

    def control_loop(self, node):

        
        if len(self.current_path) == 0:
            return
        
        if self.index >= len(self.current_path) - 1:
            self.get_logger().info("Reached final goal")
            return
        
        current_node = self.current_path[self.index]
        next_node = self.current_path[self.index +1]

        diff = next_node - current_node 

        #TURN_ANGLE_DEG = {
        #-1:  +90,
        #+1:  -90,
        #+3:   0,
        #}

        #turn_angle = TURN_ANGLE_DEG.get(diff, 0)

        if diff == -1:
             turn_angle = +90
        elif diff == +1:
             turn_angle = -90
        elif diff == 3:
             turn_angle = 0
        else:
             turn_angle = 0

        turn_angle_rad = math.radians(turn_angle)


    
        #x1, y1 = node_positions[current_node]
        #x2, y2 = node_positions[next_node]

        
        #dx = x2 - x1
        #dy = y2 - y1
        #desired_heading = math.atan2(dy, dx)

        
        #turn_angle= desired_heading - self.current_yaw

        
        #turn_angle = math.atan2(math.sin(turn_angle), math.cos(turn_angle))



        self.get_logger().info(f"Required turn angle: {math.degrees(turn_angle):.2f} degrees")

        
        TARGET_DISTANCE = 550 
        

        if self.latest_distance is None:
             speed = 0.0
        else:
             d = self.latest_distance

             if d > TARGET_DISTANCE:
                  speed = 0.20
             else:
                  speed = 0.0
                  self.get_logger().info("Reached block, checking height")   


        height_curr = self.node_height[current_node]
        height_next = self.node_height[next_node]
        height_diff = height_next - height_curr

        if speed == 0.0:
            if height_diff == 20:

                msg = String()
                msg.data = "UP"
                self.climb_pub.publish(msg)
                self.get_logger().info("Climbing +20mm step!")
                return  

            elif height_diff == -20:
                msg = String()
                msg.data = "DOWN"
                self.climb_pub.publish(msg)
                self.get_logger().info("Descending -20mm step!")
                return  # wait until done

            else:
             self.get_logger().info("No step change. continuing")
    
            



        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = turn_angle_rad

        self.cmd_pub.publish(cmd)





def main(args=None):
    rclpy.init(args=args)
    node= Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()