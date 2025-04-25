import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import tf_transformations
import sys


class TrafficFSM:
    MOVING  = "moving"  
    BRAKING = "braking" 
    STOPPED = "stopped"

class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Main Controller node has started")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.pub_turtle = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)
        self.sub_points = self.create_subscription(Point, "/next_point", self.callback_points, 10)
        self.sub_color_g = self.create_subscription(Bool, '/green', self.callback_green,10)
        self.sub_color_r = self.create_subscription(Bool, '/red', self.callback_red,10)
        self.sub_color_y = self.create_subscription(Bool, '/yellow', self.callback_yellow,10)


        # self.create_subscription(Pose, "/turtle1/pose", self.callback_odom_turtle, 1)
        self.create_subscription(Odometry, "/odom", self.callback_odom, 1)


        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)


        self.create_timer(0.01, self.control_loop)

        self.x = None
        self.y = None
        self.theta = None

        self.x_d = None
        self.y_d = None

        self.c_green = False
        self.c_red = False
        self.c_yellow = False

        self.state = TrafficFSM.MOVING

        self.arrtived = Bool()

    def callback_green(self, msg):
        self.c_green = msg.data

    def callback_red(self, msg):
        self.c_red = msg.data

    def callback_yellow(self, msg):
        self.c_yellow = msg.data



    # def callback_odom_turtle(self, msg):
    #     self.x = msg.x
    #     self.y = msg.y
    #     self.theta = msg.theta


    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.theta = tf_transformations.euler_from_quaternion(orientation_list)[2]

    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y

        #self.get_logger().info(f"Next point: {self.x_d}, {self.y_d}")

    def control_loop(self):

        if self.x is not None and self.y is not None and self.x_d is not None and self.y_d is not None:

            signal = self.current_signal()

            self.get_logger().info(f"Current signal: {signal}")

            if self.state == TrafficFSM.STOPPED:
                if signal == "green":
                    self.state = TrafficFSM.MOVING
            
            else:
                if signal == "red":
                    self.state = TrafficFSM.STOPPED
                elif signal == "yellow":
                    self.state = TrafficFSM.BRAKING
                    self.break_velocity()
                elif signal == "green":
                    self.state == TrafficFSM.MOVING

            if self.state == TrafficFSM.MOVING:
                self.advance(kv=0.2, kw=1.0)
            elif self.state == TrafficFSM.BRAKING:
                self.break_velocity()
            elif self.state == TrafficFSM.STOPPED:
                self.stop_velocity()


    def current_signal(self):
        if self.c_green:
            return "green"
        elif self.c_red:
            return "red"
        elif self.c_yellow:
            return "yellow"
        else:
            return None

    def stop_velocity(self):
        self.advance(kv=0.0, kw=0.0)

    def break_velocity(self):
        self.advance(kv=0.1, kw=0.0)

            
    def advance(self, kv=0.2, kw=1.0):

       
        msg = Twist()

        Dx = self.x_d - self.x
        Dy = self.y_d - self.y

        distance = math.sqrt(Dx**2 + Dy**2)

        angle_to_goal = math.atan2(Dy, Dx)
        angle_error = angle_to_goal - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        #self.get_logger().info(f"Distance to target: {distance}")

        if distance > 0.1:
            if abs(angle_error) > 0.1:  # Fase de rotación
                msg.linear.x = 0.0
                msg.angular.z = kw * angle_error
            else:  # Fase de avance
                msg.linear.x = kv
                msg.angular.z = kw * angle_error

            self.arrtived.data = False
            self.pub_arrived.publish(self.arrtived)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.arrtived.data = True
            self.pub_arrived.publish(self.arrtived)

        self.pub.publish(msg)
        self.pub_turtle.publish(msg)

            

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    