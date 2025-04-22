import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import tf_transformations
import sys

class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Main Controller node has started")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.pub_turtle = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)
        self.sub_points = self.create_subscription(Point, "/next_point", self.callback_points, 10) 

        self.create_subscription(Odometry, "/odom", self.callback_odom, 1)


        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)


        self.create_timer(0.01, self.control_loop)

        self.x = None
        self.y = None
        self.theta = None

        self.x_d = None
        self.y_d = None

        self.arrtived = Bool()


    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.theta = tf_transformations.euler_from_quaternion(orientation_list)[2]

    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y

        self.get_logger().info(f"Next point: {self.x_d}, {self.y_d}")

    def control_loop(self):
        kv = 0.2
        kw = 1.0

        if self.x is not None and self.y is not None and self.x_d is not None and self.y_d is not None:
            msg = Twist()

            Dx = self.x_d - self.x
            Dy = self.y_d - self.y

            distance = math.sqrt(Dx**2 + Dy**2)

            angle_to_goal = math.atan2(Dy, Dx)
            angle_error = angle_to_goal - self.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            #self.get_logger().info(f"Distance to target: {distance}")

            if distance > 0.1:
                if abs(angle_error) > 0.08:  # Fase de rotación
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
    