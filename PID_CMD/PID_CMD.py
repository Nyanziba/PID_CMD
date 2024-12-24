import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Add this import

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Node Started")
        
        self.desired_x = 9.0  # Adjust as needed
        self.desired_y = 9.0  # Adjust as needed
        self.Kp_lv = 2.0
        self.Kp_av = 1.0
        # Publisher and Subscriber
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def odom_callback(self, msg):
        # Compute control commands
        self.compute_and_publish_velocity(msg)

    def compute_and_publish_velocity(self, odom):
        # Extract current position and orientation
        current_x = odom.pose.pose.position.x
        current_y = odom.pose.pose.position.y
        orientation_q = odom.pose.pose.orientation
        _, _, current_theta = self.euler_from_quaternion(orientation_q)

        # Compute the error in position
        error_x = self.desired_x - current_x
        error_y = self.desired_y - current_y

        # Compute the distance to the target
        distance = math.sqrt(error_x**2 + error_y**2)

        # Compute the angle to the target
        angle_to_target = math.atan2(error_y, error_x)

        # Compute the control commands
        linear_velocity = self.Kp_lv * distance  # Proportional control for linear velocity
        angular_velocity = self.Kp_av * (angle_to_target - current_theta)  # Proportional control for angular velocity

        # Create and publish the velocity command
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.vel_pub.publish(vel_msg)

    def euler_from_quaternion(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main():
    rclpy.init()
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
