import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Add this import
from turtlesim.msg import Pose

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Node Started")
        
        self.desired_x = 11.0  # Adjust as needed
        self.desired_y = 1.0  # Adjust as needed
        self.desired_theta = 0.0
        self.Kp_lv = 0.4
        self.Kp_av = 4.0
        self.Kd_lv = 0.2
        self.Kd_av = 0.3
        # Time variables
        self.current_time = None
        self.last_time = self.get_clock().now()
        self.last_distance = None

        # Publisher and Subscriber
        self.my_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)


        # desired position timer
        self.timer = self.create_timer(0.1, self.update_desired_position)   
        self.time_elapsed = 0.0


    def pose_callback(self, msg):
        # Compute control commands
        self.current_time = self.get_clock().now()
        self.get_logger().info(f'Current time: {self.current_time.to_msg()}')
        self.compute_and_publish_velocity(msg)

    def update_desired_position(self):
        self.time_elapsed += 0.1
        # Example trajectory: circular path
        self.desired_x = 5.0 + 3.0 * math.cos(self.time_elapsed)
        self.desired_y = 5.0 + 3.0 * math.sin(self.time_elapsed)
        self.get_logger().info(f'Updated desired position: ({self.desired_x}, {self.desired_y})')
        
        
        

    def compute_and_publish_velocity(self, Pose):
        # Extract current position and orientation
        current_x = Pose.x
        current_y = Pose.y
        current_theta = Pose.theta
        if current_theta < 0:
            current_theta += 2 * math.pi
        print(f'Current position: ({current_x}, {current_y},{current_theta}])')
        # Compute the error in position
        error_x = self.desired_x - current_x
        error_y = self.desired_y - current_y
        
        # Compute the distance to the target
        distance = math.sqrt(error_x**2 + error_y**2)
        # Compute the time interval
        time_interval = (self.current_time.to_msg().sec + self.current_time.to_msg().nanosec * 1e-9) - (self.last_time.to_msg().sec + self.last_time.to_msg().nanosec * 1e-9)
        # Compute the differential of the distance
        if self.last_distance is not None:
            diff_distance =  (distance - self.last_distance) / time_interval
        else:
            diff_distance = 0
        #record last distance and time
        self.last_distance = distance
        self.last_time = self.get_clock().now()
        

        # Compute the angle to the target
        angle_to_target = math.atan2(error_y, error_x)
        if angle_to_target < 0:
            angle_to_target += 2 * math.pi
        print(f'Angle to target: {angle_to_target}')
        angular_error = angle_to_target - current_theta
        if angular_error > math.pi:
            angular_error -= 2 * math.pi
        elif angular_error < -math.pi:
            angular_error += 2 * math.pi

        # Compute the control commands
        linear_velocity = self.Kp_lv * distance - self.Kd_lv * diff_distance # Proportional control for linear velocity
        angular_velocity = self.Kp_av * angular_error - self.Kd_av * angular_error  # Proportional control for angular velocity

        # Create and publish the velocity command
        vel_msg = Twist()
        
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
