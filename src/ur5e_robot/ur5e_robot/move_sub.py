import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_msgs.msg import String
import math

class TurtleGoToGoal(Node):
    def __init__(self):
        super().__init__("Turtle_GoToGoal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.coordinates_sub = self.create_subscription(Point, 'slot_coordinates', self.coordinates_callback, 10)
        self.subscription = self.create_subscription(String, 'expiry_status', self.expiry_status_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal) 
        self.pose = None
        self.coordinates = []
        self.current_coordinate_index = 0
        self.waiting_at_goal = False
        self.slot_expired = False

    def pose_callback(self, data):
        self.pose = data

    def coordinates_callback(self, data):
        self.coordinates.append((data.x, data.y))

    def expiry_status_callback(self, data):
        self.slot_expired = data.data == "expired"

    def go_to_goal(self):
        if not self.coordinates:
            return  

        goal_x, goal_y = self.coordinates[self.current_coordinate_index]

        dist_tol = 0.1
        angular_tol = 0.01
        new_vel = Twist()

        goal_angle = math.atan2(goal_y - self.pose.y , goal_x - self.pose.x )
        dist = math.sqrt((goal_x - self.pose.x) ** 2 + (goal_y - self.pose.y) ** 2)

        if abs(goal_angle - self.pose.theta) > angular_tol:
            new_vel.angular.z = (goal_angle - self.pose.theta)
        else:
            if dist >= dist_tol:
                new_vel.linear.x = dist
            else:
                new_vel.linear.x = 0.0
                if not self.waiting_at_goal:
                    self.wait_at_goal_position()

        self.cmd_vel_pub.publish(new_vel)

    def wait_at_goal_position(self):
        self.get_logger().info("Waiting 20 seconds ")
        self.waiting_at_goal = True
        self.timer = self.create_timer(1.0, self.check_expiry_and_resume)  # Check expiry status every second

    def check_expiry_and_resume(self):
        if self.slot_expired:
            self.get_logger().info("Box expired. Stopping.")
            self.timer.cancel()  # Cancel the timer to stop checking expiry
            # Additional actions when the box is expired (you can add more if needed)
            return
        else:
            self.get_logger().info("Box is not expired.")

        if self.timer.time_since_last_call() >= 20.0:  # Check if 20 seconds have passed
            self.timer.cancel()  # Cancel the timer if 20 seconds have passed
            self.get_logger().info("Resuming movement")
            self.waiting_at_goal = False
            self.current_coordinate_index += 1
            if self.current_coordinate_index >= len(self.coordinates):
                self.current_coordinate_index = 0

def main(args=None):
    rclpy.init(args=args)
    turtle_goto_location_node = TurtleGoToGoal()
    rclpy.spin(turtle_goto_location_node)
    turtle_goto_location_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
