#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__("battery_monitor")

        #initialize
        self.x = None
        self.y = None
        self.x_center = 5.544445
        self.y_center = 5.544445
        self.distance_traveled = 0.0
        self.battery = 100.0
        self.drain_per_meter = 2.0
        self.returning_home = False
        self.at_home = True
        self.Kp = 2.0

        #subscribe to /turtle1/pose
        self.create_subscription(Pose,'turtle1/pose',self.callback_pose,10)
        #publish to /turtle1/cmd vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel',10)


        self.get_logger().info("Battery Monitor has been started.")

    def callback_pose(self, msg: Pose):
        # first pose: just initialize
        if self.x is None:
            self.x = msg.x
            self.y = msg.y
            return

        #c alculate Euclidean distance
        dx = msg.x - self.x
        dy = msg.y - self.y
        d = math.sqrt(dx*dx + dy*dy)

        # track total distance and update battery drain
        self.distance_traveled += d
        self.battery = max(0.0, 100.0 - self.drain_per_meter * self.distance_traveled)

        # minimum battery needed
        self.distance_to_center = math.sqrt(pow(self.x_center - msg.x, 2) + pow(self.y_center - msg.y, 2))
        self.min_battery_needed = self.distance_to_center * 2

        # if at home
        if self.distance_to_center < 0.1:
            self.at_home = True
            self.get_logger().info(f"AT HOME")
        else:
            self.at_home = False


        # if battery low
        if self.battery <= self.min_battery_needed:
            self.returning_home = True

        # IF NEED TO RETURN HOME    
        if self.returning_home:
            #self.get_logger().info(f"RETURNING HOME")
            
            # NAVIGATE HOME
            self.angle_to_center = math.atan2(self.y_center - msg.y, self.x_center - msg.x)
            self.heading_error = self.angle_to_center - msg.theta
            self.w = self.Kp * (self.heading_error)

            twist = Twist()
            twist.linear.x = 1.0          
            twist.angular.z = self.w
            self.cmd_vel_pub.publish(twist)

            if abs(self.heading_error) <= 0.05:
                self.get_logger().info(f"DRIVING HOME")

        if self.at_home:
            #stop moving
            twist = Twist()
            twist.linear.x = 0.0          
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.returning_home = False

            # reset
            self.battery = 100.0
            self.distance_traveled = 0.0
            return




        # pose update
        self.x = msg.x
        self.y = msg.y

        # log data
        self.get_logger().info(
            f"Distance: {self.distance_traveled:.2f} | Battery: {self.battery:.1f}% ")

        



def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()