#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from my_rb1_ros.srv import Rotate, RotateResponse
import math

class RotateService:

    def __init__(self):
        rospy.init_node('rotate_service_server')

        # Create service
        my_service = rospy.Service('/rotate_robot', Rotate, self.rotate_robot_callback)

        # Create publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create subscription to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize variables
        self.current_yaw = 0.0
        self.odom_received = False

        rospy.loginfo('Service Ready')

    def odom_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def rotate_robot_callback(self, request):
        rospy.loginfo('Service Requested')

        response = RotateResponse()

        # Wait for odometry data
        if not self.odom_received:
            rospy.logwarn('Waiting for odometry data...')
            timeout = rospy.Time.now() + rospy.Duration(10.0)
            while not self.odom_received and rospy.Time.now() < timeout:
                rospy.sleep(0.1)

            if not self.odom_received:
                response.result = "Failed: No odometry data received"
                return response

        try:
            # Convert degrees to radians
            target_rotation = math.radians(request.degrees)

            # Get initial yaw
            initial_yaw = self.current_yaw

            # Create twist message
            twist = Twist()

            # Set angular velocity based on rotation direction - increased for faster rotation
            base_angular_velocity = 0.8 if target_rotation > 0 else -0.8  # Faster base speed for 3-5 second target

            # Continue rotating until target is reached
            tolerance = 0.008727  # 0.5 degrees tolerance - high precision
            rate = rospy.Rate(50)  # Higher frequency for better control and drift reduction

            while not rospy.is_shutdown():
                # Calculate current rotation from initial position
                angle_diff = self.normalize_angle(self.current_yaw - initial_yaw)
                remaining_angle = self.normalize_angle(target_rotation - angle_diff)

                if abs(remaining_angle) < tolerance:
                    break

                # Create twist message with adaptive speed
                twist = Twist()

                # Adaptive speed control based on remaining angle - balanced speed and precision
                if abs(remaining_angle) > 0.4:  # > ~23 degrees - use full speed
                    twist.angular.z = base_angular_velocity
                elif abs(remaining_angle) > 0.15:  # > ~9 degrees - moderate slowdown
                    twist.angular.z = base_angular_velocity * 0.75
                elif abs(remaining_angle) > 0.05:  # > ~3 degrees - careful approach
                    twist.angular.z = base_angular_velocity * 0.4
                elif abs(remaining_angle) > 0.02:  # > ~1 degree - precision mode
                    twist.angular.z = base_angular_velocity * 0.2
                else:  # Final fine adjustment
                    twist.angular.z = base_angular_velocity * 0.12

                # Publish velocity command
                self.cmd_vel_pub.publish(twist)
                rate.sleep()

            # Stop the robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

            rospy.loginfo('Service Completed')
            response.result = "Rotation completed successfully"

        except Exception as e:
            rospy.logerr('Service failed: %s' % str(e))
            response.result = "Failed: %s" % str(e)

        return response

def main():
    try:
        rotate_service = RotateService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()