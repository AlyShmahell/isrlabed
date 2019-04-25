######################################
#               futures
######################################
from __future__ import print_function
######################################
#           standard library
######################################
import os
import re
import random
from math import *
######################################
#           transform library
######################################
import tf
import tf2_ros
######################################
#                 ROS
######################################
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Pretty:

    def __new__(self, x):
        return re.sub(r"\n\s*", "\n", x)


class Oneliner:

    def __new__(self, x):
        return re.sub(r"\n\s*", " ", x)
            

class WandeROS():

    def __init__(self):
        self.radians            = [pi/2, pi, -pi/2]
        rospy.init_node('wanderos', anonymous=True)   
        rospy.on_shutdown(self.shutdown)
        self.timer              = rospy.Rate(20)
        self.odometry              = Odometry()
        self.laserscan             = LaserScan()
        self.velocity_publisher = rospy.Publisher(
                                                    "/cmd_vel", 
                                                     Twist, 
                                                     queue_size=10
                                                )
        self.timer.sleep()
        rospy.Subscriber(
                                                    "/odom", 
                                                       Odometry, 
                                                       self.__pose_callback
                                                )
        self.timer.sleep()
        rospy.Subscriber(
                                                    "/scan",
                                                     LaserScan, 
                                                     self.__bumper_callback
                                                )
        self.timer.sleep()

    def shutdown(self):
        self.velocity_publisher.publish(Twist())
        self.timer.sleep()

    def __bumper_callback(self, laserscan):
        self.laserscan = laserscan

    def __pose_callback(self, odometry):
        self.odometry = odometry
        rospy.loginfo(
                        "Coordinate:: X: %f, Y: %f"
                        %(
                            self.odometry.pose.pose.position.x,
                            self.odometry.pose.pose.position.y
                         )
                     )

    def __call__(self):
        try:
            while not rospy.is_shutdown():
                self.think()
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROSInterruptException")
            os.system("killall gzclient -9")
            os.system("killall gzserver -9")
        

    def think(self):
        if sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2) > 10.0:
            self.home(0,0)
        else:
            self.move(1.0)

    def home(self, goalX, goalY):
        self.rotate_by(
                            atan2(
                                    goalY - self.odometry.pose.pose.position.y, 
                                    goalX - self.odometry.pose.pose.position.x
                                 ) 
                            - tf.transformations.euler_from_quaternion(
                                                                            [
                                                                                self.odometry.pose.pose.orientation.x,
                                                                                self.odometry.pose.pose.orientation.y,
                                                                                self.odometry.pose.pose.orientation.z,
                                                                                self.odometry.pose.pose.orientation.w
                                                                            ]
                                                                      )[2]
                         )
        self.move(
                        sqrt(
                                pow(goalX - self.odometry.pose.pose.position.x, 2) 
                                + pow(goalY - self.odometry.pose.pose.position.y, 2)
                             )
                    )

    def move(self, distance):
        rospy.loginfo("move function called, moving forward %f meters" %distance)
        start     = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
        distance_moved = 0.0
        while True :
            if(len(self.laserscan.ranges) <= 1):
                continue
            if(self.laserscan.ranges[0] <= 0.5):
                self.velocity_publisher.publish(
                                                    Vector3(-0.4, 0, 0),
                                                    Vector3(0, 0, 0)
                                              )
                self.timer.sleep()
                self.rotate_by(random.choice(self.radians))
                self.timer.sleep()
                break
            self.velocity_publisher.publish(
                                                Vector3(0.4, 0, 0),
                                                Vector3(0, 0, 0)
                                           )
            self.timer.sleep()
            end            = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
            distance_moved = distance_moved + abs(abs(float(end)) - abs(float(start)))
            start          = end
            if not (distance_moved < distance):
                break
        self.velocity_publisher.publish(Twist())
        self.timer.sleep()

    def rotate_by(self, radians):
        if (radians>0):
            angular_velocity_z = -0.3
        else:
            angular_velocity_z = 0.3
        while radians > 2*pi :
            radians -= 2*pi
        while radians < 0:
            radians += 2*pi
        rospy.loginfo("rotate_by function called, rotating by: %f" %radians)
        previous_yaw = tf.transformations.euler_from_quaternion(
                                                                    [
                                                                        self.odometry.pose.pose.orientation.x,
                                                                        self.odometry.pose.pose.orientation.y,
                                                                        self.odometry.pose.pose.orientation.z,
                                                                        self.odometry.pose.pose.orientation.w
                                                                    ]
                                                               )[2]
        self.timer.sleep()
        angle_turned = 0.0
        while True:
            self.velocity_publisher.publish(
                                                Vector3(0, 0, 0),
                                                Vector3(0, 0, angular_velocity_z)
                                           )
            self.timer.sleep()
            current_yaw = tf.transformations.euler_from_quaternion(
                                                                        [
                                                                            self.odometry.pose.pose.orientation.x,
                                                                            self.odometry.pose.pose.orientation.y,
                                                                            self.odometry.pose.pose.orientation.z,
                                                                            self.odometry.pose.pose.orientation.w
                                                                        ]
                                                                  )[2]
            self.timer.sleep()
            angle_turned = angle_turned + abs(abs(current_yaw) - abs(previous_yaw))
            previous_yaw = current_yaw
            if angle_turned > abs(radians):
                rospy.loginfo("final bearing is: %f" %current_yaw)
                break
        self.velocity_publisher.publish(Twist())
        self.timer.sleep()

    def rotate_to(self, bearing):
        rospy.loginfo("rotate_to function called, bearing is: %f" %bearing)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                                                                        [
                                                                            self.odometry.pose.pose.orientation.x,
                                                                            self.odometry.pose.pose.orientation.y,
                                                                            self.odometry.pose.pose.orientation.z,
                                                                            self.odometry.pose.pose.orientation.w
                                                                        ]
                                                                    )
        self.timer.sleep()
        while abs(yaw-bearing)>0.1:
            self.velocity_publisher.publish(
                                            Vector3(0, 0, 0),
                                            Vector3(0, 0, 0.3)
                                          )
            self.timer.sleep()
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                                                                            [
                                                                                self.odometry.pose.pose.orientation.x,
                                                                                self.odometry.pose.pose.orientation.y,
                                                                                self.odometry.pose.pose.orientation.z,
                                                                                self.odometry.pose.pose.orientation.w
                                                                            ]
                                                                        )
            self.timer.sleep()
        rospy.loginfo("final bearing is: %f" %yaw)
        self.velocity_publisher.publish(Twist())
        self.timer.sleep()

if __name__ == '__main__':
    wanderos = WandeROS()
    wanderos()