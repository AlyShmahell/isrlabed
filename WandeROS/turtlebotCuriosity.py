################
################
################
import copy
from math import *
import numpy as np
import tf
import tf2_ros
import rospy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
################
################
################
class WandeROS():
	########	
	########
	def __init__(self):
		###
		rospy.init_node('wanderos', anonymous=True)   
		rospy.on_shutdown(self.__shutdown__)
		###
		self.bumperResponse = [pi/2, pi, -pi/2]
		self.landmarks      = [
									[1.44,-0.98,  "cube"],
									[0.99,-3.44,  "dumpster"],
									[-2.00,-3.48, "cylinder"],
									[-4.00,-1.00, "barrier"],
									[0.00,-1.53,  "bookshelf"]
						      ]
		self.vicinity 	 	= [-1, "Undefined"]
		###
		self.odometry 	 	= Odometry()
		self.bumper_event 	= BumperEvent()
		###
		self.velocity_publisher = rospy.Publisher(
													"/cmd_vel_mux/input/teleop", 
												 	Twist, 
												 	queue_size=10
												)
		self.pose_subscriber    = rospy.Subscriber(
													"/odom", 
											   		Odometry, 
											   		self.__pose_callback
											    )
		self.bumper_subscriber  = rospy.Subscriber(
													"/mobile_base/events/bumper",
												 	BumperEvent, 
												 	self.__bumper_callback
												)
		###
		init_rate = rospy.Rate(20)
		init_rate.sleep()
		while not rospy.is_shutdown():
			self.think()
	########	
	########
	def __shutdown__(self):
		rospy.loginfo("Shutting Down...")
		self.velocity_publisher.publish(Twist())
		rospy.sleep(1)
	########	
	########
	def __bumper_callback(self, bumper_event):
		self.bumper_event = BumperEvent
	########	
	########
	def __pose_callback(self, odometry):
		self.odometry = odometry
		self.odometry.pose.pose.position.x = round(self.odometry.pose.pose.position.x, 4)
		self.odometry.pose.pose.position.y = round(self.odometry.pose.pose.position.y, 4)
		self.vicinity[0] = -1
		for i in range(len(self.landmarks)):
			distance = sqrt(
								pow((self.landmarks[i][0]-odometry.pose.pose.position.x), 2)
								+pow((self.landmarks[i][1]-odometry.pose.pose.position.y), 2)
						   )
			if (self.vicinity[0] == -1 or distance < self.vicinity[0]):
				self.vicinity[0] = distance
				self.vicinity[1] = self.landmarks[i][2]
		rospy.loginfo("current position data, X: %f, Y:%f. closest to: %s at %f" 
					  %(
						  self.odometry.pose.pose.position.x,
						  self.odometry.pose.pose.position.y,
						  self.vicinity[1],self.vicinity[0])
						)
	########	
	########
	def think(self):
		if sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)>10.0:
			self.__home(0,0)
		else:
			self.__move(1.0)
	########
	########
	def __home(self, goalX, goalY):
		self.__rotate_by(
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
		self.__move(
						sqrt(
								pow(goalX - self.odometry.pose.pose.position.x, 2) 
								+ pow(goalY - self.odometry.pose.pose.position.y, 2)
							 )
					)
	########	
	########
	def __move(self, distance):
		rospy.loginfo("__move function called, moving for: %f meters" %distance)
		move_rate = rospy.Rate(20)
		start     = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
		distance_moved = 0.0
		while True :
			if(self.bumper_event.state == BumperEvent.PRESSED):
				self.velocity_publisher.publish(
													Vector3(-0.4, 0, 0),
													Vector3(0, 0, 0)
											  )
				move_rate.sleep()
				self.__rotate_by(self.bumperResponse[self.bumper_event.bumper])
				move_rate.sleep()
				break
			self.velocity_publisher.publish(
												Vector3(0.4, 0, 0),
												Vector3(0, 0, 0)
										  )
			move_rate.sleep()
			end            = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
			distance_moved = distance_moved + abs(abs(float(end)) - abs(float(start)))
			start          = end
			if not (distance_moved < distance):
				break
		self.velocity_publisher.publish(Twist())
		move_rate.sleep()
	########	
	########
	def __rotate_by(self, radians):
		###
		if (radians>0):
			angular_velocity_z = -0.3
		else:
			angular_velocity_z = 0.3
		###
		while radians > 2*pi :
			radians-=2*pi
		while radians < 0:
			radians+=2*pi
		###
		rospy.loginfo("__rotate_by function called, rotating by: %f" %radians )
		rotate_rate = rospy.Rate(20)
		previous_yaw = tf.transformations.euler_from_quaternion(
																	[
																		self.odometry.pose.pose.orientation.x,
																		self.odometry.pose.pose.orientation.y,
																		self.odometry.pose.pose.orientation.z,
																		self.odometry.pose.pose.orientation.w
																	]
															  )[2]
		rotate_rate.sleep()
		angle_turned = 0.0
		while True:
			self.velocity_publisher.publish(
												Vector3(0, 0, 0),
												Vector3(0, 0, angular_velocity_z)
										   )
			rotate_rate.sleep()
			current_yaw = tf.transformations.euler_from_quaternion(
																		[
																			self.odometry.pose.pose.orientation.x,
																			self.odometry.pose.pose.orientation.y,
																			self.odometry.pose.pose.orientation.z,
																			self.odometry.pose.pose.orientation.w
																		]
																  )[2]
			rotate_rate.sleep()
			angle_turned = angle_turned + abs(abs(current_yaw) - abs(previous_yaw))
			previous_yaw = current_yaw
			if angle_turned > abs(radians):
				rospy.loginfo("final bearing is: %f" %current_yaw)
				break
		self.velocity_publisher.publish(Twist())
		rotate_rate.sleep()
	########	
	########
	def __rotate_to(self, bearing):
		rospy.loginfo("rotate to function called, bearing is: %f" %float(bearing))
		rotate_rate = rospy.Rate(20)
		roll, pitch, yaw = tf.transformations.euler_from_quaternion(
																		[
																			self.odometry.pose.pose.orientation.x,
																			self.odometry.pose.pose.orientation.y,
																			self.odometry.pose.pose.orientation.z,
																			self.odometry.pose.pose.orientation.w
																		]
																	)
		rotate_rate.sleep()
		while abs(yaw-bearing)>0.1:
			self.velocity_publisher.publish(
											Vector3(0, 0, 0),
											Vector3(0, 0, 0.3)
										  )
			rotate_rate.sleep()
			roll, pitch, yaw = tf.transformations.euler_from_quaternion(
																			[
																				self.odometry.pose.pose.orientation.x,
																				self.odometry.pose.pose.orientation.y,
																				self.odometry.pose.pose.orientation.z,
																				self.odometry.pose.pose.orientation.w
																			]
																		)
			rotate_rate.sleep()

		rospy.loginfo("final bearing is: %f" %yaw)
		self.velocity_publisher.publish(Twist())
		rotate_rate.sleep()
################
################
################
if __name__ == '__main__':
	try:
		WandeROS()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterruptException")
