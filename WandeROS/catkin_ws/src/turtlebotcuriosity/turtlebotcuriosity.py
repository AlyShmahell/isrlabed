#!/usr/bin/env python
import copy
from math import *
import numpy
import tf
import tf2_ros
import rospy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3



class turtlebotCuriosity():
	def __init__(self):
		
		rospy.init_node('turtlebotCuriosity', anonymous=True)   
		rospy.on_shutdown(self.__shutdown__)
		
		self.bumperResponse = [pi/2,pi,-pi/2]
		self.landmarks=[[1.44,-0.98,"cube"],[0.99,-3.44,"dumpster"],[-2.00,-3.48,"cylinder"],
				[-4.00,-1.00,"barrier"],[0.00,-1.53,"bookshelf"]]
		self.vacinity = [-1,"Undefined"]
		
		self.odometry = Odometry()
		self.bumperEvent = BumperEvent()
		
		self.velocityPublisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		self.poseSubscriber = rospy.Subscriber("/odom", Odometry, self.poseCallback)
		self.bumperSubscriber = rospy.Subscriber("/mobile_base/events/bumper",BumperEvent, self.bumperCallback)
		
		initRate = rospy.Rate(20)
		initRate.sleep()
		while not rospy.is_shutdown():
			self.think()
		
	
	def __shutdown__(self):
		rospy.loginfo("Shutting Down")
		self.velocityPublisher.publish(Twist())
		rospy.sleep(1)
		
	
	def bumperCallback(self,bumperEvent):
		self.bumperEvent = bumperEvent
		
	
	def poseCallback(self,odometry):
		self.odometry = odometry
		self.odometry.pose.pose.position.x = round(self.odometry.pose.pose.position.x,4)
		self.odometry.pose.pose.position.y = round(self.odometry.pose.pose.position.y,4)
		self.vacinity[0]=-1
		for i in range(len(self.landmarks)):
			distance = sqrt(pow((self.landmarks[i][0]-odometry.pose.pose.position.x),2)+
					pow((self.landmarks[i][1]-odometry.pose.pose.position.y),2));
			if (self.vacinity[0]==-1 or distance<self.vacinity[0]):
				self.vacinity[0] = distance
				self.vacinity[1] = self.landmarks[i][2]
		rospy.loginfo("current position data, X: %f, Y:%f. closest to: %s at %f" 
				%(self.odometry.pose.pose.position.x,self.odometry.pose.pose.position.y,self.vacinity[1],self.vacinity[0]))
		
	
	def think(self):
		if(sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)>10.0):
			self.home(0,0)
		else:
			self.move(1.0)
	
	
	def home(self, goalX, goalY):
		self.rotateBy(atan2(goalY - self.odometry.pose.pose.position.y, goalX - self.odometry.pose.pose.position.x) - 
				tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
									self.odometry.pose.pose.orientation.y,
									self.odometry.pose.pose.orientation.z,
									self.odometry.pose.pose.orientation.w])[2])
		self.move(sqrt(pow(goalX - self.odometry.pose.pose.position.x, 2) + pow(goalY - self.odometry.pose.pose.position.y, 2)))
		
	
	def move(self, distance):
		rospy.loginfo("move function called, moving for: %f meters" %distance)
		moveRate = rospy.Rate(20)
		start = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
		distanceMoved = 0.0
		while True :
			if(self.bumperEvent.state == BumperEvent.PRESSED):
				self.velocityPublisher.publish(Vector3(-0.2,0,0),Vector3(0,0,0))
				moveRate.sleep()
				self.rotateBy(self.bumperResponse[self.bumperEvent.bumper])
				moveRate.sleep()
				break
			self.velocityPublisher.publish(Vector3(0.2,0,0),Vector3(0,0,0))
			moveRate.sleep()
			end = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
			distanceMoved = distanceMoved+abs(abs(float(end)) - abs(float(start)))
			start = end
			if not (distanceMoved<distance):
				break
		self.velocityPublisher.publish(Twist())
		moveRate.sleep()
		
	
	def rotateBy(self, radians):
		
		if (radians>0):
			angularVelocityZ = -0.3
		else:
			angularVelocityZ = 0.3
		
		while radians > 2*pi :
			radians-=2*pi
		while radians < 0:
			radians+=2*pi
		
		rospy.loginfo("rotateby function called, rotating by: %f" %radians )
		rotatRate = rospy.Rate(20)
		previousYaw = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
								self.odometry.pose.pose.orientation.y,
								self.odometry.pose.pose.orientation.z,
								self.odometry.pose.pose.orientation.w])[2]
		rotatRate.sleep()
		angleTurned = 0.0
		while True:
			self.velocityPublisher.publish(Vector3(0,0,0),Vector3(0,0,angularVelocityZ))
			rotatRate.sleep()
			currentYaw = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
									self.odometry.pose.pose.orientation.y,
									self.odometry.pose.pose.orientation.z,
									self.odometry.pose.pose.orientation.w])[2]
			rotatRate.sleep()
			angleTurned = angleTurned + abs(abs(currentYaw) - abs(previousYaw))
			previousYaw = currentYaw
			if (angleTurned > abs(radians)):
				rospy.loginfo("final bearing is: %f" %currentYaw)
				break
		self.velocityPublisher.publish(Twist())
		rotatRate.sleep()
		
	
	def rotateTo(self, bearing):
		rospy.loginfo("rotate to function called, bearing is: %f" %float(bearing))
		rotateRate = rospy.Rate(20)
		roll, pitch, yaw = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
										self.odometry.pose.pose.orientation.y,
										self.odometry.pose.pose.orientation.z,
										self.odometry.pose.pose.orientation.w])
		rotateRate.sleep()
		while abs(yaw-bearing)>0.1:
			self.velocityPublisher.publish(Vector3(0,0,0),Vector3(0,0,0.3))
			rotateRate.sleep()
			roll, pitch, yaw = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
											self.odometry.pose.pose.orientation.y,
											self.odometry.pose.pose.orientation.z,
											self.odometry.pose.pose.orientation.w])
			rotateRate.sleep()

		rospy.loginfo("final bearing is: %f" %yaw)
		self.velocityPublisher.publish(Twist())
		rotateRate.sleep()



if __name__ == '__main__':
	try:
		turtlebotCuriosity()
		rospy.spin()	
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterruptException")
