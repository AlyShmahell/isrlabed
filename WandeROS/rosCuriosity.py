################
################
################
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
################
################
################
class rosCuriosity():
	########	
	########
	def __init__(self):
		###
		rospy.init_node('rosCuriosity', anonymous=True)   
		rospy.on_shutdown(self.__shutdown__)
		###
		self.bumperResponse = [90,180,-90]
		self.landmarks=[[1.44,-0.98,"cube"],[0.99,-3.44,"dumpster"],[-2.00,-3.48,"cylinder"],
				[-4.00,-1.00,"barrier"],[0.00,-1.53,"bookshelf"]]
		self.vacinity = [-1,"Undefined"]
		###
		self.odometry = Odometry()
		self.bumperEvent = BumperEvent()
		###
		self.velocityPublisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
		self.poseSubscriber = rospy.Subscriber("/odom", Odometry, self.poseCallback)
		self.bumperSubscriber = rospy.Subscriber("/mobile_base/events/bumper",BumperEvent, self.bumperCallback)
		###
		initRate = rospy.Rate(20)
		initRate.sleep()
		while not rospy.is_shutdown():
			self.think()
	########	
	########
	def __shutdown__(self):
		rospy.loginfo("Shutting Down")
		self.velocityPublisher.publish(Twist())
		rospy.sleep(1)
	########	
	########
	def bumperCallback(self,bumperEvent):
		self.bumperEvent = bumperEvent
	########	
	########
	def poseCallback(self,odometry):
		self.odometry = odometry
		self.vacinity[0]=-1
		for i in range(len(self.landmarks)):
			distance = sqrt(pow((self.landmarks[i][0]-odometry.pose.pose.position.x),2)+
					pow((self.landmarks[i][1]-odometry.pose.pose.position.y),2));
			if (self.vacinity[0]==-1 or distance<self.vacinity[0]):
				self.vacinity[0] = distance
				self.vacinity[1] = self.landmarks[i][2]
	########	
	########
	def think(self):
		thinkRate = rospy.Rate(20)
		if(sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)>10.0):
			deltaY = abs(self.odometry.pose.pose.position.y)
			deltaX = abs(self.odometry.pose.pose.position.x)
			bearing = 0.0
			
			if self.odometry.pose.pose.position.y > 0 and self.odometry.pose.pose.position.x > 0:
				bearing = -180 + atan2(deltaY, deltaX)*180/pi
			if self.odometry.pose.pose.position.y > 0 and self.odometry.pose.pose.position.x < 0:
				bearing = -1 * atan2(deltaY, deltaX)*180/pi
			if self.odometry.pose.pose.position.y < 0 and self.odometry.pose.pose.position.x < 0:
				bearing = atan2(deltaY, deltaX)*180/pi
			if self.odometry.pose.pose.position.y < 0 and self.odometry.pose.pose.position.x > 0:
				bearing = 180 - atan2(deltaY, deltaX)*180/pi
			rospy.loginfo("before rotate by, Y: %f, X: %f, Bearing: %f" %(self.odometry.pose.pose.position.y,
									self.odometry.pose.pose.position.x, bearing))
			self.rotateTo(bearing)
			self.move(sqrt(deltaX ** 2 + deltaY ** 2))
			thinkRate.sleep()
		else:
			self.move(1.0)
	########	
	########
	def move(self, distance):
		rospy.loginfo("move function called, moving by: %f" %distance)
		moveRate = rospy.Rate(20)
		start = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
		distanceMoved = 0.0
		while True :
			if(self.bumperEvent.state == BumperEvent.PRESSED):
				self.velocityPublisher.publish(Vector3(-0.4,0,0),Vector3(0,0,0))
				moveRate.sleep()
				self.rotateBy(self.bumperResponse[self.bumperEvent.bumper])
				moveRate.sleep()
			self.velocityPublisher.publish(Vector3(0.4,0,0),Vector3(0,0,0))
			moveRate.sleep()
			end = sqrt(self.odometry.pose.pose.position.x ** 2 + self.odometry.pose.pose.position.y ** 2)
			distanceMoved = distanceMoved+abs(abs(float(end)) - abs(float(start)))
			start = end
			rospy.loginfo("inside move function loop, moved by: %f" %distanceMoved)
			if not (distanceMoved<distance):
				break
		self.velocityPublisher.publish(Twist())
		moveRate.sleep()
	########	
	########
	def rotateBy(self, degrees):
		rospy.loginfo("rotate by function called")
		rotatRate = rospy.Rate(20)
		radians = degrees/57.2957795
		euler = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
								self.odometry.pose.pose.orientation.y,
								self.odometry.pose.pose.orientation.z,
								self.odometry.pose.pose.orientation.w])
		previousYaw = euler[2]
		rotatRate.sleep()
		angleTurned = 0.0
		while True:
			if (radians>0):
				self.velocityPublisher.publish(Vector3(0,0,0),Vector3(0,0,-0.3))
			else:
				self.velocityPublisher.publish(Vector3(0,0,0),Vector3(0,0,0.3))
			rotatRate.sleep()
			euler = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
									self.odometry.pose.pose.orientation.y,
									self.odometry.pose.pose.orientation.z,
									self.odometry.pose.pose.orientation.w])
			currentYaw = euler[2]
			angleTurned = angleTurned + abs(abs(currentYaw) - abs(previousYaw))
			previousYaw = currentYaw
			rotatRate.sleep()
			if (angleTurned > abs(radians)):
				break
		self.velocityPublisher.publish(Twist())
		rotatRate.sleep()
	########	
	########
	def rotateTo(self, bearing):
		rospy.loginfo("rotate to function called, bearing is: %f" %float(bearing/57.2957795))
		rotateRate = rospy.Rate(20)
		bearing = float(bearing/57.2957795)
		roll, pitch, yaw = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
										self.odometry.pose.pose.orientation.y,
										self.odometry.pose.pose.orientation.z,
										self.odometry.pose.pose.orientation.w])
		while abs(yaw-bearing)>0.1:
			self.velocityPublisher.publish(Vector3(0,0,0),Vector3(0,0,0.3))
			rotateRate.sleep()
			roll, pitch, yaw = tf.transformations.euler_from_quaternion([self.odometry.pose.pose.orientation.x,
											self.odometry.pose.pose.orientation.y,
											self.odometry.pose.pose.orientation.z,
											self.odometry.pose.pose.orientation.w])

		rospy.loginfo("final bearing: %f" %yaw)
		self.velocityPublisher.publish(Twist())
		rotateRate.sleep()
################
################
################
if __name__ == '__main__':
	try:
		rosCuriosity()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterruptException")
