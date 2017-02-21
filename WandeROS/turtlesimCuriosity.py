################
################
################
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg import Vector3
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
################
################
################
class turtlebot():
	########	
	########
	def __init__(self):
		rospy.init_node('turtlesimCuriosity', anonymous=True)
		self.velocityPublisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
		self.pose = Pose()
		if not rospy.is_shutdown():
			self.goTo(5,5)
	########	
	########
	def callback(self, Pose):
		self.pose = Pose
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
	########	
	########
	def goTo(self, goalX, goalY):
		goalRate = rospy.Rate(20)
		while sqrt(pow((goalX - self.pose.x), 2) + pow((goalY - self.pose.y), 2)) > 0.01:
			linearVelocityX = 1.5 * sqrt(pow((goalX - self.pose.x), 2) + pow((goalY - self.pose.y), 2))
			angularVelocityZ = 4 * (atan2(goalY - self.pose.y, goalX - self.pose.x) - self.pose.theta)
			self.velocityPublisher.publish(Twist(Vector3(linearVelocityX,0,0),Vector3(0,0,angularVelocityZ)))
			goalRate.sleep()
		self.velocityPublisher.publish(Twist())
################
################
################
if __name__ == '__main__':
	try:
		turtlebot()
	except rospy.ROSInterruptException: pass
