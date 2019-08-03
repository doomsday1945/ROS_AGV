import tf
import rospy
from math import copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point

class CalibrateLinear():
	def __init__(self):
		rospy.init_node('calibate_linear_node', anonymous=False)

		rospy.on_shutdown(self.shutdown)

		self.test_distance = rospy.get_param("~test_distance", 2.0)
		self.speed = rospy.get_param("~linear_speed", 0.17)
		self.tolerance = rospy.get_param("~tolerance_linear", 0.005)
		self.odom_linear_scale = rospy.get_param("~linear_scale", 1.000)
		self.rate = rospy.get_param("~check_rate", 15)
		check_rate = rospy.Rate(self.rate)
		self.start_test = True

		self.cmd_topic = rospy.get_param("~cmd_topic", '/cmd_vel')
		self.cmd_vel = rospy.Publisher(self.cmd_topic, Twist, queue_size = 5)

		self.base_frame =rospy.get_param('~base_frame', '/base_footprint')
		self.odom_frame = rospy.get_param('~odom_frame', '/odom')

		self.tf_listener = tf.TransformListener()
		rospy.sleep(2)

		self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(30.0))
		self.position = Point()

		self.position = self.get_position()
		self.x_start = self.position.x
		self.y_start = self.position.y

		self.print_summary()

		while not rospy.is_shutdown():
			self.position = self.get_position()
			check_rate.sleep()

			if self.start_test:
				distance = sqrt(pow((self.position.x - self.x_start), 2) +
								pow((self.position.y - self.y_start), 2))

				distance *= self.odom_linear_scale
				error = self.test_distance - distance
				rospy.loginfo("-->self_distance: " + str(error))

				move_cmd = Twist()
				if error < self.tolerance:
					self.start_test = False
					self.cmd_vel.publish(Twist())
					rospy.logwarn("Now stop move robot !")
				else:
					move_cmd.linear.x = self.speed
					self.cmd_vel.publish(move_cmd)
			else:
				actual_dist = input("Please input actual distance: ")
				linear_scale_error = float(actual_dist)/self.test_distance
				self.odom_linear_scale *= linear_scale_error
				rospy.logwarn("Now get linear_scale: " + str(self.odom_linear_scale))
				self.print_summary()
				self.start_test = True
				self.x_start = self.position.x
				self.y_start = self.position.y

	def print_summary(self):
		rospy.logwarn("~~~~~Now Start Linear Speed Caliration~~~~~")
		rospy.logwarn("-> test_distance: " + str(self.test_distance))
		rospy.logwarn("-> linear_speed: " + str(self.speed))
		rospy.logwarn("-> move_time: " + str(self.test_distance/self.speed))
		rospy.logwarn("-> cmd_topic: " + str(self.cmd_topic))
		rospy.logwarn("-> distance_tolerance: " + str(self.tolerance))
		rospy.logwarn("-> linear_scale: " + str(self.odom_linear_scale))

	def get_position(self):
		try:
			(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.logerr("lookup TF exception!")
		return Point(*trans)

	def shutdown(self):
		rospy.logwarn("shutdown test node, stopping the robot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		CalibrateLinear()
		rospy.spin()
	except:
		rospy.logerr("Caliration terminated ba unknown problems!")