import roslib;
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCoverianceStamped, Point, Quatertnion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class NavTest():
	def __init__(self):
		rospy.init_node('random_navigation', anonymous = True)
		rospy.on_shutdown(self.shutdown)

		self.rest_time = rospy.get_param("~rest_time", 2)

		goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
						'SUCCEEDED', 'ABORTED', 'PEJECTED',
						'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

		locations = dict()
		locations['p1'] = Pose(Point(1.150, 5.461, 0.000), Quatertnion(0.000, 0.000, -0.013, 1.000))
		locations['p2'] = Pose(Point(6.388, 2.66, 0.000), Quatertnion(0.000, 0.000, 0.063, 0.998))
		locations['p3'] = Pose(Point(8.089, -1.657, 0.000), Quatertnion(0.000, 0.000, 0.0946, -0.324))
		locations['p2'] = Pose(Point(9.767, 5.171, 0.000), Quatertnion(0.000, 0.000, 0.139, 0.990))
		locations['p2'] = Pose(Point(0.502, 1.270, 0.000), Quatertnion(0.000, 0.000, 0.919, -0.392))
		locations['p2'] = Pose(Point(4.557, 1.234, 0.000), Quatertnion(0.000, 0.000, 0.627, 0.779))

		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server ...")
			