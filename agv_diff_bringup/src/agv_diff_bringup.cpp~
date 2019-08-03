#include <agv_diff_bringup/agv_diff_hw.h>

double RobotV_ = 0;
double YawRate_ = 0;

void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV_ = msg.linear.x;
	YawRate_ = msg.angular.z;
}

int main(int argc, char **argv)
{
  // initialize ros
	ros::init(argc, argv, "agv_diff_bringup");
	ros::NodeHandle nh;

	agv::AgvDiffHw robot;

//	start loop
	ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);

	ros::Rate rate(1.0 / robot.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();
	while (ros::ok())
	{
//		robot.read();
		if(loop_flag)
		{
			robot.update(RobotV_, YawRate_);
		}
//		robot.write();
		rate.sleep();
	}
	spinner.stop();
	return 0;
}
