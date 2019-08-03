#ifndef AGV_DIFF_HW_H
#define AGV_DIFF_HW_H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <agv_diff_msgs/AgvVelocity.h>
#define NDOF NSLAVE
#define PI 3.1415926

extern "C" {
	#include "agv_diff_bringup/agv_ethercat.h"
}

namespace agv
{
class AgvDiffHw
{
public:
	AgvDiffHw();
	~AgvDiffHw();
	bool update(double RobotV, double YawRate);
	double Vel_Limit(double, double, double);
	ros::Time getTime() const;
	ros::Duration getPeriod() const;
	ros::Publisher vel_pub;
	ros::Publisher pub_;


private:
	void readwrite_speed(double RobotV, double YawRate);
//	Raw Data
    ros::NodeHandle nh;
	double x_, y_, th_;
	double vx_, vy_, vth_;
    tf::TransformBroadcaster odom_broadcaster_;
	ros::Time current_time_, last_time_;
	std::vector<double> jnt_curr_vel_;
	std::vector<double> eth_cmd_vel_;

};

}

#endif 
