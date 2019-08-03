#include <sstream>
#include <agv_diff_bringup/agv_diff_hw.h>

namespace agv
{
//读取的速度
int *eth_curr_vel_temp_;
//下发的速度
int eth_cmd_vel_temp_[4];
int count = 0;
int ENC_FULL_[4] = {33, 17400, 33, 17400};
const double ROBOT_RADIUS = 0.125;
const double ROBOT_LENGTH = 0.622;

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};

//限速
double AgvDiffHw::Vel_Limit(double v, double min, double max)
{
	if(v > max)
		v = max;
	if(v < min)
		v = min;
	return v;
}

void AgvDiffHw::readwrite_speed(double RobotV, double YawRate)
{
	ros::Time curr_time;

	agv_diff_msgs::AgvVelocity agv_vel;

	eth_curr_vel_temp_ = igh_update(eth_cmd_vel_temp_);
	for(size_t i = 0; i < 4; i++)
	{
		jnt_curr_vel_[i] = -(eth_curr_vel_temp_[i]) * (2*PI) * ROBOT_RADIUS / (ENC_FULL_[i] * 60);
	}

	agv_vel.VelRA = jnt_curr_vel_[2];
	agv_vel.VelRB = jnt_curr_vel_[0];
	agv_vel.VelethA = eth_curr_vel_temp_[2];
	agv_vel.VelethB = eth_curr_vel_temp_[0];
	agv_vel.VelCA = eth_cmd_vel_[2];
	agv_vel.VelCB = eth_cmd_vel_[0];	

	vel_pub.publish(agv_vel);
	
	vx_ = (jnt_curr_vel_[0] + jnt_curr_vel_[2]) / 2;
	vth_ = (jnt_curr_vel_[0] - jnt_curr_vel_[2]) / ROBOT_LENGTH;
	
	curr_time = ros::Time::now();
	double dt = (curr_time - last_time_).toSec();
	double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
	double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
	double delta_th = vth_ *dt;
	x_ += delta_x;
	y_ += delta_y;
    th_ += delta_th;
    last_time_ = curr_time;

	double r = RobotV / YawRate;
    // 计算左右轮期望速度
	if(RobotV == 0)
	{
		eth_cmd_vel_[2] = -YawRate * ROBOT_RADIUS;
		eth_cmd_vel_[0] = YawRate * ROBOT_RADIUS;
	} 
    else if(YawRate == 0)
	{
		eth_cmd_vel_[2] = RobotV;
		eth_cmd_vel_[0] = RobotV;
	}
	else
	{
		eth_cmd_vel_[2]  = YawRate * (r - ROBOT_RADIUS);
		eth_cmd_vel_[0] = YawRate * (r + ROBOT_RADIUS);
	}
	for(size_t i = 0; i < 4; i++)
	{
		eth_cmd_vel_temp_[i] = -(Vel_Limit(eth_cmd_vel_[i], -0.5, 0.5)) * (60 * ENC_FULL_[i]) / (2*PI*ROBOT_RADIUS);
	}
	eth_curr_vel_temp_ = igh_update(eth_cmd_vel_temp_);

}

AgvDiffHw::AgvDiffHw()
{
	igh_configure();
	jnt_curr_vel_.clear();
//	Initialize joint value
	jnt_curr_vel_.resize(4);
	eth_cmd_vel_.clear();
	eth_cmd_vel_.resize(4);

	eth_curr_vel_temp_ = igh_get_curr_vel();
    pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
	vel_pub = nh.advertise<agv_diff_msgs::AgvVelocity>("agv_vel_real", 50);		
}

bool AgvDiffHw::update(double RobotV, double YawRate)
{
//	Handle current position (for reading)
	readwrite_speed(RobotV, YawRate);

    current_time_ = ros::Time::now();
    // 发布TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_footprint";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = current_time_;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x_;
    msgl.pose.pose.position.y = y_;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx_;
    msgl.twist.twist.linear.y = vy_;
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
  
    pub_.publish(msgl);
}


ros::Time AgvDiffHw::getTime() const 
{
    return ros::Time::now();
}

ros::Duration AgvDiffHw::getPeriod() const 
{
    return ros::Duration(0.001);
}
AgvDiffHw::~AgvDiffHw()
{
	igh_stop();
	igh_cleanup();
}
}

