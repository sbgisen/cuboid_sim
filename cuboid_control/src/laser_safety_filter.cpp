#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>

//double safety_distance_, robot_width_, max_posi_lin_acc_;
//double safety_scaling_;
double max_posi_lin_acc_ , max_posi_lin_brk_ ;

//ros::Publisher pub_cmd_, pub_d_;
ros::Publisher pub_cmd_;
//ros::Subscriber sub_scan_, sub_cmd_;
ros::Subscriber sub_cmd_;

ros::Time last_time_;
double last_vx_ , last_yaw_ ;

/*
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	double angle = scan->angle_min;
	double min_dist = 200;
	for (size_t i = 0, l = scan->ranges.size(); i<l;++i, angle += scan->angle_increment){
		if(angle < -1.5 || angle > 1.5) continue;
		if(scan->ranges[i] < scan->range_min || scan->range_max < scan->ranges[i]) continue;
		double w = std::fabs(sin(angle) * scan->ranges[i]);
		double d = cos(angle) * scan->ranges[i];
		if(w < robot_width_ / 2.0 && d < min_dist){
			min_dist = d;
		}
	}
	safety_scaling_ = std::min(1.0, std::max(0.1, min_dist / safety_distance_));
	//std::cout << min_dist << " : " << safety_scaling_ << std::endl;
	std_msgs::Float64 dbg;
	dbg.data = min_dist;
	pub_d_.publish(dbg);
}
*/

void cmdCallback(const geometry_msgs::TwistConstPtr& cmd)
{
	ros::Time now = ros::Time::now();

	geometry_msgs::Twist scmd;
	double cmd_x = 0.0;
	//if(std::fabs(cmd->linear.x) > 0.001){
	double cmd_ax = cmd->linear.x - last_vx_;
	if (cmd_ax > 0.) {
		if (cmd_ax > max_posi_lin_acc_) {
			cmd_x = last_vx_ + max_posi_lin_acc_;
		} else {
			cmd_x = cmd->linear.x;
		}
	} else {
		if (cmd_ax < -max_posi_lin_brk_) {
			cmd_x = last_vx_ - max_posi_lin_brk_;
		} else {
			cmd_x = cmd->linear.x;
		}
	}
	scmd.linear.x = cmd_x;
	//}

	if (std::fabs(cmd->angular.z) > 0.001) {
		//double lin_scale = std::fabs(scmd.linear.x / cmd_x );
		//scmd.angular.z = lin_scale * cmd->angular.z;

		if (std::fabs(cmd_x) > 0.001) {
			double lin_scale = std::fabs(scmd.linear.x / cmd_x );
			scmd.angular.z = lin_scale * cmd->angular.z;
		} else {
			scmd.angular.z = cmd->angular.z;
		}
	}
	pub_cmd_.publish(scmd);

	last_vx_ = cmd_x;
	last_time_ = now;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "laser_safety_filter");
	ros::NodeHandle nh("~");

	//safety_scaling_ = 1.0;
	last_vx_ = 0;
	last_yaw_ = 0.0;
	last_time_ = ros::Time::now();

	//nh.param<double>("safety_dist", safety_distance_, 0.5);
	//nh.param<double>("robot_width", robot_width_, 0.4);
	nh.param<double>("max_posi_lin_acc", max_posi_lin_acc_, 1.0);
	nh.param<double>("max_posi_lin_brk", max_posi_lin_brk_, 1.0);
	/*
	if(safety_distance_ > 0.0){
		sub_scan_ = nh.subscribe("/scan", 1, scanCallback);
	}
	*/
	sub_cmd_ = nh.subscribe("/cmd_vel_raw", 1, cmdCallback);
	pub_cmd_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	//pub_d_ = nh.advertise<std_msgs::Float64>("/dist_front_object", 10);
	ros::spin();
	return 0;
}
