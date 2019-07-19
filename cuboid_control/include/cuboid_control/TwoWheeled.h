/*
 * Copyright (c) 2019, SoftBank corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <vesc_msgs/VescStateStamped.h>

namespace cuboid_control
{

class TwoWheeled : public hardware_interface::RobotHW
{
public:
	//TwoWheeled(const ros::NodeHandle& nh=ros::NodeHandle());
	TwoWheeled(ros::NodeHandle nh);

	ros::Time getTime() const { return ros::Time::now(); }
	ros::Duration getPeriod() const { return ros::Duration(0.01); }

	void read(ros::Time, ros::Duration);
	void write(ros::Time, ros::Duration);

	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	double pos_[3], vel_[3], eff_[3], cmd_[3];

protected:
	int NUM_MOTOR_POLES;
	double duty_multiplier, duty_limiter, r_motor_direction, l_motor_direction, r_sensor_direction, l_sensor_direction;

	ros::NodeHandle root_nh_;
	ros::Publisher duty_l_pub_, duty_r_pub_;
	ros::Subscriber sensor_l_sub_, sensor_r_sub_;
	double tachometer_[2];

	void sensor_l_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state);
	void sensor_r_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state);
	int r_pid_control(double target_vel, double* duty_out, int init_flag);
	int l_pid_control(double target_vel, double* duty_out, int init_flag);
	int r_ff_control(double target_v, double* duty_out, int init_flag);
	int l_ff_control(double target_v, double* duty_out, int init_flag);
	double r_counter_td(long count_in, int init_flag);
	double l_counter_td(long count_in, int init_flag);
	double kp_import;
	double ki_import;
	double kd_import;
	ros::Publisher left_P_pub_, left_I_pub_, left_D_pub_, left_x_pub_, left_xd_pub_, left_xtarget_pub_, left_xtargetd_pub_, left_duty_pub_, left_dutylimit_pub_;
	ros::Publisher right_P_pub_, right_I_pub_, right_D_pub_, right_x_pub_, right_xd_pub_, right_xtarget_pub_, right_xtargetd_pub_, right_duty_pub_, right_dutylimit_pub_;
	ros::Publisher ctrl_freq_pub_;
	int vesc_ready[2] = {0,0};
	double ctrlFreq = 1.;
	double pos_tmp_[2]={0,0}; //エラー処理用に一回バッファに入れることにした
	double pos_tmp_prev_[2]={0,0};
	int16_t displacement_[2]={0,0};
	int16_t displacement_prev_[2]={0,0};
	int16_t displacement_diff_[2]={0,0};
};

} // namespace rrcar_control

