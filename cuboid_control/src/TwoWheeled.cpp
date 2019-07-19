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

#include <cuboid_control/TwoWheeled.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <iostream>

namespace cuboid_control
{

//TwoWheeled::TwoWheeled(const ros::NodeHandle& nh)
TwoWheeled::TwoWheeled(ros::NodeHandle nh)
	: pos_( {0}), vel_({0}), eff_({0}), cmd_({0}),
root_nh_(nh), tachometer_({0.0})
{
	// load parameters
	std::string l_wheel_drv_topic_name, r_wheel_drv_topic_name;
	std::string l_wheel_sensor_topic_name, r_wheel_sensor_topic_name;
	root_nh_.param<std::string>("/base/base_driver/l_wheel_drv_topic", l_wheel_drv_topic_name, "wheel_left/commands/motor/duty_cycle");
	root_nh_.param<std::string>("/base/base_driver/r_wheel_drv_topic", r_wheel_drv_topic_name, "wheel_right/commands/motor/duty_cycle");
	root_nh_.param<std::string>("/base/base_driver/l_wheel_sensor_topic", l_wheel_sensor_topic_name, "wheel_left/sensors/core");
	root_nh_.param<std::string>("/base/base_driver/r_wheel_sensor_topic", r_wheel_sensor_topic_name, "wheel_right/sensors/core");
	root_nh_.param("/base/base_driver/num_motor_poles", NUM_MOTOR_POLES, 258);
	//load pid gain
	root_nh_.param("/base/base_driver/Kp", kp_import, 0.04);
	root_nh_.param("/base/base_driver/Ki", ki_import, 0.004);
	root_nh_.param("/base/base_driver/Kd", kd_import, 0.1);

	root_nh_.param("/base/base_driver/r_motor_direction", r_motor_direction, 1.0);
	root_nh_.param("/base/base_driver/l_motor_direction", l_motor_direction, 1.0);
	if (r_motor_direction > 0.) {
		r_motor_direction = 1.;
	} else {
		r_motor_direction = -1.;
	}
	if (l_motor_direction > 0.) {
		l_motor_direction = 1.;
	} else {
		l_motor_direction = -1.;
	}
	root_nh_.param("/base/base_driver/r_sensor_direction", r_sensor_direction, 1.0);
	root_nh_.param("/base/base_driver/l_sensor_direction", l_sensor_direction, 1.0);
	if (r_sensor_direction > 0.) {
		r_sensor_direction = 1.;
	} else {
		r_sensor_direction = -1.;
	}
	if (l_sensor_direction > 0.) {
		l_sensor_direction = 1.;
	} else {
		l_sensor_direction = -1.;
	}


	root_nh_.param("/base/base_driver/duty_multiplier", duty_multiplier, 1.0);
	root_nh_.param("/base/base_driver/duty_limiter", duty_limiter, 1.0);
	//init pub/sub
	duty_l_pub_ = root_nh_.advertise<std_msgs::Float64>(l_wheel_drv_topic_name, 10);
	duty_r_pub_ = root_nh_.advertise<std_msgs::Float64>(r_wheel_drv_topic_name, 10);
	sensor_l_sub_ = root_nh_.subscribe(l_wheel_sensor_topic_name, 10, &TwoWheeled::sensor_l_Callback, this);
	sensor_r_sub_ = root_nh_.subscribe(r_wheel_sensor_topic_name, 10, &TwoWheeled::sensor_r_Callback, this);

	left_P_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/P", 10);
	left_I_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/I", 10);
	left_D_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/D", 10);
	left_x_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/angle/x", 10);
	left_xd_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/angle/xd", 10);
	left_xtarget_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/angle/xtarget", 10);
	left_xtargetd_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/angle/xtargetd", 10);
	left_duty_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/duty", 10);
	left_dutylimit_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_left/angle_controller/dutylimit", 10);
	ctrl_freq_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_controller/controller_frequency", 10);

	right_P_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/P", 10);
	right_I_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/I", 10);
	right_D_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/D", 10);
	right_x_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/angle/x", 10);
	right_xd_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/angle/xd", 10);
	right_xtarget_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/angle/xtarget", 10);
	right_xtargetd_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/angle/xtargetd", 10);
	right_duty_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/duty", 10);
	right_dutylimit_pub_ = root_nh_.advertise<std_msgs::Float64>("wheel_right/angle_controller/dutylimit", 10);

	// connect and register joint_state_insterfaces
	hardware_interface::JointStateHandle state_handle0("wheel_left_joint", &pos_[0], &vel_[0], &eff_[0]);
	jnt_state_interface.registerHandle(state_handle0);

	hardware_interface::JointStateHandle state_handle1("wheel_right_joint", &pos_[1], &vel_[1], &eff_[1]);
	jnt_state_interface.registerHandle(state_handle1);

	registerInterface(&jnt_state_interface);

	// connect and register joint_effort_interfaces
	hardware_interface::JointHandle vel_handle0(jnt_state_interface.getHandle("wheel_left_joint"), &cmd_[0]);
	jnt_vel_interface.registerHandle(vel_handle0);

	hardware_interface::JointHandle vel_handle1(jnt_state_interface.getHandle("wheel_right_joint"), &cmd_[1]);
	jnt_vel_interface.registerHandle(vel_handle1);

	registerInterface(&jnt_vel_interface);

	//PID initialization
	vesc_ready[0]  =  0;
	vesc_ready[1]  =  0;
	//Buffer Initialization
	pos_tmp_[0] = 0.0;
	pos_tmp_[1] = 0.0;
	pos_tmp_prev_[0] = 0.0;
	pos_tmp_prev_[1] = 0.0;
	displacement_prev_[0]=0;
	displacement_prev_[1]=0;
	displacement_diff_[0]=0;
	displacement_diff_[1]=0;
}

void TwoWheeled::read(ros::Time time, ros::Duration period)
{
	//直前の値を更新
	pos_tmp_prev_[0] = pos_tmp_[0];
	pos_tmp_prev_[1] = pos_tmp_[1];
	//今回の値を更新
	displacement_prev_[0] = displacement_[0];
	displacement_[0]=(int16_t)tachometer_[0];
	if(vesc_ready[0]==0){ //初期化
		displacement_prev_[0] = displacement_[0];
	}
	displacement_prev_[1] = displacement_[1];
	displacement_[1]=(int16_t)tachometer_[1];
	if(vesc_ready[1]==0){ //初期化
		displacement_prev_[1] = displacement_[1];
	}

	displacement_diff_[0] = displacement_[0] - displacement_prev_[0];
	displacement_diff_[1] = displacement_[1] - displacement_prev_[1];

	if(abs(displacement_diff_[0])>NUM_MOTOR_POLES/4){ //異常に大きく変化してた時は、変化を無視(異常と判断)
		displacement_diff_[0]=0;
		vesc_ready[0] = 0;
	}
	if(abs(displacement_diff_[1])>NUM_MOTOR_POLES/4){ //異常に大きく変化してた時は、変化を無視(異常と判断)
		displacement_diff_[1]=0;
		vesc_ready[1] = 0;
	}

	//現在角度の更新
	pos_tmp_[0] += l_sensor_direction * (double)displacement_diff_[0] / (double)NUM_MOTOR_POLES * 2.0 * M_PI;
	pos_tmp_[1] += r_sensor_direction * (double)displacement_diff_[1] / (double)NUM_MOTOR_POLES * 2.0 * M_PI;
	if(pos_tmp_[0] > M_PI){ //-πからπに正規化
		pos_tmp_[0] -= 2.0 * M_PI;
	}else if(pos_tmp_[0] < -M_PI){
		pos_tmp_[0] += 2.0 * M_PI;
	}
	if(pos_tmp_[1] > M_PI){ //-πからπに正規化
		pos_tmp_[1] -= 2.0 * M_PI;
	}else if(pos_tmp_[1] < -M_PI){
		pos_tmp_[1] += 2.0 * M_PI;
	}

	//前回の角度との差を算出
	double pos_tmp_diff[2];
	pos_tmp_diff[0]=pos_tmp_[0]-pos_tmp_prev_[0];
	pos_tmp_diff[1]=pos_tmp_[1]-pos_tmp_prev_[1];
	if(pos_tmp_diff[0] > M_PI){ //-πからπに正規化
		pos_tmp_diff[0] -= 2.0 * M_PI;
	}else if(pos_tmp_diff[0] < -M_PI){
		pos_tmp_diff[0] += 2.0 * M_PI;
	}
	if(pos_tmp_diff[1] > M_PI){ //-πからπに正規化
		pos_tmp_diff[1] -= 2.0 * M_PI;
	}else if(pos_tmp_diff[1] < -M_PI){
		pos_tmp_diff[1] += 2.0 * M_PI;
	}

	pos_[0] = pos_tmp_[0];
	pos_[1] = pos_tmp_[1];

	//エラー処理しない場合は以下の２行のみ
//	pos_[0] = l_sensor_direction * tachometer_[0] / (double)NUM_MOTOR_POLES * 2.0 * M_PI;
//	pos_[1] = r_sensor_direction * tachometer_[1] / (double)NUM_MOTOR_POLES * 2.0 * M_PI;

}

void TwoWheeled::write(ros::Time time, ros::Duration period)
{
	/*
		// command each motor
		std_msgs::Float64::Ptr cmd_l(new std_msgs::Float64);
		std_msgs::Float64::Ptr cmd_r(new std_msgs::Float64);

		std_msgs::Float64::Ptr duty_l_tmp(new std_msgs::Float64);
		std_msgs::Float64::Ptr duty_r_tmp(new std_msgs::Float64);

		double target_vel_in[2]={};
		target_vel_in[0]=cmd_[0];
		target_vel_in[1]=cmd_[1];
		double duty_out[2]={};
		pid_control(target_vel_in,duty_out,0);

		cmd_r -> data = duty_out[0];
		cmd_l -> data = 0.;//duty_out[1];
		duty_l_pub_.publish(cmd_l);
		duty_r_pub_.publish(cmd_r);
	*/
}

void TwoWheeled::sensor_l_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
	//read parameter
	root_nh_.getParam("/base/base_driver/duty_multiplier", duty_multiplier);
	root_nh_.getParam("/base/base_driver/duty_limiter", duty_limiter);
	//update ctrlFreq
	ctrlFreq = 50.0;//(1.0 / getPeriod().toSec()); //[Hz]

	tachometer_[0] = -state->state.displacement;
	//vesc_ready[0]=1;
	// command left motor
	std_msgs::Float64::Ptr cmd_l(new std_msgs::Float64);
	std_msgs::Float64::Ptr duty_l_tmp(new std_msgs::Float64);

	double target_vel_in = 0.0;
	target_vel_in = l_motor_direction * cmd_[0];


	double duty_out = 0.0;
	if (vesc_ready[0] == 0) {
		l_pid_control(target_vel_in, &duty_out, 1);
		l_ff_control(target_vel_in, &duty_out, 1);
		vesc_ready[0] = 1;
	} else {
		l_pid_control(target_vel_in, &duty_out, 0);
		//l_ff_control(target_vel_in,&duty_out,0);
	}
	cmd_l -> data = -duty_out;
	duty_l_pub_.publish(cmd_l);
}

int TwoWheeled::l_pid_control(double target_vel, double* duty_out, int init_flag) {
	std_msgs::Float64::Ptr msgs_tmp(new std_msgs::Float64);
	static double counter_ofs = 0; //for debug
	double wheel_Kp = kp_import;
	double wheel_Ki = ki_import;
	double wheel_Kd = kd_import;
	//ROS_INFO("Kp=%f,Ki=%f,Kd=%f\n",kp_import,ki_import,kd_import);
	static double p_tmp = 0.0, i_tmp = 0.0, i_prev = 0, d_tmp = 0.0;

	const double motorHallPpr = (double)NUM_MOTOR_POLES; //[PPR]
	const double motorDiameter = 0.1; //[m]

	const double i_duty_limit = 0.2;
	const double count_deviation_limit = (double)NUM_MOTOR_POLES;
	const double target_veloc_scaling_tmp = 1.0;//0.6

	double duty_limit = fabs(duty_limiter);
	if (duty_limit > 1.) {
		duty_limit = 1.;
	}

	static long pose_sens = 0; //HubMotor=60[ppr]//右回転が正
	pose_sens = -(long)tachometer_[0];

	static int init_flag_tmp = 1;
	if (init_flag != 0) {
		init_flag_tmp = 1;
	}

	//初期化
	static double pose_target = 0;
	if (init_flag_tmp == 1) {
		//ROS_INFO("\n\n\n initializing_l \n\n\n");
		counter_ofs = (double)pose_sens; //for debug
		pose_target = (double)pose_sens;
		i_tmp = 0.;
		p_tmp = 0.;
		init_flag_tmp = 0;
		d_tmp = 0;
		l_counter_td(pose_sens, 1);
		*duty_out = 0.0;
		//return 0;
	} else {
		pose_target += target_veloc_scaling_tmp * ((double) - target_vel * (double)motorHallPpr / (2 * 3.1416) / ctrlFreq);
	}

	if (pose_target > (double)LONG_MAX) {
		pose_target += (double)LONG_MIN;
	} else if (pose_target < (double)LONG_MIN) {
		pose_target += (double)LONG_MAX;
	}

	//PID制御
	//偏差リミット
	if ((long)pose_target - (long)pose_sens > (long)count_deviation_limit) {
		pose_target = (double)pose_sens + count_deviation_limit;
	} else if ((long)pose_target - (long)pose_sens < -(long)count_deviation_limit) {
		pose_target = (double)pose_sens - count_deviation_limit;
	}
	//左車輪

	double pose_target_diff = ((double)target_vel);//*(double)motorHallPpr/(2*3.1416)/(double)ctrlFreq);
	double sens_target_diff = -l_counter_td((long)pose_sens, 0) * 2 * 3.1416 / ((double)NUM_MOTOR_POLES) * ctrlFreq;

	d_tmp = -(pose_target_diff - sens_target_diff);
	p_tmp = (double)((long)pose_target - (long)pose_sens);
	i_prev = i_tmp;
	i_tmp += (p_tmp / ctrlFreq);
	double l_duty_out;
	l_duty_out = (wheel_Kp * p_tmp + wheel_Ki * i_tmp + wheel_Kd * d_tmp);

	if (l_duty_out > duty_limit) {
		l_duty_out = duty_limit;
		if (i_tmp > i_prev) { //anti reset wind up
			i_tmp = i_prev;
			l_duty_out = (wheel_Kp * p_tmp + wheel_Ki * i_tmp + wheel_Kd * d_tmp);
		}
	} else if (l_duty_out < -duty_limit) {
		l_duty_out = -duty_limit;
		if (i_tmp < i_prev) { //anti reset wind up
			i_tmp = i_prev;
			l_duty_out = (wheel_Kp * p_tmp + wheel_Ki * i_tmp + wheel_Kd * d_tmp);
		}
	}
	if (wheel_Ki * i_tmp > i_duty_limit) { //anti reset wind up
		i_tmp = i_duty_limit / wheel_Ki;
	} else if (wheel_Ki * i_tmp < -i_duty_limit) {
		i_tmp = -i_duty_limit / wheel_Ki;
	}


	if (init_flag_tmp == 0) {
		*duty_out = -duty_multiplier * l_duty_out;
	}
	if (*duty_out > 1.) {
		*duty_out = 1.;
	} else if (*duty_out < -1.) {
		*duty_out = -1.;
	}

	//停止時のトルクオフ↓
	static int duty_zero_counter = 0;
	static int wheel0_stop__flag = 0;
	static int wheel1_stop__flag = 0;
	if (cmd_[0] < 0.0001 && cmd_[0] > -0.0001) {
		wheel0_stop__flag = 1;
	} else {
		wheel0_stop__flag = 0;
	}
	if (cmd_[1] < 0.0001 && cmd_[1] > -0.0001) {
		wheel1_stop__flag = 1;
	} else {
		wheel1_stop__flag = 0;
	}
	if (wheel0_stop__flag == 1 && wheel1_stop__flag == 1) {
		if (duty_zero_counter < 10) {
			duty_zero_counter++;
		}
	} else {
		duty_zero_counter = 0;
	}
	if (duty_zero_counter == 10) {
		*duty_out = 0.0;
		vesc_ready[0] = 0;
		vesc_ready[1] = 0;
	}

	//停止時のトルクオフ↑
	//for debug
	msgs_tmp->data = p_tmp;
	left_P_pub_.publish(msgs_tmp);
	msgs_tmp->data = i_tmp;
	left_I_pub_.publish(msgs_tmp);
	msgs_tmp->data = d_tmp;
	left_D_pub_.publish(msgs_tmp);

	msgs_tmp->data = pose_sens - counter_ofs;
	left_x_pub_.publish(msgs_tmp);
	msgs_tmp->data = -sens_target_diff * 6.28 / 60 * (double)ctrlFreq;
	left_xd_pub_.publish(msgs_tmp);

	msgs_tmp->data = pose_target  - counter_ofs;
	left_xtarget_pub_.publish(msgs_tmp);
	msgs_tmp->data = target_vel;//pose_target_diff;
	left_xtargetd_pub_.publish(msgs_tmp);

	msgs_tmp->data = *duty_out;
	left_duty_pub_ .publish(msgs_tmp);

	msgs_tmp->data = duty_limit;
	left_dutylimit_pub_.publish(msgs_tmp);

	msgs_tmp->data =  ctrlFreq;
	ctrl_freq_pub_.publish(msgs_tmp);
	return 0;
}
double TwoWheeled::l_counter_td(long count_in, int init_flag) {
	static uint16_t counter_changed_log[10][2] = {}; //[カウント値,変化しなかった期間(サンプリングに変化がなかったカウント(最小で1))]
	static double counter_td_tmp[10] = {}; //移動平均を出力するため、10個分ログる
	static uint16_t counter_changed_single = 1; //データが変わっていない期間をカウント
	int i = 0;
	if (init_flag != 0) {
		counter_changed_single = 1;
		for (i = 0; i < 10; i++) {
			counter_changed_log[i][0] = (uint16_t)count_in;
			counter_changed_log[i][1] = 100;
			counter_td_tmp[i] = 0;
		}
		return 0.0;
	}
	if (counter_changed_log[0][0] != (uint16_t)count_in) {
		for (i = 1; i < 10; i++) {
			counter_changed_log[10 - i][0] = counter_changed_log[9 - i][0];
		}
		counter_changed_log[0][0] = (uint16_t)count_in;
		counter_changed_log[0][1] = counter_changed_single;
		counter_changed_single = 1;
	} else {

		if (counter_changed_single > counter_changed_log[0][1]) {
			counter_changed_log[0][1] = counter_changed_single;
		}
		if(counter_changed_single < 100){
			counter_changed_single++;
		}
	}

	for (i = 1; i < 10; i++) {
		counter_td_tmp[10 - i] = counter_td_tmp[9 - i];
	}
	counter_td_tmp[0] = (double)(counter_changed_log[0][0] - counter_changed_log[1][0] ) / (double)counter_changed_log[0][1];
	double output = 0.0;
	output = counter_td_tmp[0];
	if(fabs(output)>100.0){ //変化量が異常だった場合0にする(エラー処理)
		output = 0.0;
	}
	return output;
}
void TwoWheeled::sensor_r_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
	tachometer_[1] = state->state.displacement;
	//vesc_ready[1]=1;
	// command right motor
	std_msgs::Float64::Ptr cmd_r(new std_msgs::Float64);
	std_msgs::Float64::Ptr duty_r_tmp(new std_msgs::Float64);

	double target_vel_in = 0.0;
	target_vel_in = r_motor_direction * cmd_[1];

	double duty_out = 0.0;
	if (vesc_ready[1] == 0) {
		r_pid_control(target_vel_in, &duty_out, 1);
		r_ff_control(target_vel_in, &duty_out, 1);
		vesc_ready[1] = 1;
	} else {
		r_pid_control(target_vel_in, &duty_out, 0);
		//r_ff_control(target_vel_in,&duty_out,0);
	}
	cmd_r -> data = -duty_out;
	duty_r_pub_.publish(cmd_r);

}

int TwoWheeled::r_pid_control(double target_vel, double* duty_out, int init_flag) {
	static double counter_ofs = 0; //for debug
	std_msgs::Float64::Ptr msgs_tmp(new std_msgs::Float64);

	double wheel_Kp = kp_import;
	double wheel_Ki = ki_import;
	double wheel_Kd = kd_import;
	//ROS_INFO("Kp=%f,Ki=%f,Kd=%f\n",kp_import,ki_import,kd_import);
	static double p_tmp = 0.0, i_tmp = 0.0, i_prev = 0.0, d_tmp = 0.0;

	const double motorHallPpr = (double)NUM_MOTOR_POLES; //[PPR]
	const double motorDiameter = 0.1; //[m]

	double duty_limit = fabs(duty_limiter);
	if (duty_limit > 1.) {
		duty_limit = 1.;
	}
	const double i_duty_limit = 0.2;
	const double count_deviation_limit = 60.;
	const double target_veloc_scaling_tmp = 1.0;


	static long pose_sens = 0; //HubMotor=60[ppr]//右回転が正
	pose_sens = -(long)tachometer_[1];

	static int init_flag_tmp = 1;
	if (init_flag != 0) {
		init_flag_tmp = 1;
	}

	//初期化
	static double pose_target = 0;
	if (init_flag_tmp == 1) {
		//ROS_INFO("\n\n\n initializing_r \n\n\n");
		counter_ofs = (double)pose_sens; //for debug
		pose_target = (double)pose_sens;
		i_tmp = 0.;
		p_tmp = 0.;
		init_flag_tmp = 0;
		d_tmp = 0;
		r_counter_td(pose_sens, 1);
		*duty_out = 0.0;
		//return 0;
	} else {
		//pose_target += target_veloc_scaling_tmp*((double)-target_vel*(double)motorHallPpr/(2*3.1416)/(double)ctrlFreq);
		pose_target += target_veloc_scaling_tmp * ((double) - target_vel * (double)motorHallPpr / (2 * 3.1416) / ctrlFreq);
	}

	if (pose_target > (double)LONG_MAX) {
		pose_target += (double)LONG_MIN;
	} else if (pose_target < (double)LONG_MIN) {
		pose_target += (double)LONG_MAX;
	}

	//PID制御
	//偏差リミット
	if ((long)pose_target - (long)pose_sens > (long)count_deviation_limit) {
		pose_target = (double)pose_sens + count_deviation_limit;
	} else if ((long)pose_target - (long)pose_sens < -(long)count_deviation_limit) {
		pose_target = (double)pose_sens - count_deviation_limit;
	}
	//右車輪
	//d_tmp=((double)((long)pose_target-(long)pose_sens)-p_tmp)*ctrlFreq;
	//double pose_target_diff = ((double)-target_vel*(double)motorHallPpr/(2*3.1416)/(double)ctrlFreq);~
	//double sens_target_diff = r_counter_td((long)pose_sens,0)/(double)ctrlFreq;~
	double pose_target_diff = ((double)target_vel);//*(double)motorHallPpr/(2*3.1416)/(double)ctrlFreq);
	double sens_target_diff = -r_counter_td((long)pose_sens, 0) * 2 * 3.1416 / ((double)NUM_MOTOR_POLES) * ctrlFreq;
	d_tmp = -(pose_target_diff - sens_target_diff);
	p_tmp = (double)((long)pose_target - (long)pose_sens);
	i_prev = i_tmp;
	i_tmp += (p_tmp / ctrlFreq);
	double r_duty_out;
	r_duty_out = (wheel_Kp * p_tmp + wheel_Ki * i_tmp + wheel_Kd * d_tmp);

	if (r_duty_out > duty_limit) {
		r_duty_out = duty_limit;
		if (i_tmp > i_prev) { //anti reset wind up
			i_tmp = i_prev;
			r_duty_out = (wheel_Kp * p_tmp + wheel_Ki * i_tmp + wheel_Kd * d_tmp);
		}
	} else if (r_duty_out < -duty_limit) {
		r_duty_out = -duty_limit;
		if (i_tmp < i_prev) { //anti reset wind up
			i_tmp = i_prev;
			r_duty_out = (wheel_Kp * p_tmp + wheel_Ki * i_tmp + wheel_Kd * d_tmp);
		}
	}
	if (wheel_Ki * i_tmp > i_duty_limit) { //anti reset wind up
		i_tmp = i_duty_limit / wheel_Ki;
	} else if (wheel_Ki * i_tmp < -i_duty_limit) {
		i_tmp = -i_duty_limit / wheel_Ki;
	}
	if (init_flag_tmp == 0) {
		*duty_out = duty_multiplier * r_duty_out;
	}
	if (*duty_out > 1.) {
		*duty_out = 1.;
	} else if (*duty_out < -1.) {
		*duty_out = -1.;
	}

	//停止時のトルクオフ↓
	static int duty_zero_counter = 0;
	static int wheel0_stop__flag = 0;
	static int wheel1_stop__flag = 0;
	if (cmd_[0] < 0.0001 && cmd_[0] > -0.0001) {
		wheel0_stop__flag = 1;
	} else {
		wheel0_stop__flag = 0;
	}
	if (cmd_[1] < 0.0001 && cmd_[1] > -0.0001) {
		wheel1_stop__flag = 1;
	} else {
		wheel1_stop__flag = 0;
	}
	if (wheel0_stop__flag == 1 && wheel1_stop__flag == 1) {
		if (duty_zero_counter < 10) {
			duty_zero_counter++;
		}
	} else {
		duty_zero_counter = 0;
	}
	if (duty_zero_counter == 10) {
		*duty_out = 0.0;
		vesc_ready[0] = 0;
		vesc_ready[1] = 0;
	}

	//停止時のトルクオフ↑
	//for debug
	msgs_tmp->data = p_tmp;
	right_P_pub_.publish(msgs_tmp);
	msgs_tmp->data = i_tmp;
	right_I_pub_.publish(msgs_tmp);
	msgs_tmp->data = d_tmp;
	right_D_pub_.publish(msgs_tmp);

	msgs_tmp->data = pose_sens - counter_ofs;
	right_x_pub_.publish(msgs_tmp);
	msgs_tmp->data = -sens_target_diff * 6.28 / 258 * (double)ctrlFreq;
	right_xd_pub_.publish(msgs_tmp);

	msgs_tmp->data = pose_target  - counter_ofs;
	right_xtarget_pub_.publish(msgs_tmp);
	msgs_tmp->data = target_vel;//pose_target_diff;
	right_xtargetd_pub_.publish(msgs_tmp);

	msgs_tmp->data = *duty_out;
	right_duty_pub_ .publish(msgs_tmp);

	msgs_tmp->data = duty_limit;
	left_dutylimit_pub_.publish(msgs_tmp);

	return 0;
}

double TwoWheeled::r_counter_td(long count_in, int init_flag) {
	static uint16_t counter_changed_log[10][2] = {}; //[カウント値,変化しなかった期間(サンプリングに変化がなかったカウント(最小で1))]
	static double counter_td_tmp[10] = {}; //移動平均を出力するため、10個分ログる
	static uint16_t counter_changed_single = 1; //データが変わっていない期間をカウント
	int i = 0;
	if (init_flag != 0) {
		counter_changed_single = 1;
		for (i = 0; i < 10; i++) {
			counter_changed_log[i][0] = (uint16_t)count_in;
			counter_changed_log[i][1] = 100;
			counter_td_tmp[i] = 0;
		}
		return 0.0;
	}
	if (counter_changed_log[0][0] != (uint16_t)count_in) {
		for (i = 1; i < 10; i++) {
			counter_changed_log[10 - i][0] = counter_changed_log[9 - i][0];
		}
		counter_changed_log[0][0] = (uint16_t)count_in;
		counter_changed_log[0][1] = counter_changed_single;
		counter_changed_single = 1;
	} else {

		if (counter_changed_single > counter_changed_log[0][1]) {
			counter_changed_log[0][1] = counter_changed_single;
		}
		if(counter_changed_single < 100){
			counter_changed_single++;
		}
	}

	for (i = 1; i < 10; i++) {
		counter_td_tmp[10 - i] = counter_td_tmp[9 - i];
	}
	counter_td_tmp[0] = (double)(counter_changed_log[0][0] - counter_changed_log[1][0] ) / (double)counter_changed_log[0][1];
	double output = 0.0;
	/*
		for(i=0;i<10;i++){
			output += counter_td_tmp[i]*0.1;
		}
	*/
	output = counter_td_tmp[0];
	if(fabs(output)>100.0){ //変化量が異常だった場合0にする(エラー処理)
		output = 0.0;
	}
	return output;
}

int TwoWheeled::r_ff_control(double target_v, double* duty_out, int init_flag) {
	double a_v, a_vdt;
	a_v = 0.02;
	a_vdt = 0.001;
	static double target_v_prev = 0;
	double target_vdt = 0;

	if (init_flag == 1) {
		target_v_prev = 0;
		target_vdt = 0;
		*duty_out = 0.0;
		return 0.0;
	} else {
		target_vdt = (target_v - target_v_prev) * (double)ctrlFreq;
		target_v_prev = target_v;
		double ff_tmp = 0;
		ff_tmp = a_v * target_v + a_vdt * target_vdt;
		if (ff_tmp > 1.0) {
			ff_tmp = 1.0;
		} else if (ff_tmp < -1.0) {
			ff_tmp = -1.0;
		}
		*duty_out -= ff_tmp;
		if (*duty_out > 1.0) {
			*duty_out = 1.0;
		} else if (*duty_out < -1.0) {
			*duty_out = -1.0;
		}
	}
	return 0;
}
int TwoWheeled::l_ff_control(double target_v, double* duty_out, int init_flag) {
	double a_v, a_vdt;
	a_v = 0.02;
	a_vdt = 0.001;
	static double target_v_prev = 0;
	static double target_vdt = 0;

	if (init_flag == 1) {
		target_v_prev = 0;
		target_vdt = 0;
		*duty_out = 0.0;
		return 0.0;
	} else {
		target_vdt = (target_v - target_v_prev) * (double)ctrlFreq;
		target_v_prev = target_v;
		double ff_tmp = 0;
		ff_tmp = a_v * target_v + a_vdt * target_vdt;
		if (ff_tmp > 1.0) {
			ff_tmp = 1.0;
		} else if (ff_tmp < -1.0) {
			ff_tmp = -1.0;
		}
		*duty_out += ff_tmp;
		if (*duty_out > 1.0) {
			*duty_out = 1.0;
		} else if (*duty_out < -1.0) {
			*duty_out = -1.0;
		}

	}
	return 0;
}

} // namespace rrcar_control

