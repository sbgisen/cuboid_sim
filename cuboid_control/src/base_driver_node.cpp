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
#include <controller_manager/controller_manager.h>
#include <cuboid_control/TwoWheeled.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "base_driver");
	ros::NodeHandle nh;

	cuboid_control::TwoWheeled base(nh);
	controller_manager::ControllerManager cm(&base, nh);

	ros::Rate rate(1.0 / base.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(ros::ok())
	{
		ros::Time now = base.getTime();
		ros::Duration dt = base.getPeriod();

		base.read(now, dt);

		cm.update(now, dt);

		base.write(now, dt);
		rate.sleep();
	}
	spinner.stop();

	return 0;
}
