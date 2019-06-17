#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped
from collections import deque

# constant value
time_interval = 0.1  # [sec]
imu_callback_hz = 50.0  # [Hz]
stop_counter_max = int(time_interval*imu_callback_hz)
# variable
stop_counter = 0
target_veloc_x = 0.0  # Target linear velocity
target_veloc_w = 0.0  # Target angular velocity
# counter=[0,0]
# counter_prev=[0,0]
motor_speed = [0.0, 0.0]
gyro_offset = 0.0
gyro_buffer_num = 50
gyro_raw_buffer = [0.0]*gyro_buffer_num  # zero array with 10 members
gyro_1st_order_delay = 0.0
gyro_2nd_order_delay = 0
gyro_buffer_counter = 0
gyro_raw_prev = 0
gyro_offset_lpf = 0
gyro_offset_log = [0 for i in range(100)]


def callback_cmd(data):
    global target_veloc_x
    global target_veloc_w
    target_veloc_x = data.linear.x
    target_veloc_w = data.angular.z


def callback_vesc_r(data):
    #global counter
    #counter[0] = data.state.current_motor
    global motor_speed
    motor_speed[0] = data.state.speed


def callback_vesc_l(data):
    #global counter
    #counter[1] = data.state.current_motor
    global motor_speed
    motor_speed[1] = data.state.speed


def callback_imu(data):
    global time_interval
    global imu_callback_hz
    #global stop_counter_max
    #global stop_counter
    global target_veloc_x
    global target_veloc_w
    #global counter
    #global counter_prev
    global motor_speed
    global gyro_offset
    global gyro_buffer_num
    global gyro_raw_buffer
    global gyro_1st_order_delay
    global gyro_2nd_order_delay
    global gyro_buffer_counter
    global gyro_raw_prev
    global gyro_offset_lpf

    pub_data = data

    # low pass filter
    gyro_1st_order_delay = 0.99 * gyro_1st_order_delay + 0.01 * data.angular_velocity.z
    gyro_2nd_order_delay = 0.99 * gyro_2nd_order_delay + 0.01 * gyro_1st_order_delay
    gyro_diff_maximum = 0.0
    for i in range(gyro_buffer_num - 1):
        gyro_raw_buffer[gyro_buffer_num - i - 1] = gyro_raw_buffer[gyro_buffer_num - i - 2]
    gyro_raw_buffer[0] = data.angular_velocity.z

#  for i in range(gyro_buffer_num - 1):
#    if abs(gyro_raw_buffer[i+1]-gyro_raw_buffer[i])>gyro_diff_maximum:
#      gyro_diff_maximum=abs(gyro_raw_buffer[i+1]-gyro_raw_buffer[i])

#  for i in range(gyro_buffer_num):
#    for j in range(gyro_buffer_num):
#      if abs(gyro_raw_buffer[i]-gyro_raw_buffer[j])>gyro_diff_maximum:
#        gyro_diff_maximum=abs(gyro_raw_buffer[i]-gyro_raw_buffer[j])

    for i in range(gyro_buffer_num-1):
        for j in range(i+1, gyro_buffer_num):
            if abs(gyro_raw_buffer[i]-gyro_raw_buffer[j]) > gyro_diff_maximum:
                gyro_diff_maximum = abs(gyro_raw_buffer[i]-gyro_raw_buffer[j])

    # if abs(target_veloc_x) < 0.0001 and abs(target_veloc_w) < 0.0001 and abs(motor_speed[0]) < 100.0 and abs(motor_speed[1]) < 100.0 \
    if abs(target_veloc_x) < 0.0001 and abs(target_veloc_w) < 0.0001 and abs(motor_speed[0]) < 0.0001 and abs(motor_speed[1]) < 0.0001 \
            and abs(gyro_1st_order_delay - data.angular_velocity.z) < 0.05 and abs(gyro_2nd_order_delay - data.angular_velocity.z) < 0.05 \
            and gyro_diff_maximum < 0.05:  # and abs(data.angular_velocity.z - gyro_offset) < 0.05 :
        gyro_offset_lpf = 0.995*gyro_offset_lpf + (1.0-0.995)*data.angular_velocity.z
        gyro_offset = gyro_offset_lpf
    elif gyro_diff_maximum > 0.05:
        gyro_offset = gyro_offset_log[49]
        for i in range(49):
            gyro_offset_log[i] = gyro_offset_log[49]
#  else:
#    gyro_offset = gyro_offset_log[49]
    # gyro_offset_log[49]=1.0
    pub_data.angular_velocity.x = 0.0
    pub_data.angular_velocity.y = 0.0
    pub_data.angular_velocity.z = data.angular_velocity.z - gyro_offset_log[49]  # gyro_offset
    for i in range(49):
        gyro_offset_log[49 - i] = gyro_offset_log[49-i-1]
    gyro_offset_log[0] = gyro_offset
    pub.publish(pub_data)
#  for i in range(49):
#    gyro_offset_log[i+1]=gyro_offset_log[i]
#  gyro_offset_log[0]=gyro_2nd_order_delay
#  print('sub_gyro:{0}, gyro_offset:{1}, pub_gyro:{2}, stop:{3}, buf:{4}, speed1:{5}, speed2:{6}'.format(data.angular_velocity.z,gyro_offset,pub_data.angular_velocity.z,stop_counter,gyro_buffer_counter,motor_speed[0],motor_speed[1]))

    target_veloc_x = target_veloc_x*0.99
    target_veloc_w = target_veloc_w*0.99


rospy.init_node('imu_filter')
in_topic = rospy.get_param('~in', '/gyro/imu')
out_topic = rospy.get_param('~out', '/gyro/imu_filtered')
cmd_topic = rospy.get_param('~cmd', '/cmd_vel')
vesc_l_topic = rospy.get_param('~vesc_l', '/sensors/core')
vesc_r_topic = rospy.get_param('~vesc_r', '/sensors/core')

#print in_topic, out_topic

pub = rospy.Publisher(out_topic, Imu, queue_size=10)
rospy.Subscriber(in_topic, Imu, callback_imu)
rospy.Subscriber(cmd_topic, Twist, callback_cmd)
rospy.Subscriber(vesc_l_topic, VescStateStamped, callback_vesc_l)
rospy.Subscriber(vesc_r_topic, VescStateStamped, callback_vesc_r)

last_time = rospy.Time.now()

rospy.spin()
