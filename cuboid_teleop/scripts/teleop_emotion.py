#!/usr/bin/env python

# Copyright (c) 2019, SoftBank corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class TeleopEmotion(object):

    _CONTROL_HZ = 4

    def __init__(self):

        self.emo_anger_pressed_ = False
        self.emo_bright_pressed_ = False
        self.emo_sad_pressed_ = False
        self.emo_puzzle_pressed_ = False
        self.emergence_pressed_ = False

        rospy.init_node("teleop_face", anonymous=True)
        self._init_params()
        self._init_pubsub()

        rospy.Timer(rospy.Duration(1. / self._CONTROL_HZ), self._timer_callback)

    def _init_params(self):
        self.emo_anger_ = rospy.get_param("~emotion_anger", 12)
        self.emo_bright_ = rospy.get_param("~emotion_bright", 13)
        self.emo_sad_ = rospy.get_param("~emotion_sad", 14)
        self.emo_puzzle_ = rospy.get_param("~emotion_puzzle", 15)
        self.emergence_ = rospy.get_param("~emotion_puzzle", 3)

    def _init_pubsub(self):
        rospy.Subscriber("joy", Joy, self._update_joy)
        self.pub = rospy.Publisher("emotion", String, queue_size=10)

    def _update_joy(self, joy):
        self.emo_anger_pressed_ = joy.buttons[self.emo_anger_]
        self.emo_bright_pressed_ = joy.buttons[self.emo_bright_]
        self.emo_sad_pressed_ = joy.buttons[self.emo_sad_]
        self.emo_puzzle_pressed_ = joy.buttons[self.emo_puzzle_]
        self.emergence_pressed_ = joy.buttons[self.emergence_]

    def _timer_callback(self, msg):
        if self.emo_anger_pressed_:
            self.pub.publish("anger")
        elif self.emo_bright_pressed_:
            self.pub.publish("bright")
        elif self.emo_sad_pressed_:
            self.pub.publish("sad")
        elif self.emo_puzzle_pressed_:
            self.pub.publish("puzzle")
        elif self.emergence_pressed_:
            rospy.signal_shutdown("Shutdown signal button is pressed")
        else:
            self.pub.publish("hello")


if __name__ == "__main__":
    node = TeleopEmotion()

    rospy.spin()
