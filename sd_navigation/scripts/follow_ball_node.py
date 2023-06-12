# Copyright 2023 Josh Newans
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
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowBall():

    def __init__(self):
        rospy.set_param("rcv_timeout_secs", 1.0)
        rospy.set_param("angular_chase_multiplier", 0.7)
        rospy.set_param("forward_chase_speed", 0.1)
        rospy.set_param("search_angular_speed", 0.5)
        rospy.set_param("max_size_thresh", 0.1)
        rospy.set_param("filter_value", 0.9)

        self.rcv_timeout_secs = rospy.get_param('rcv_timeout_secs')
        self.angular_chase_multiplier = rospy.get_param('angular_chase_multiplier')
        self.forward_chase_speed = rospy.get_param('forward_chase_speed')
        self.search_angular_speed = rospy.get_param('search_angular_speed')
        self.max_size_thresh = rospy.get_param('max_size_thresh')
        self.filter_value = rospy.get_param('filter_value')

        timer_period = 0.1  # seconds
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

        self.ball_sub = rospy.Subscriber("/detect_ball/detected_ball", Point, self.listener_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


    def timer_callback(self, event=None):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            rospy.logdebug('Target: {}'.format(self.target_val))
            rospy.logdebug('Distance: {}'.format(self.target_dist))
            if (self.target_dist < self.max_size_thresh):
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier*self.target_val
        else:
            rospy.logdebug('Target lost')
            msg.angular.z = self.search_angular_speed
        self.cmd_vel_pub.publish(msg)


    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()


if __name__ == '__main__':
    rospy.init_node('follow_ball_node', anonymous=True)
    FollowBall()
    rospy.spin()