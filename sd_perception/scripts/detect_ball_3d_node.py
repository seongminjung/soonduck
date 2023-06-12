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
from geometry_msgs.msg      import Point
from visualization_msgs.msg import Marker
import math

class DetectBall3d():

    def __init__(self):
        rospy.loginfo('3D Ball Detecting initialized')

        self.ball2d_sub = rospy.Subscriber("/detect_ball/detected_ball", Point, self.ball_rcv_callback)
        self.ball3d_pub = rospy.Publisher("/detect_ball/detected_ball_3d", Point, queue_size=1)
        self.ball_marker_pub = rospy.Publisher("/detect_ball/ball_3d_marker", Marker, queue_size=1)

        rospy.set_param("h_fov",1.089)
        rospy.set_param("ball_radius",0.033)
        rospy.set_param("aspect_ratio",4.0/3.0)
        rospy.set_param("camera_frame",'front_cam_link')

        self.h_fov = rospy.get_param('h_fov')
        self.v_fov = self.h_fov/rospy.get_param('aspect_ratio')
        self.ball_radius = rospy.get_param('ball_radius')
        self.camera_frame = rospy.get_param('camera_frame')


    def ball_rcv_callback(self, data:Point):

        # Calculate angular size and consequently distance
        ang_size = data.z*self.h_fov
        d = self.ball_radius/(math.atan(ang_size/2))

        # Calculate angular and distance deviations in X and Y
        y_ang = data.y*self.v_fov/2
        y = d*math.sin(y_ang)
        d_proj = d*math.cos(y_ang)

        x_ang = data.x*self.h_fov/2
        x = d_proj*math.sin(x_ang)
        z = d_proj*math.cos(x_ang)


        p = Point()
        p.x = x
        p.y = y
        p.z = z
        self.ball3d_pub.publish(p)


        m = Marker()
        m.header.frame_id = self.camera_frame

        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.x = self.ball_radius*2
        m.scale.y = self.ball_radius*2
        m.scale.z = self.ball_radius*2
        m.color.r = 0.933
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        self.ball_marker_pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('detect_ball_3d_node', anonymous=True)
    DetectBall3d()
    rospy.spin()

