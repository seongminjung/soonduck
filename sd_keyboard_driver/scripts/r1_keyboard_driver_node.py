import rospy
from time import time

from typing import Optional, Union

from pynput import keyboard
from pynput.keyboard import Key, KeyCode
from geometry_msgs.msg import Twist


# key bindings
R1_FORWARD = "w"
R1_BACKWARD = "s"
R1_LEFT = "a"
R1_RIGHT = "d"
R2_FORWARD = "up"
R2_BACKWARD = "down"
R2_LEFT = "left"
R2_RIGHT = "right"

HELP_MSG = """
Use keyboard to control SoonDuck

Key Bindings:
   [w]
[a][s][d]

[w] = Forward
[s] = Backward
[a] = Left
[d] = Right

[b] = Toggle ball tracking

[p] = Show this help"""

LINEAR_ACCEL = 1.3
LINEAR_DECEL = 1.8
ANGULAR_ACCEL = 2.5
ANGULAR_DECEL = 3.0

class KeyboardDriverNode():
    def __init__(self):
        self.r1_cmd_vel_publisher = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.r2_cmd_vel_publisher = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
        rospy.loginfo(HELP_MSG)

        self.r1_des_vel = Twist()
        self.r1_prev_msg = Twist()
        self.r2_des_vel = Twist()
        self.r2_prev_msg = Twist()

        self.status = {
            "r1_forward": 0,
            "r1_backward": 0,
            "r1_left": 0,
            "r1_right": 0,
            "r2_forward": 0,
            "r2_backward": 0,
            "r2_left": 0,
            "r2_right": 0,
        }
        rospy.Timer(rospy.Duration(0.01), self.pub_twist)
        self.prev_time = time() - 0.01
        self.cur_time = time()

    def on_press(self, key: Optional[Union[Key, KeyCode]]):
        try:
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == R1_FORWARD:
                self.status["r1_forward"] = 1
            if key == R1_BACKWARD:
                self.status["r1_backward"] = 1
            if key == R1_LEFT:
                self.status["r1_left"] = 1
            if key == R1_RIGHT:
                self.status["r1_right"] = 1

            if key == R2_FORWARD:
                self.status["r2_forward"] = 1
            if key == R2_BACKWARD:
                self.status["r2_backward"] = 1
            if key == R2_LEFT:
                self.status["r2_left"] = 1
            if key == R2_RIGHT:
                self.status["r2_right"] = 1

            self.r1_des_vel.linear.x = self.status["r1_forward"] - self.status["r1_backward"]
            self.r1_des_vel.angular.z = self.status["r1_left"] - self.status["r1_right"]
            self.r2_des_vel.linear.x = self.status["r2_forward"] - self.status["r2_backward"]
            self.r2_des_vel.angular.z = self.status["r2_left"] - self.status["r2_right"]

            # Make sure this listener node is destroyed when the node is shutdown
            if rospy.is_shutdown():
                return False

        except rospy.ROSInterruptException:
            pass

    def on_release(self, key: Optional[Union[Key, KeyCode]]):
        try:
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == R1_FORWARD:
                self.status["r1_forward"] = 0
            if key == R1_BACKWARD:
                self.status["r1_backward"] = 0
            if key == R1_LEFT:
                self.status["r1_left"] = 0
            if key == R1_RIGHT:
                self.status["r1_right"] = 0

            if key == R2_FORWARD:
                self.status["r2_forward"] = 0
            if key == R2_BACKWARD:
                self.status["r2_backward"] = 0
            if key == R2_LEFT:
                self.status["r2_left"] = 0
            if key == R2_RIGHT:
                self.status["r2_right"] = 0

            self.r1_des_vel.linear.x = self.status["r1_forward"] - self.status["r1_backward"]
            self.r1_des_vel.angular.z = self.status["r1_left"] - self.status["r1_right"]
            self.r2_des_vel.linear.x = self.status["r2_forward"] - self.status["r2_backward"]
            self.r2_des_vel.angular.z = self.status["r2_left"] - self.status["r2_right"]

        except rospy.ROSInterruptException:
            pass

    def pub_twist(self, event=None):
        self.cur_time = time()
        dt = self.cur_time - self.prev_time
        r1_dvx = self.r1_des_vel.linear.x - self.r1_prev_msg.linear.x
        r1_dvz = self.r1_des_vel.angular.z - self.r1_prev_msg.angular.z

        r2_dvx = self.r2_des_vel.linear.x - self.r2_prev_msg.linear.x
        r2_dvz = self.r2_des_vel.angular.z - self.r2_prev_msg.angular.z

        r1_msg = Twist()
        if r1_dvx > LINEAR_ACCEL * dt:
            # only when the difference is greater than the acceleration, we will accelerate
            r1_msg.linear.x = self.r1_prev_msg.linear.x + LINEAR_ACCEL * dt
        elif r1_dvx < -LINEAR_DECEL * dt:
            r1_msg.linear.x = self.r1_prev_msg.linear.x - LINEAR_DECEL * dt
        else:
            # otherwise, we will just set the velocity to the desired velocity directly
            r1_msg.linear.x = self.r1_des_vel.linear.x
            
        if r1_dvz > ANGULAR_ACCEL * dt:
            r1_msg.angular.z = self.r1_prev_msg.angular.z + ANGULAR_ACCEL * dt
        elif r1_dvz < -ANGULAR_DECEL * dt:
            r1_msg.angular.z = self.r1_prev_msg.angular.z - ANGULAR_DECEL * dt
        else:
            r1_msg.angular.z = self.r1_des_vel.angular.z

        r2_msg = Twist()
        if r2_dvx > LINEAR_ACCEL * dt:
            # only when the difference is greater than the acceleration, we will accelerate
            r2_msg.linear.x = self.r2_prev_msg.linear.x + LINEAR_ACCEL * dt
        elif r2_dvx < -LINEAR_DECEL * dt:
            r2_msg.linear.x = self.r2_prev_msg.linear.x - LINEAR_DECEL * dt
        else:
            # otherwise, we will just set the velocity to the desired velocity directly
            r2_msg.linear.x = self.r2_des_vel.linear.x
            
        if r2_dvz > ANGULAR_ACCEL * dt:
            r2_msg.angular.z = self.r2_prev_msg.angular.z + ANGULAR_ACCEL * dt
        elif r2_dvz < -ANGULAR_DECEL * dt:
            r2_msg.angular.z = self.r2_prev_msg.angular.z - ANGULAR_DECEL * dt
        else:
            r2_msg.angular.z = self.r2_des_vel.angular.z

        self.r1_cmd_vel_publisher.publish(r1_msg)
        self.r1_prev_msg = r1_msg
        self.r2_cmd_vel_publisher.publish(r2_msg)
        self.r2_prev_msg = r2_msg

        self.prev_time = self.cur_time


if __name__ == '__main__':
    rospy.init_node('r1_keyboard_driver_node', anonymous=True)
    node = KeyboardDriverNode()
    while not rospy.is_shutdown():
        with keyboard.Listener(
            on_press=node.on_press, on_release=node.on_release
        ) as listener:
            listener.join()
