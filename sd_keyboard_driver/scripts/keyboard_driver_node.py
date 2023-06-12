import rospy

from typing import Optional, Union

from pynput import keyboard
from pynput.keyboard import Key, KeyCode
from geometry_msgs.msg import Twist


# key bindings
FORWARD = "w"
BACKWARD = "s"
LEFT = "a"
RIGHT = "d"

HELP = "p"

HELP_MSG = """
Use keyboard to control SoonDuck

Key Bindings:
   [w]
[a][s][d]

[w] = Forward
[s] = Backward
[a] = Left
[d] = Right

[p] = Show this help"""


class KeyboardDriverNode():
    def __init__(self):
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.loginfo(HELP_MSG)
        self.status = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
        }

    def on_press(self, key: Optional[Union[Key, KeyCode]]):
        try:
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == FORWARD:
                self.status["forward"] = True
            if key == BACKWARD:
                self.status["backward"] = True
            if key == LEFT:
                self.status["left"] = True
            if key == RIGHT:
                self.status["right"] = True
            if key == HELP:
                rospy.loginfo(HELP_MSG)

            self.pub_twist()

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

            if key == FORWARD:
                self.status["forward"] = False
            if key == BACKWARD:
                self.status["backward"] = False
            if key == LEFT:
                self.status["left"] = False
            if key == RIGHT:
                self.status["right"] = False

            self.pub_twist()

        except rospy.ROSInterruptException:
            pass

    def pub_twist(self):
        msg = Twist()
        msg.linear.x = (self.status["forward"] - self.status["backward"])
        msg.angular.z = (self.status["left"] - self.status["right"])
        self.cmd_vel_publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node('keyboard_driver_node', anonymous=True)
    node = KeyboardDriverNode()
    while not rospy.is_shutdown():
        with keyboard.Listener(
            on_press=node.on_press, on_release=node.on_release
        ) as listener:
            listener.join()
