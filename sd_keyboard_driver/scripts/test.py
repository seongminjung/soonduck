from typing import Union

from pynput import keyboard
from pynput.keyboard import Key, KeyCode


# key bindings
FORWARD = "w"
BACKWARD = "s"
LEFT = "a"
RIGHT = "d"
UP = "2"
DOWN = "x"

ROLL_LEFT = "j"
ROLL_RIGHT = "l"
PITCH_UP = "i"
PITCH_DOWN = "k"
YAW_LEFT = "h"
YAW_RIGHT = ";"

HELP = "p"

HELP_MSG = """
Use keyboard to control ROV

Key Bindings:
   [2]
   [w]            [i]
[a][s][d]   [h][j][k][l][;]
   [x]

[w] = Forward
[s] = Backward
[a] = Left
[d] = Right
[2] = Up
[x] = Down

[j] = Roll Left
[l] = Roll Right
[i] = Pitch Up
[k] = Pitch Down
[h] = Yaw Left
[;] = Yaw Right

[p] = Show this help"""

status = {
    "forward": False,
    "backward": False,
    "left": False,
    "right": False,
    "up": False,
    "down": False,
    "roll_left": False,
    "roll_right": False,
    "pitch_up": False,
    "pitch_down": False,
    "yaw_left": False,
    "yaw_right": False,
}

def on_press(key: Union[Key, KeyCode, None]):
    if isinstance(key, keyboard.KeyCode):
        key = key.char
    elif isinstance(key, keyboard.Key):
        key = key.name

    if key == FORWARD:
        status["forward"] = True
    if key == BACKWARD:
        status["backward"] = True
    if key == LEFT:
        status["left"] = True
    if key == RIGHT:
        status["right"] = True
    if key == UP:
        status["up"] = True
    if key == DOWN:
        status["down"] = True
    if key == ROLL_LEFT:
        status["roll_left"] = True
    if key == ROLL_RIGHT:
        status["roll_right"] = True
    if key == PITCH_UP:
        status["pitch_up"] = True
    if key == PITCH_DOWN:
        status["pitch_down"] = True
    if key == YAW_LEFT:
        status["yaw_left"] = True
    if key == YAW_RIGHT:
        status["yaw_right"] = True
    if key == HELP:
        print(HELP_MSG)

    pub_twist()

def on_release(key: Union[Key, KeyCode, None]):
    if isinstance(key, keyboard.KeyCode):
        key = key.char
    elif isinstance(key, keyboard.Key):
        key = key.name

    if key == FORWARD:
        status["forward"] = False
    if key == BACKWARD:
        status["backward"] = False
    if key == LEFT:
        status["left"] = False
    if key == RIGHT:
        status["right"] = False
    if key == UP:
        status["up"] = False
    if key == DOWN:
        status["down"] = False
    if key == ROLL_LEFT:
        status["roll_left"] = False
    if key == ROLL_RIGHT:
        status["roll_right"] = False
    if key == PITCH_UP:
        status["pitch_up"] = False
    if key == PITCH_DOWN:
        status["pitch_down"] = False
    if key == YAW_LEFT:
        status["yaw_left"] = False
    if key == YAW_RIGHT:
        status["yaw_right"] = False

    pub_twist()

def pub_twist():
    print(status)

while True:
    with keyboard.Listener(
        on_press=on_press, on_release=on_release
    ) as listener:
        listener.join()