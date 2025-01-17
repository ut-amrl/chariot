"""
Copyright (c) 2023 Maxwell Svetlik

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""



"""
This script interprets gamepad input as joystick commands to control Odrive controllers through a MotorManager.

The joysticks report values in [-1,1] which are multiplied by a set constant to achieve
the input speed to the motor controller.
"""

import datetime
import threading
import time
import structlog

from chariot.control.odrive_manager import OdriveMotorManager
from chariot.joystick import (
    JoystickController,
    XboxControllerResult,
)

LOG = structlog.get_logger()

LINEAR_SPEED_MULTIPLIER = 5  # Each integer increase of the multiplier increases velocity by ~0.314 m/s.
ANGULAR_SPEED_MULTIPLIER = 10  # radians/s
MESSAGE_LIFETIME = datetime.timedelta(
    seconds=0.2
)  # s; the length of time a single message is valid in the
# absence of another command. This serves as a safety feature in the event connection with the gamepad is lost.

VERBOSE = False

def run_joystick_daemon(jc: JoystickController):
    threading.Thread(target=jc.run).start()


def get_cmd(msg: XboxControllerResult):
    if datetime.datetime.now() - msg.born >= MESSAGE_LIFETIME:
        if VERBOSE:
            LOG.debug("Last joystick command outside lifecycle. Sending 0 velocity.")
        return XboxControllerResult()
    return msg.controller


def main():
    mgr = OdriveMotorManager()
    jc = JoystickController()
    run_joystick_daemon(jc)

    try:
        mgr.set_motors_idle()
        mgr.set_motors_active()
        while True:
            controller_res = get_cmd(jc.get_input())
            target_vel = controller_res.LeftJoystickY
            radius = controller_res.LeftJoystickX
            mgr.set_motors_arc_velocity(
                target_vel * LINEAR_SPEED_MULTIPLIER, radius * ANGULAR_SPEED_MULTIPLIER
            )
            time.sleep(0.025)
    except KeyboardInterrupt:
        mgr.set_motors_arc_velocity(0, 0)
        LOG.info("Shutdown signal received. Ending execution.")


if __name__ == "__main__":
    main()
