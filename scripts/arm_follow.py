import time
import numpy as np
import os
import sys
import click

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5
from peripherals.keystroke_counter import KeystrokeCounter, KeyCode

@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
def main(model: str):
    np.set_printoptions(precision=3, suppress=True)
    interface0 = "can0"
    interface1 = "can1"
    # arx5_0 = arx5.Arx5JointController(model, interface0)
    # arx5_1 = arx5.Arx5JointController(model, interface1)
    controller0 = arx5.Arx5JointController(model, interface0)
    controller1 = arx5.Arx5JointController(model, interface1)
    robot0_config = controller0.get_robot_config()
    controller0_config = controller1.get_controller_config()
    robot1_config = controller0.get_robot_config()
    controller1_config = controller1.get_controller_config()

    controller0.enable_background_send_recv()
    controller0.reset_to_home()
    controller0.enable_gravity_compensation("../models/arx5.urdf")
    controller1.enable_background_send_recv()
    controller1.reset_to_home()
    controller1.enable_gravity_compensation("../models/arx5.urdf")

    print("Following mode ready. Press 's' to start.")

    following_started = False
    with KeystrokeCounter() as key_counter:
        while True:
            press_events = key_counter.get_press_events()
            for key_stroke in press_events:
                if key_stroke == KeyCode(char="t"):
                    if following_started:
                        print(f"Following is already started!")
                        continue
                    print("Following started! Press 'q' to quit teaching.")
                    controller0.set_to_damping()
                    gain = controller0.get_gain()
                    gain.kd()[:] *= 0.1
                    controller0.set_gain(gain)  # set to passive
                    following_started = True
                    start_time = time.monotonic()
                elif key_stroke == KeyCode(char="q"):
                    print(f"Teaching stopped!")
                    following_started = False
                    controller0.reset_to_home()
                    controller1.reset_to_home()
                    return
            if following_started:
                joint_state = controller0.get_state() 
                controller1.set_joint_cmd(joint_state)
                time.sleep(controller1_config.controller_dt)
                

if __name__ == "__main__":
    main()
