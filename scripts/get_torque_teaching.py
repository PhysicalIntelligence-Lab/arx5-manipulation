import os
import sys
import time
import numpy as np
import click
import matplotlib.pyplot as plt

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from arx5_interface import Arx5CartesianController, EEFState, Gain


from peripherals.keystroke_counter import KeystrokeCounter, KeyCode



def collet_torque_data(controller: Arx5CartesianController, data_file: str):
    controller.reset_to_home()

    controller_config = controller.get_controller_config()

    print("Collect torque mode ready. Press 't' to start teaching.")
    teaching_started = False
    traj = []
    start_time = 0
    with KeystrokeCounter() as key_counter:
        while True:
            press_events = key_counter.get_press_events()
            for key_stroke in press_events:
                if key_stroke == KeyCode(char="t"):
                    controller.set_to_damping()

                    gain = controller.get_gain()
                    gain.kd()[:] *= 0.1
                    controller.set_gain(gain)  # set to passive
                    if teaching_started:
                        print(f"Teaching is already started!")
                        continue
                    print("Teaching started! Press 'q' to quit teaching.")
                    teaching_started = True
                    start_time = time.monotonic()
                elif key_stroke == KeyCode(char="q"):
                    print(f"Teaching stopped! Trajectory saved to file {data_file}")
                    teaching_started = False
                    np.save(data_file, traj, allow_pickle=True)
                    return
            if teaching_started:
                state = controller.get_eef_state()
                state.timestamp = time.monotonic() - start_time
                traj.append(
                    {
                        "pose_6d": state.pose_6d().copy(),
                        "gripper_pos": state.gripper_pos,
                        "gripper_torque": state.gripper_torque
                    }
                )
                time.sleep(controller_config.controller_dt)

def plot_torque(data_file: str):
    data = np.load(data_file, allow_pickle=True)
    torque = []
    time_stamp = []
    curr_time = 0
    for info in data:
        torque.append(info["gripper_torque"])
        time_stamp.append(curr_time)
        curr_time += 1
    # print(torque)
    plt.plot(time_stamp, torque)
    plt.show()

@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
@click.option("--urdf_path", "-u", default="../models/arx5.urdf", help="URDF file path")
def main(model: str, interface: str, urdf_path: str):
    controller = Arx5CartesianController(model, interface, urdf_path)

    np.set_printoptions(precision=4, suppress=True)
    os.makedirs("data", exist_ok=True)
    collet_torque_data(controller, "data/teach_traj.npy")
    plot_torque("data/teach_traj.npy")

if __name__ == "__main__":
    main()
