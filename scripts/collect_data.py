import pyrealsense2 as rs
import numpy as np
from scipy.spatial.transform import Rotation
import time
import os
import sys
from threading import Thread, Event
from PIL import Image
from pynput import keyboard
import click

curr_dir = os.getcwd()
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5

gripper_pos_data = []
gripper_torque_data = []
gripper_vel_data = []
eef_pos_data = []
eef_pose_data = []
joint_pos_data = []
joint_vel_data = []
joint_torque_data = []
cam1_time = []
cam2_time = []
arm_time = []


def camera1_thread(camera1_event, camera2_event, arm_event, finish_event, trial_num):
    global cam1_time
    root_path = os.path.expanduser("~") + "/data/trial" + trial_num + "/"
    cam1_serial_number = '230322277180'
    
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device(cam1_serial_number)
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start camera1 pipeline
    pipeline_1.start(config_1)

    # Synchronization
    camera1_event.set()
    camera2_event.wait()
    arm_event.wait()
    
    cnt = 0
    while not finish_event.is_set():    
        start_time = time.perf_counter()
        frames_1 = pipeline_1.wait_for_frames()
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        end_time = time.perf_counter()
        interval = end_time - start_time
        cam1_time.append(time.perf_counter())
        image = Image.fromarray(color_image_1)
        cnt += 1
        image.save(root_path + "1-"+str(cnt) + ".jpg")

        np.save(root_path + "cam1_"+ str(cnt) + "_color.npy", color_image_1)
        np.save(root_path + "cam1_" + str(cnt) + "_depth.npy", depth_image_1)

        if interval <= 0.1:
            time.sleep(0.1 - interval)

def camera2_thread(camera1_event, camera2_event, arm_event, finish_event, trial_num):
    global cam2_time
    root_path = os.path.expanduser("~") + "/data/trial" + trial_num + "/"
    # ...from Camera 2
    cam2_serial_number = '230322271473'
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device(cam2_serial_number)
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start camera1 pipeline
    pipeline_2.start(config_2)

    # Synchronization
    camera2_event.set()
    camera1_event.wait()
    arm_event.wait()

    cnt = 0
    while not finish_event.is_set():   
        start_time = time.perf_counter() 
        frames_2 = pipeline_2.wait_for_frames()
        depth_frame_2 = frames_2.get_depth_frame()
        color_frame_2 = frames_2.get_color_frame()
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        end_time = time.perf_counter()
        interval = end_time - start_time
        cam2_time.append(end_time)
        image = Image.fromarray(color_image_2)
        cnt += 1
        image.save(root_path + "2-"+str(cnt) + ".jpg")
        np.save(root_path + "cam2_"+ str(cnt) + "_color.npy", color_image_2)
        np.save(root_path + "cam2_" + str(cnt) + "_depth.npy", depth_image_2)

        if interval <= 0.1:
            time.sleep(0.1 - interval)

def arm_thread(camera1_event, camera2_event, arm_event, finish_event):
    global gripper_pos_data
    global gripper_vel_data
    global gripper_torque_data
    global joint_pos_data
    global joint_vel_data
    global joint_torque_data
    global eef_pos_data
    global eef_pose_data
    global arm_time
    np.set_printoptions(precision=3, suppress=True)
    interface0 = "can0"
    interface1 = "can1"
    urdf = "../models/arx5.urdf"
    controller0 = arx5.Arx5CartesianController("L5", interface0, urdf)
    controller1 = arx5.Arx5CartesianController("L5", interface1, urdf)
    robot0_config = controller0.get_robot_config()
    controller0_config = controller1.get_controller_config()
    robot1_config = controller0.get_robot_config()
    controller1_config = controller1.get_controller_config()
    controller0.reset_to_home()
    controller1.reset_to_home()

    arm_event.set()
    camera1_event.wait()
    camera2_event.wait()

    start_time = time.perf_counter()
    print("Following Starts. Press `esc` to quit.")

    controller0.set_to_damping()
    gain = controller0.get_gain()
    gain.kd()[:] *= 0.1
    controller0.set_gain(gain)  # set to passive

    
    while not finish_event.is_set():
        start_time = time.perf_counter()
        eef_state = controller0.get_eef_state()
        eef_state.timestamp = 0.0
        eef_state.gripper_pos *= 5
        eef_state.gripper_vel = 0.0
        eef_state.gripper_torque = 0.0
        controller1.set_eef_cmd(eef_state)

        follower_joint_state = controller1.get_joint_state()

        joint_pos_data.append(follower_joint_state.pos())
        joint_vel_data.append(follower_joint_state.vel())
        joint_torque_data.append(follower_joint_state.torque())

        gripper_pos_data.append(follower_joint_state.gripper_pos)
        gripper_vel_data.append(follower_joint_state.gripper_vel)
        gripper_torque_data.append(follower_joint_state.gripper_torque)
        
        eef_data = eef_state.pose_6d()
        eef_pose_data.append(Rotation.from_euler('xyz', eef_data[3:], degrees=False).as_quat())
        eef_pos_data.append(eef_data[0:3])

        arm_time.append(time.perf_counter())

        end_time = time.perf_counter()  
        time.sleep(controller1_config.controller_dt - (end_time - start_time))

    print(f"Fllowing stopped!")
    controller0.reset_to_home()
    controller1.reset_to_home()

@click.command()
@click.argument("trial_num")  # ARX arm model: X5 or L5
def main(trial_num: str):
    root_path = os.path.expanduser("~") + "/data/trial"+ trial_num + "/"
    if os.path.exists(root_path):
        print("Trial already exist! Enter another trial number.")
        return
    else:
        os.makedirs(root_path)
    camera1_ready_event = Event()
    camera2_ready_event = Event()
    arm_ready_event = Event()
    finish_event = Event()

    t1 = Thread(target=camera1_thread, args=(camera1_ready_event, camera2_ready_event, arm_ready_event, finish_event, trial_num, ))
    t2 = Thread(target=camera2_thread, args=(camera1_ready_event, camera2_ready_event, arm_ready_event, finish_event, trial_num, ))
    t3 = Thread(target=arm_thread, args=(camera1_ready_event, camera2_ready_event, arm_ready_event, finish_event, ))

    t1.start()
    t2.start()
    t3.start()

    with keyboard.Events() as events:
        for event in events:
            if event.key == keyboard.Key.esc:
                finish_event.set()
                break

    t1.join()
    t2.join()
    t3.join()

    gripper_pos_np = np.array(gripper_pos_data)
    gripper_vel_np = np.array(gripper_vel_data)
    gripper_torque_np = np.array(gripper_torque_data)
    joint_pos_np = np.array(joint_pos_data)
    joint_vel_np = np.array(joint_vel_data)
    joint_torque_np = np.array(joint_torque_data)

    eef_pos_np = np.array(eef_pos_data)
    eef_pose_np = np.array(eef_pose_data)

    arm_time_np = np.array(arm_time)
    cam1_time_np = np.array(cam1_time)
    cam2_time_np = np.array(cam2_time)

    traj_pos_np = np.hstack((joint_pos_np, gripper_pos_np.reshape((-1,1))))
    traj_vel_np = np.hstack((joint_vel_np, gripper_vel_np.reshape((-1,1))))
    traj_torque_np = np.hstack((joint_torque_np, gripper_torque_np.reshape((-1,1))))

    np.save(root_path + "gripper_pos.npy", gripper_pos_np)
    np.save(root_path + "gripper_vel.npy", gripper_vel_np)
    np.save(root_path + "gripper_torque.npy", gripper_torque_np)
    np.save(root_path + "joint_pos.npy", joint_pos_np)
    np.save(root_path + "joint_vel.npy", joint_vel_np)
    np.save(root_path + "joint_torque.npy", joint_torque_np)
    np.save(root_path + "traj_pos.npy", traj_pos_np)
    np.save(root_path + "traj_vel.npy", traj_vel_np)
    np.save(root_path + "traj_torque.npy", traj_torque_np)
    np.save(root_path + "eef_pos.npy", eef_pos_data)
    np.save(root_path + "eef_pose.npy", eef_pose_data)
    np.save(root_path + "arm_time_stamp.npy", arm_time_np)
    np.save(root_path + "cam1_time_stamp.npy", cam1_time_np)
    np.save(root_path + "cam2_time_stamp.npy", cam2_time_np)

if __name__ == "__main__":
    main()
    


