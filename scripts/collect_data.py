import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import sys
from threading import Thread, Event
from PIL import Image
from pynput import keyboard

curr_dir = os.getcwd()
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5

gripper_pos_data = []
gripper_torque_data = []
gripper_vel_data = []
joint_pos_data = []
joint_vel_data = []
joint_torque_data = []
cam1_time = []
cam2_time = []
arm_time = []


def camera1_process(camera1_event, camera2_event, arm_event, finish_event):
    global cam1_time
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
        image.save("/home/wfy/data/1-"+str(end_time) + ".jpg")
        if interval <= 0.1:
            time.sleep(0.1 - interval)

def camera2_process(camera1_event, camera2_event, arm_event, finish_event):
    global cam2_time
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
        image.save("/home/wfy/data/2-"+str(end_time) + ".jpg")
        if interval <= 0.1:
            time.sleep(0.1 - interval)



def arm_process(camera1_event, camera2_event, arm_event, finish_event):
    global gripper_pos_data
    global gripper_vel_data
    global gripper_torque_data
    global joint_pos_data
    global joint_vel_data
    global joint_torque_data
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
    print("Following Starts.")

    controller0.set_to_damping()
    gain = controller0.get_gain()
    gain.kd()[:] *= 0.1
    controller0.set_gain(gain)  # set to passive

    start_time = time.perf_counter()
    while not finish_event.is_set():
        global_time = time.time()
        eef_state = controller0.get_eef_state()
        eef_state.timestamp = 0.0
        eef_state.gripper_pos *= 5
        controller1.set_eef_cmd(eef_state)

        follower_joint_state = controller1.get_joint_state()
        gripper_pos_data.append(follower_joint_state.gripper_pos)
        gripper_vel_data.append(follower_joint_state.gripper_vel)
        gripper_torque_data.append(follower_joint_state.gripper_torque)
        joint_pos_data.append(follower_joint_state.pos())
        joint_vel_data.append(follower_joint_state.vel())
        joint_torque_data.append(follower_joint_state.torque())

        arm_time.append(time.perf_counter())

        time.sleep(controller1_config.controller_dt)

    print(f"Fllowing stopped!")
    controller0.reset_to_home()
    controller1.reset_to_home()


def main():
    camera1_ready_event = Event()
    camera2_ready_event = Event()
    arm_ready_event = Event()
    finish_event = Event()

    t1 = Thread(target=camera1_process, args=(camera1_ready_event, camera2_ready_event, arm_ready_event, finish_event,))
    t2 = Thread(target=camera2_process, args=(camera1_ready_event, camera2_ready_event, arm_ready_event, finish_event,))
    t3 = Thread(target=arm_process, args=(camera1_ready_event, camera2_ready_event, arm_ready_event, finish_event,))

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
    arm_time_np = np.array(arm_time)
    cam1_time_np = np.array(cam1_time)
    cam2_time_np = np.array(cam2_time)
    root_path = "/home/wfy/data/"
    np.save(root_path + "gripper_pos.npy", gripper_pos_np)
    np.save(root_path + "gripper_vel.npy", gripper_vel_np)
    np.save(root_path + "gripper_torque.npy", gripper_torque_np)
    np.save(root_path + "joint_pos.npy", joint_pos_np)
    np.save(root_path + "joint_vel.npy", joint_vel_np)
    np.save(root_path + "joint_torque.npy", joint_torque_np)
    np.save(root_path + "arm_time_stamp.npy", arm_time_np)
    np.save(root_path + "cam1_time_stamp.npy", cam1_time_np)
    np.save(root_path + "cam2_time_stamp.npy", cam2_time_np)

if __name__ == "__main__":
    main()
    


