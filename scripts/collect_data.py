import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import sys
from threading import Thread, Event


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5
from peripherals.keystroke_counter import KeystrokeCounter, KeyCode


def camera_process(camera_event, arm_event, finish_event, duration):
    cam1_serial_number = '230322277180'
    cam2_serial_number = '230322271473'
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device(cam1_serial_number)
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # ...from Camera 2
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device(cam2_serial_number)
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    camera_event.set()
    arm_event.wait()
    
    
    while not finish_event.is_set():
        start_time = time.perf_counter()
        frames_1 = pipeline_1.wait_for_frames()
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())

        # frames_2 = pipeline_2.wait_for_frames()
        # depth_frame_2 = frames_2.get_depth_frame()
        # color_frame_2 = frames_2.get_color_frame()
        # depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        # color_image_2 = np.asanyarray(color_frame_2.get_data())
        end_time = time.perf_counter()
        duration[0] = end_time - start_time


def arm_process(camera_event, arm_event, finish_event, duration):
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
    camera_event.wait()
    # start_time2 = time.time()

    start_time = time.perf_counter()
    print("Following Starts.")

    controller0.set_to_damping()
    gain = controller0.get_gain()
    gain.kd()[:] *= 0.1
    controller0.set_gain(gain)  # set to passive

    while not finish_event.is_set():
        global_time = time.time()
        eef_state = controller0.get_eef_state()
        eef_state.timestamp = 0.0
        eef_state.gripper_pos *= 5
        controller1.set_eef_cmd(eef_state)
        start_time = time.monotonic()

        time.sleep(controller1_config.controller_dt)
    end_time = time.perf_counter()
    duration[0] = global_time

    print(f"Fllowing stopped!")
    controller0.reset_to_home()
    controller1.reset_to_home()


def main():
    camera_ready_event = Event()
    arm_ready_event = Event()
    finish_event = Event()
    d1 = [0]
    d2 = [0]
    t1 = Thread(target=camera_process, args=(camera_ready_event, arm_ready_event, finish_event,d1,))
    t2 = Thread(target=arm_process, args=(camera_ready_event, arm_ready_event, finish_event,d2,))

    t1.start()
    t2.start()

    quit_pressed = False
    while not quit_pressed:
        time.sleep(1)
        quit_pressed = True
        finish_event.set()
        
    
    t1.join()
    t2.join()
    print(d1[0])
    print(d2[0])

if __name__ == "__main__":
    main()
    


