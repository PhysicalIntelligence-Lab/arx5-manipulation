import time
import numpy as np
import os
import sys
import click
from multiprocessing import Process, Queue

curr_dir = os.path.dirname(os.path.abspath(__file__))
print(curr_dir)
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
os.chdir(curr_dir)
import mujoco
import mujoco.viewer
from peripherals.keystroke_counter import KeystrokeCounter, KeyCode
from motor_controller import MotorControlConfig, MotorController
from utils import get_compensation
import matplotlib.pyplot as plt

from threading import Thread
import threading

from pynput import keyboard

def simulationThread(queue):
  dest = np.array([0,0,0,0,0,0,0.0,0])
  m = mujoco.MjModel.from_xml_path('./../models/R5a/urdf/R5a.xml')
  d = mujoco.MjData(m)
  # d.qfrc_applied[6] = 0
  # d.qfrc_applied[7] = 0

  i = 0
  with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.

    cfg = MotorControlConfig()
    controller = MotorController(cfg)
    controllers = []
    
    for i in range(m.nv):
      cfg = MotorControlConfig()
      controller = MotorController(cfg)
      cfg.kt = 0.2
      match i:
        case 0:
          cfg.kp = 10000
          cfg.kd = 500
        case 1:
          cfg.kp = 200
          cfg.kd = 50
        case 2:
          cfg.kp = 200
          cfg.kd = 30
        case 3:
          cfg.kp = 100
          cfg.kd = 10
        case 4:
          cfg.kp = 30
          cfg.kd = 5
        case 5:
          cfg.kp = 20
          cfg.kd = 2
        case 6:
          cfg.kp = 500
          cfg.kd = 50
        case 7:
          cfg.kp = 500
          cfg.kd = 50
      controllers.append(controller)

    data = []
    t = []
    torque = []
    cnt = 0


    num_step = 1
    step_cnt = 0

    while viewer.is_running():
      if not queue.empty():
        dest = queue.get()
      step_start = time.time()

      # mj_step can be replaced with code that also evaluates
      # a policy and applies a control signal before stepping the physics.
      mujoco.mj_step(m, d)
      viewer.sync()
      compensation = get_compensation(m,d,controllers[0].param.kt)
      for i in range(m.nv):
        d.qfrc_applied[i] = controllers[i].output(d.qpos[i],d.qvel[i], dest[i] * step_cnt / num_step,0.0,compensation[i])

      time_until_next_step = m.opt.timestep - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)
      
      if step_cnt < num_step:
        step_cnt += 1
    

def teleOpThread(queue):
    dest = np.array([0,0,0,0,0,0,0.0,0])
    np.set_printoptions(precision=3, suppress=True)
    interface0 = "can0"
    model = "../arx5-sdk/models/arx5.urdf"
    controller0 = arx5.Arx5CartesianController("L5", interface0, model)
    robot0_config = controller0.get_robot_config()
    controller0_config = controller0.get_controller_config()
    controller0.reset_to_home()

    print("Following mode ready. Press 's' to start.")

    following_started = False
    with KeystrokeCounter() as key_counter:
        while True:
            press_events = key_counter.get_press_events()
            for key_stroke in press_events:
                if key_stroke == KeyCode(char="s"):
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
                    return
            if following_started:
                # joint_state = controller0.get_state() 
                # controller1.set_joint_cmd(joint_state)
                eef_state = controller0.get_eef_state()
                eef_state.timestamp = 0.0
                # print(eef_state.pose_6d)
                joint_state = controller0.get_joint_state()
                dest[:6] = joint_state.pos().copy()
                dest[6] = eef_state.gripper_pos * 1.8 + 0.002
                dest[7] = eef_state.gripper_pos * 1.8 + 0.002
                queue.put(dest)
                time.sleep(controller0_config.controller_dt)
            

def main():
    queue = Queue()
    process_mujoco = [Process(target=simulationThread, args=(queue,))]
    process_mujoco[0].start()
    teleOpThread(queue)
    process_mujoco[0].join()
    # sim_thread = Thread(simulationThread())
    # teleop_thread = Thread(teleOpThread())
    # teleop_thread.start()
    # sim_thread.start()
    # teleop_thread.join()
    # sim_thread.join()
    
    

if __name__ == "__main__":
    main()
  




### Useful Code Snippets
      # print(d.qpos[1])
      # data.append(d.qpos[1])
      # t.append(cnt)
      # cnt += 1

      # d.qfrc_applied[6] = 0
      # d.qfrc_applied[7] = 0
      # compensation = controllers[2].get_compensation(d.qfrc_applied[2])
