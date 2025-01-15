import os
import zarr
import numpy as np
from tqdm import tqdm

os.system('rm -rf pick_and_place_data.zarr')
output_path = os.path.expanduser("~") + "/data/pick_and_place_data.zarr"
store = zarr.DirectoryStore(output_path)
root = zarr.group(store=store)
data_group = root.create_group('data')
meta_group = root.create_group('meta')
action_arr = data_group.create_dataset('action', shape=(0,7,), chunks=(10, 7), dtype='f4')
img1_arr = data_group.create_dataset('img1', shape = (0,480,640,3), chunks = (10, 480, 640, 3), dtype='f4')
img2_arr = data_group.create_dataset('img2', shape = (0,480,640,3), chunks = (10, 480, 640, 3), dtype='f4')
img1_depth_arr = data_group.create_dataset('depth1', shape = (0,480,640), chunks = (10, 480, 640), dtype='f4')
img2_depth_arr = data_group.create_dataset('depth2', shape = (0,480,640), chunks = (10, 480, 640), dtype='f4')
joint_pos_arr = data_group.create_dataset('joint_pos', shape = (0,7,), chunks = (10, 7), dtype='f4')
joint_vel_arr = data_group.create_dataset('joint_vel', shape = (0,7,), chunks = (10, 7), dtype='f4')
joint_torque_arr = data_group.create_dataset('joint_torque', shape = (0,20,7), chunks = (10,20,7), dtype='f8')
episode_end_arr = meta_group.create_dataset('episode_ends', shape = (0,), chunks = (10, ), dtype='i4')

episode_end_idx = []
acc_episode_len = 0

max_episode = 10
data_idx = 0

for episode_num in tqdm(range(1, max_episode+1), desc="Processing", unit="iter", ncols=100):
    dir_path = os.path.expanduser("~") + "/data/trial" + str(episode_num) + "/"
    cnt = 0
    files = os.listdir(dir_path)
    cam1_file = []
    cam2_file = []
    cam1_depth_file = []
    cam2_depth_file = []

    for file in files:
        if os.path.isfile(os.path.join(dir_path, file)) and file.endswith(".npy"):
            if file == "cam1_time_stamp.npy":
                cam1_time = np.load(dir_path + "cam1_time_stamp.npy")
            elif file == "cam2_time_stamp.npy":
                cam2_time = np.load(dir_path + "cam2_time_stamp.npy")
            elif file == "arm_time_stamp.npy":
                arm_time = np.load(dir_path + "arm_time_stamp.npy")
            elif file == "traj_pos.npy":
                joint_pos = np.load(dir_path + "traj_pos.npy")
            elif file == "traj_vel.npy":
                joint_vel = np.load(dir_path + "traj_vel.npy")
            elif file == "traj_torque.npy":
                joint_torque = np.load(dir_path + "traj_torque.npy")
            elif file == "eef_pose.npy":
                eef_pose = np.load(dir_path + "eef_pose.npy")
            elif file == "gripper_pos.npy":
                gripper_pos = np.load(dir_path + "gripper_pos.npy")
            elif file[0:4] == "cam1" and file.endswith("_color.npy"):
                cam1_file.append(file)
            elif file[0:4] == "cam1" and file.endswith("_depth.npy"):
                cam1_depth_file.append(file)
            elif file[0:4] == "cam2" and file.endswith("_color.npy"):
                cam2_file.append(file)
            elif file[0:4] == "cam2" and file.endswith("_depth.npy"):
                cam2_depth_file.append(file)

    # Update episode index (does not count last image)
    episode_len = len(cam1_file) - 1
    acc_episode_len += episode_len
    episode_end_idx.append(acc_episode_len)
    episode_end_arr.resize((episode_end_arr.shape[0]+1))
    episode_end_arr[-1] = acc_episode_len

    # Sort the files
    cam1_file.sort()
    cam2_file.sort()
    cam1_depth_file.sort()
    cam2_depth_file.sort()

    # Add image to the zarr (get rid of the last image)
    for i in range(episode_len):
        img1_arr.resize((img1_arr.shape[0]+1, img1_arr.shape[1], img1_arr.shape[2], img1_arr.shape[3]))
        img1 = np.load(dir_path + cam1_file[i])
        img1_arr[img1_arr.shape[0]-1] = zarr.array(img1)

        img2_arr.resize((img2_arr.shape[0]+1, img2_arr.shape[1], img2_arr.shape[2], img2_arr.shape[3]))
        img2 = np.load(dir_path + cam2_file[i])
        img2_arr[img1_arr.shape[0]-1] = zarr.array(img2)

        img1_depth_arr.resize((img1_depth_arr.shape[0]+1, img1_depth_arr.shape[1], img1_depth_arr.shape[2]))
        img1_depth = np.load(dir_path + cam1_depth_file[i])
        img1_depth_arr[img1_arr.shape[0]-1] = zarr.array(img1_depth)

        img2_depth_arr.resize((img2_depth_arr.shape[0]+1, img2_depth_arr.shape[1], img2_depth_arr.shape[2]))
        img2_depth = np.load(dir_path + cam2_depth_file[i])
        img2_depth_arr[img2_depth_arr.shape[0]-1] = zarr.array(img2_depth)

    # Map Image to joint information
    diff = cam1_time - cam2_time    # Make sure cam1_time and cam2_time has the same dimension
    i = 0
    arm_idx = []
    for j in range(episode_len):
        while i < arm_time.shape[0]:
            if np.abs(cam1_time[j]-arm_time[i]) < 0.01:
                # print("%f, %f, %f"%(cam1_time[j], arm_time[i], arm_time[i+20]))
                arm_idx.append(i)
                break
            i+=1
    # print(arm_idx)
    joint_pos_sample = zarr.array([joint_pos[s] for s in arm_idx])
    joint_vel_sample = zarr.array([joint_vel[s] for s in arm_idx])
    joint_torque_sample = zarr.array([joint_torque[s:s+20] for s in arm_idx])
    action_sample = zarr.array(np.hstack(([eef_pose[s] for s in arm_idx], [gripper_pos.reshape((-1,1))[s] for s in arm_idx])))

    # Implementation: Action is the same as joint position
    # action_arr.resize((action_arr.shape[0] + episode_len, action_arr.shape[1]))
    # action_arr[-episode_len:] = joint_pos_sample

    # Implementation: Action space is eef pose, and gripper position
    action_arr.resize((action_arr.shape[0] + episode_len, action_arr.shape[1]))
    action_arr[-episode_len:] = action_sample

    joint_pos_arr.resize((joint_pos_arr.shape[0] + episode_len, joint_pos_arr.shape[1]))
    joint_pos_arr[-episode_len:] = joint_pos_sample

    joint_vel_arr.resize((joint_vel_arr.shape[0] + episode_len, joint_vel_arr.shape[1]))
    joint_vel_arr[-episode_len:] = joint_vel_sample

    joint_torque_arr.resize((joint_torque_arr.shape[0] + episode_len, joint_torque_arr.shape[1], joint_torque_arr.shape[2]))
    joint_torque_arr[-episode_len:] = joint_torque_sample





