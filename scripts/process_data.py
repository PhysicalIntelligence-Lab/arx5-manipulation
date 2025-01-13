import os
import zarr
import numpy as np

# trial_num = "1"
# root_path = os.path.expanduser("~") + "/data/trial" + trial_num + "/"
# output_path = os.path.expanduser("~") + "data/dataset/data" + trial_num

os.system('rm -rf pick_and_place_data.zarr')
store = zarr.DirectoryStore('pick_and_place_data.zarr')
root = zarr.group(store=store)
data_group = root.create_group('data')
meta_group = root.create_group('meta')
action_arr = data_group.create_dataset('action', shape=(0,7,), chunks=(100, 7), dtype='f4')
img1_arr = data_group.create_dataset('img1', shape = (0,3,480,640), chunks = (10, 3, 480, 640), dtype='f4')
img2_arr = data_group.create_dataset('img2', shape = (0,3,480,640), chunks = (10, 3, 480, 640), dtype='f4')
img1_depth_arr = data_group.create_dataset('depth1', shape = (0,480,640), chunks = (10, 480, 640), dtype='f4')
img2_depth_arr = data_group.create_dataset('depth2', shape = (0,480,640), chunks = (10, 480, 640), dtype='f4')
joint_pos_arr = data_group.create_dataset('joint_pos', shape = (0,7,), chunks = (100, 7), dtype='f4')
joint_vel_arr = data_group.create_dataset('joint_vel', shape = (0,7,), chunks = (100, 7), dtype='f4')
joint_torque_arr = data_group.create_dataset('joint_torque', shape = (0,7,), chunks = (100, 7), dtype='f4')

episode_end_idx = []

max_episode = 1
data_idx = 0
for episode_num in range(1, max_episode+1):
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
            elif file[0:4] == "cam1" and file.endswith("_color.npy"):
                cam1_file.append(file)
            elif file[0:4] == "cam1" and file.endswith("_depth.npy"):
                cam1_depth_file.append(file)
            elif file[0:4] == "cam2" and file.endswith("_color.npy"):
                cam2_file.append(file)
            elif file[0:4] == "cam2" and file.endswith("_depth.npy"):
                cam2_depth_file.append(file)


    cam1_file.sort()
    cam2_file.sort()
    cam1_depth_file.sort()
    cam2_depth_file.sort()
    # Add image to the zarr (get rid of the last image)
    for i in range(len(cam1_file) - 1):
        img1_arr.resize((img1_arr.shape[0]+1, img1_arr.shape[1], img1_arr.shape[2], img1_arr.shape[3]))
        img1 = np.load(dir_path + cam1_file[i])
        img2_arr.resize((img2_arr.shape[0]+1, img2_arr.shape[1], img2_arr.shape[2], img2_arr.shape[3]))
        img2 = np.load(dir_path + cam2_file[i])
        # img1_arr[img1_arr.shape[0]-1] = zarr.array(img1)
    
    # Add to the action and joint_pos zarr array
    # action_arr.resize((action_arr.shape[0]+1, action_arr.shape[1]))
    # action_arr[action_arr.shape[0]-1] = zarr.array(joint_pos)

    # joint_pos_arr.resize((joint_pos_arr.shape[0]+1, joint_pos_arr.shape[1]))
    # joint_pos_arr[joint_pos_arr.shape[0]-1] = zarr.array(joint_pos)

    


# # print(cnt)

# data = np.load(os.path.join(root_path, "joint_vel.npy"))

# cam_data_len = cam1_time.shape[0]
# if cam1_time.shape[0] > cam2_time.shape[0]:
#     print("camera shape not equal")
#     cam1_data_len = cam2_time.shape[0]



# cam1_arr = []
# i = 0
# arm_idx = []
# for j in range(cam_data_len-1):
#     while i < arm_time.shape[0]:
#         if np.abs(cam1_time[j]-arm_time[i]) < 0.01:
#             print("%f, %f, %f"%(cam1_time[j], arm_time[i], arm_time[i+20]))
#             # data_arr = cam1_folder.create_dataset(str(i-1)+".1", shape=, chunks=(5,), dtype='f4')
#             # cam1_arr.append()
#             # cam2_arr.append()
#             arm_idx.append(i)
#             break
#         i+=1

# print(joint_torque.shape)
# print(arm_idx)



