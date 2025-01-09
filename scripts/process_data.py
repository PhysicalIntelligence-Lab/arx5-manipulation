import os
# import zarr
import numpy as np

trial_num = "2"
root_path = os.path.expanduser("~") + "/data/trial" + trial_num + "/"
output_path = os.path.expanduser("~") + "data/dataset/data" + trial_num

cnt = 0
files = os.listdir(root_path)
for file in files:
    if os.path.isfile(os.path.join(root_path, file)) and file.endswith(".npy"):
        if file == "cam1_time_stamp.npy":
            cam1_time = np.load(root_path + "cam1_time_stamp.npy")
        elif file == "cam2_time_stamp.npy":
            cam2_time = np.load(root_path + "cam2_time_stamp.npy")
        elif file == "arm_time_stamp.npy":
            arm_time = np.load(root_path + "arm_time_stamp.npy")

# print(cnt)

data = np.load(os.path.join(root_path, "joint_vel.npy"))
# print(data.shape)
print(cam1_time-cam2_time)
print(arm_time)
print(arm_time.shape)
print(cam1_time.shape)
print(arm_time[-2])
print(cam1_time[-2])
# print(cam2_time)


print(val)


