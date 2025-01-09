import os
import zarr
import numpy as np

trial_num = "1"
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
# print(arm_time)
# print(arm_time.shape)
# print(cam1_time.shape)
# print(arm_time[0])
# print(arm_time[-1])
# print(cam1_time[0])
# print(cam1_time[-1])

cam_data_len = cam1_time.shape[0]
if cam1_time.shape[0] > cam2_time.shape[0]:
    print("camera shape not equal")
    cam1_data_len = cam2_time.shape[0]

store = zarr.DirectoryStore('pick_and_place_data.zarr')
root = zarr.group(store=store)
action_folder = root.create_group('action')
cam1_folder = root.create_group('cam1')
cam2_folder = root.create_group('cam2')

cam1_arr = []
i = 1
for j in range(cam_data_len):
    if i < cam1_time.shape[0]-1:
        if np.abs(cam1_time[i]-arm_time[j]) < 0.005:
            print("%f, %f, %f"%(cam1_time[i], arm_time[j], arm_time[j+20]))
            # data_arr = cam1_folder.create_dataset(str(i-1)+".1", shape=, chunks=(5,), dtype='f4')
            cam1_arr.append()
            cam2_arr = 
            i+=1



# data_arr[:] = [1,2,3,4,5]
# print(root)

