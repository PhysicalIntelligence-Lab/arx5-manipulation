import mujoco
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
# from transforms3d.derivations.quaternions import qmult
# from transforms3d.euler import quat2euler
# from transforms3d.quaternions import qinverse

class RingBuffer:
    def __init__(self,capacity):
        self.capacity = capacity
        self.buffer = [None] * capacity
        self.head = 0
        self.size = 0
    
    def append(self,val):
        self.buffer[self.head] = val
        self.head = (self.head + 1) % self.capacity
        if self.size < self.capacity:
            self.size += 1
    
    def get_mean(self):
        sum = 0
        for i in range(self.size):
            sum += self.buffer[i]
        return sum / self.size
    
    def is_full(self):
        if self.capacity == self.size:
            return True
        else:
            return False

buffer = RingBuffer(10) 

def get_compensation(model,data,kt):
    orig_qacc = data.qacc.copy()
    data.qacc[:] = 0
    mujoco.mj_inverse(model,data)
    buffer.append(data.qfrc_inverse.copy())
    data.qacc = orig_qacc.copy()
    compensation = buffer.get_mean()
    
    if buffer.is_full():
        compensation = compensation / kt
    else:
        compensation = np.zeros(compensation.size)

    return compensation

# implemented with gradient descent
def inverse_kinematics(model, data, target_position, target_orientation):

    tol = 0.01
    step = 0.0001
    iterations = 20000
    jacp = np.zeros((3,model.nv))
    jacr = np.zeros((3,model.nv))
    mujoco.mj_forward(model,data)
    eef_id = model.body("link6").id
    

    for _ in range(iterations):
        err_p = d.xpos[eef_id] - target_position
        err_r = np.array(qmult(d.xquat[eef_id] , qinverse(target_orientation)))
        err_r = quat2euler(err_r)
        # err_r = d.xquat[eef_id][1:] / d.xquat[eef_id][0]  - target_orientation[1:] / target_orientation[0]
        # print(R.from_quat(d.xquat[eef_id]).inv())
        err = np.hstack((err_r,err_p))
        print(np.linalg.norm(err))
        if np.linalg.norm(err) < tol:
            break

        mujoco.mj_jacBodyCom(model,data,jacp,jacr,eef_id)
        J = np.vstack((jacp,jacr))
        J_inv = np.linalg.pinv(J)
        delta_q = J_inv @ err
        d.qpos += step * delta_q
        mujoco.mj_kinematics(model,data)

    
    return d.qpos


if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_path('./models/R5a/urdf/R5a.xml')
    d = mujoco.MjData(m)
    for i in range(6):
        d.qpos[i] = 0.1
    mujoco.mj_forward(m,d)

    eef_id = m.body("link6").id

    target_pos = d.xpos[eef_id].copy()
    target_quat = d.xquat[eef_id].copy()
    print(target_pos)
    print(target_quat)

    for i in range(6):
        d.qpos[i] = 0.0

    sol = inverse_kinematics(m,d,target_pos,target_quat)
    print(sol)
