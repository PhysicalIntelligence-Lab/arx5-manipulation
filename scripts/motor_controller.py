
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

class MotorControlConfig():
    def __init__(self):
        self.kp = 10
        self.kd = 1
        self.kt = 1
        self.pmax = 12.5
        self.pmin = -12.5
        self.vmax = 45
        self.vmin = -45
        self.tmax = 1800
        self.tmin = -1800

class MotorController():
    def __init__(self, cfg: MotorControlConfig):
        self.param = cfg
        self.buffer = RingBuffer(50)

    def clamp(self, val, min_val, max_val):
        if val < min_val:
            return min_val
        elif val > max_val:
            return max_val
        else:
            return val

    def output(self,p_fb:float, v_fb:float, target_p:float, target_v:float, target_t:float):
        target_p = self.clamp(target_p, self.param.pmin, self.param.pmax)
        target_v = self.clamp(target_v, self.param.vmin, self.param.vmax)
        target_t = self.clamp(target_t, self.param.tmin, self.param.tmax)
        return (((target_p - p_fb) * self.param.kp + (target_v - v_fb) * self.param.kd) + target_t) * self.param.kt
    
    def get_compensation(self,curr_torque):
        self.buffer.append(curr_torque)
        return self.buffer.get_mean() / self.param.kt
    

    
