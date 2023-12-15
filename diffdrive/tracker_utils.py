from builtin_interfaces.msg import Time, Duration
import numpy as np

from typing import Union


class PDController(object):
    """
    Propotional-derivative controller object
    """
    def __init__(self, name:str="", kp:float=1.0, kd:float=0.1) -> None:
        self.name = name

        self.kp = kp
        self.kd = kd

        self.e_prev = None
        self.e_prev_time = Time()
        pass

    def update(self, e_new:float, e_new_time:Time) -> None:
        """
        Update e_new and time 
        """
        self.e_prev = e_new 
        self.e_prev_time = e_new_time
        return
    
    def getOutput(self, e_new:float, e_new_time:Time) -> float:
        res = 0.0
        res += self.kp * e_new
        if self.e_prev is not None:
            e_diff = e_new - self.e_prev
            t_diff_sec = e_new_time.sec - self.e_prev_time.sec
            t_diff_nanosec = e_new_time.nanosec - self.e_prev_time.nanosec
            t_diff = t_diff_sec + t_diff_nanosec/1.0e9
            if abs(t_diff) <= 1.0e-9:
                edot = 0.
            else:
                edot = e_diff/t_diff
            res += self.kd * edot
        self.update(e_new, e_new_time)
        return res
    

def compareTime(t1:Union[Time, Duration], t2:[Time, Duration]) -> bool:
    """
    Checks whether t1 is greater than or equal to t2
    """
    if t1.sec > t2.sec: 
        return True
    elif t1.sec < t2.sec: 
        return False
    else: 
        return t1.nanosec >= t2.nanosec

def timeDiff(t1:Time, t2:Time) -> Time:
    """
    Subtracts t2 from t1 if compareTime(t1, t2) is true and returns the 
    results, if not returns 0.
    """
    res = Time()

    if not compareTime(t1, t2):
        print("t2 is greater than t1!!")
        return res
    
    sec_diff = t1.sec - t2.sec
    nanosec_diff = t1.nanosec - t2.nanosec
    if nanosec_diff<0:
        sec_diff -= 1
        nanosec_diff += int(1e9)
    res.sec = int(sec_diff)
    res.nanosec = int(nanosec_diff)
    return res

def timeToFloat(t:Union[Time, Duration]) -> float:
    """
    Convert a time message to a float.
    """
    sec = t.sec
    nanosec = t.nanosec
    res = sec + nanosec/1.0e9
    return res

def FloatToTime(t:float) -> Time:
    """
    Convert a float into time.
    """
    res = Time()
    res.sec = int(np.floor(t))
    res.nanosec = int(1.0e9 * (t-res.sec))
    return res

def getRotationMatrix(theta:float) -> np.ndarray:
    """
    Get rotation matrix from angle theta. 
    Theta should be radians not degree!!
    """
    res = np.eye(2) * 0.0
    res[0, 0] = np.cos(theta)
    res[0, 1] = -np.sin(theta)
    res[1, 0] = np.sin(theta)
    res[1, 1] = np.cos(theta)
    return res