import numpy as np
import cvxpy as cp

from .tracker_utils import getRotationMatrix

from typing import Union

def generateCircTraj(x0:float, y0:float, theta0:float, 
                     radius:float, angle:float, CW:bool, 
                     max_speed:float, dt:float) -> np.ndarray:
    """
    generate circular trajectory
    """
    res = np.zeros((4, 100))

    s_max = angle * radius
    v_max = max_speed
    if v_max*dt > s_max:
        v_max = s_max/dt
    dtheta_max = v_max / radius

    t_mid = (s_max-(dt*v_max)) / v_max
    total_traj_time = 2*dt + t_mid
    rot_matrix = getRotationMatrix(theta0)
    t_arr = np.linspace(0, total_traj_time, 100)

    res[0, :] = t_arr

    theta_arr = np.zeros(t_arr.shape)
    for k in range(theta_arr.size):
        if t_arr[k] < dt:
            theta_arr[k] = dtheta_max/dt * 0.5 * t_arr[k]**2
        elif t_arr[k] >= dt and t_arr[k] < dt + t_mid:
            theta_arr[k] = dt * dtheta_max * 0.5 + (t_arr[k]-dt)*dtheta_max
        else:
            theta_arr[k] = dt * dtheta_max * 0.5 + t_mid*dtheta_max
            t_dummy = t_arr[k] - (t_mid + dt)
            theta_arr[k] += (dtheta_max + dtheta_max - (dtheta_max/dt)*t_dummy) * 0.5 * t_dummy

    for k in range(theta_arr.size):

        xk = radius * np.sin(theta_arr[k])
        if not CW:
            yk = radius * (1.0 - np.cos(theta_arr[k]))
            thetak = theta_arr[k]
        else:
            yk = -1.0 * radius * (1.0 - np.cos(theta_arr[k]))
            thetak = -1.0 * theta_arr[k]

        pose_vec = rot_matrix @ np.array([xk, yk])
        xpose = pose_vec[0] + x0
        ypose = pose_vec[1] + y0
        thetapose = thetak + theta0

        res[1, k] = xpose
        res[2, k] = ypose
        res[3, k] = thetapose
        
    return res

class trajectoryGenerator(object):
    def __init__(self, N:int, dt:float, max_speed:float) -> None:
        """
        TrajectoryGenerator object given the parameters, generate smooth trajectories using cvxpy
        """
        self.N = N
        self.dt = dt
        self.max_speed = max_speed

        self.pk_list = []
        self.vk_list = []
        self.ak_list = []

        self.setVariables()

        self.p0 = np.zeros((2, 1))
        self.pd = np.zeros((2, 1))
        self.theta0 = 0.
        self.thetad = 0.

        self.trajectory_set = False
        self.init_state_set = False
        self.des_state_set = False

        self.problem = None
        return
    
    def setVariables(self) -> None:
        """
        Set Decision variables lists
        """
        self.pk_list = [cp.Variable((2, 1)) for _ in range(self.N+1)]
        self.vk_list = [cp.Variable((2, 1)) for _ in range(self.N+1)]
        self.ak_list = [cp.Variable((2, 1)) for _ in range(self.N+1)]
        return
    
    def setP0(self, p0:np.ndarray) -> None:
        """
        Set Initial Position
        """
        if type(p0) == np.ndarray and p0.shape == (3,1):
            self.p0 = p0[0:2]
            self.theta0 = p0[2, 0]
            self.init_state_set = True
        else:
            print("initial state is not set. it should be a (3,1) numpy array")
        return
    

    def setPd(self, pd:np.ndarray) -> None:
        """
        Set Desired Position
        """
        if type(pd) == np.ndarray and pd.shape == (3, 1):
            self.pd = pd[0:2]
            self.thetad = pd[2, 0]
            self.des_state_set = True
        else:
            print("desired state is not set. it should be a (3,1) numpy array")
        return

    def setProblem(self, EPS_C:float=0.01) -> None:
        """
        Set Optimization problem
        """

        if self.init_state_set and self.des_state_set:
            pass
        else:
            print("Set both initial and desired state first")
            return
        obj_func = 0.
        constr = []

        MAX_SPEED = self.max_speed
        N = self.N
        DT = self.dt
        for k in range(N):
            ak = self.ak_list[k]
            akp1 = self.ak_list[k+1]
            vk = self.vk_list[k]
            vkp1 = self.vk_list[k+1]
            pk = self.pk_list[k]
            pkp1 = self.pk_list[k+1]

            obj_func += cp.norm(akp1 - ak, 2)**2
            constr.append(vkp1 - vk == (DT/2.0) * (akp1 + ak))
            constr.append(pkp1 - pk == DT*vk + ak*((DT**2)/2) + (akp1 - ak)*((DT**2)/6) )
            constr.append(cp.norm(vk, 2) <= MAX_SPEED)
        
        a0 = self.ak_list[0]
        aN = self.ak_list[-1]
        p0 = self.pk_list[0]
        pN = self.pk_list[-1]
        v0 = self.vk_list[0]
        vN = self.vk_list[-1]
        v1 = self.vk_list[1]
        vNm1 = self.vk_list[-1]

        # Initial and terminal orientation constraint
        alpha0  = np.array([[np.cos(self.theta0)], [self.theta0]])
        alphad = np.array([[np.cos(self.thetad)], [self.thetad]])
        # Helper variables for oriantation constraints
        c0 = cp.Variable(nonneg=True)
        cN = cp.Variable(nonneg=True)
        # constr.append(a0 == c0 * alpha0)
        # constr.append(self.ak_list[-1] == aNm1)
        # constr.append(vNm1 == - cN * alphad)
        # constr.append(c0 >= EPS_C)
        # constr.append(cN >= EPS_C)

        # Initial and desired position constraints
        constr.append(p0 == self.p0)
        constr.append(pN == self.pd)
        # Initial and desired speed constraints
        constr.append(v0 == np.zeros(v0.shape))
        constr.append(vN == np.zeros(vN.shape))
        constr.append(aN == 0.)
        
        # Objective definition
        obj = cp.Minimize(obj_func)

        # Problem definition
        self.problem = cp.Problem(obj, constr)
        return
    
    def solveProblem(self, solver:str="ECOS", verbose:bool=False) -> bool:
        """
        Solve optimization problem
        """
        if self.init_state_set and self.des_state_set:
            self.problem.solve(solver=solver, verbose=verbose)
            if self.problem.status == "optimal":
                self.trajectory_set = True
                self.init_state_set = False
                self.des_state_set = False
                return True
            else:
                print("optimization problem is not solved accurately..")
                return False
        return False
    
    def getTrajectory(self, t0:float=0.0) -> Union[np.ndarray, None]:
        """
        Get solved trajectory
        """
        if self.trajectory_set:
            times = np.linspace(t0, self.dt*self.N, self.N+1).reshape(1, self.N+1)
            ans = cp.vstack([
                cp.hstack(self.pk_list),
                cp.hstack(self.vk_list),
                cp.hstack(self.ak_list), 
                times
            ]).value
            return ans
        else:
            print("There is no valid trajectory!!1")
            return 


if __name__ == "__main__":
    pass

    trajgen = trajectoryGenerator(20, 0.5, 10)

    trajgen.setP0(np.array([[0.0], 
                            [0.0], 
                            [0.0]]))

    trajgen.setPd(np.array([[3.0],
                            [4.0], 
                            [-0.3*np.pi/2]]))
    
    trajgen.setProblem()

    trajgen.solveProblem(verbose=True)

    opt_traj = trajgen.getTrajectory()

    px = opt_traj[0, :]
    py = opt_traj[1, :]
    vx = opt_traj[2, :]
    vy = opt_traj[3, :]
    ax = opt_traj[4, :]
    ay = opt_traj[5, :]
    times = opt_traj[6, :]
    print("MAIN IS RUN!!")
    pass