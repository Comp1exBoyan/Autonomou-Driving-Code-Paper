#! /usr/bin/env python

# framework of steering control system
'''
 high-level controller --- model predictive control 
 
 
 low-level controller  --- pid/adrc 
 steering wheel angle  --- ddelta  ==> output of MPC  -- the desired steering angle of the vehicle
 : input -- ddelta output -- control value sent to the steering motor 


 This is a ros node which finally sends out the two control value :
                                                     -- steering motor CAN
													 -- back wheel motor CAN
 ''' 


import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import cvxpy
import math

# mpc parameters
NX = 3  # x = x, y, yaw
NU = 2  # u = [v,delta]
T = 8  # horizon length
R = np.diag([0.1, 0.1])  # input cost matrix
Rd = np.diag([0.1, 0.1])  # input difference cost matrix
Q = np.diag([1, 1, 1])  # state cost matrix
Qf = Q  # state final matrix
MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(45.0)  # maximum steering speed [rad/s]
MAX_VEL = 2.0




#车辆
dt = 0.1  # 时间间隔，单位：s
L = 2  # 车辆轴距，单位：m
v = 2  # 初始速度
x_0 = 0  # 初始x
y_0 = -3  # 初始y
psi_0 = 0  # 初始航向角

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

class vehicle_model():
	'''
		vehicle model
		def A,B,D

	'''
	def __init__(self, x, y, psi, v, L, dt):
		self.x = x
		self.y = y
		self.psi = psi
		self.v = v
		self.L = L
		# 实现是离散的模型
		self.dt = dt 


	def state_update(self, data):
		'''
		订阅节点数据，更新状态
		'''



		return self.x, self.y, self.psi, self.v

	def state_space(self, ref_delta, ref_yaw):
		"""将模型离散化后的状态空间表达

		Args:
			ref_delta (_type_): 参考的转角控制量
			ref_yaw (_type_): 参考的偏航角

		Returns:
			_type_: _description_
		"""

		A = np.matrix([
			[1.0, 0.0, -self.v*self.dt*math.sin(ref_yaw)],
			[0.0, 1.0, self.v*self.dt*math.cos(ref_yaw)],
			[0.0, 0.0, 1.0]])

		B = np.matrix([
			[self.dt*math.cos(ref_yaw), 0],
			[self.dt*math.sin(ref_yaw), 0],
			[self.dt*math.tan(ref_delta)/self.L, self.v*self.dt /(self.L*math.cos(ref_delta)*math.cos(ref_delta))]
		])

		C = np.eye(3)
		return A, B, C



	


def normalize_angle(angle):
	"""
		Normalize an angle to  [-pi, pi]
	"""
	if angle > np.pi:
		angle -= 2.0 * np.pi
	elif angle < -np.pi:
		angle += 2.0 * np.pi
	else:
		pass 
	return angle 

	
		
def linear_mpc_control(xref, x0, delta_ref, ugv):   # ugv -- car model
    """
    linear mpc control

    xref: reference point
    x0: initial state
    delta_ref: reference steer angle
    ugv:车辆对象
    returns: 最优的控制量和最优状态
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0  # 代价函数
    constraints = []  # 约束条件

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t]-delta_ref[:, t], R)   

        if t != 0:
            cost += cvxpy.quad_form(x[:, t] - xref[:, t], Q)

        A, B, C = ugv.state_space(delta_ref[1, t], xref[2, t])
        constraints += [x[:, t + 1]-xref[:, t+1] == A @
                        (x[:, t]-xref[:, t]) + B @ (u[:, t]-delta_ref[:, t])]


    cost += cvxpy.quad_form(x[:, T] - xref[:, T], Qf)

    constraints += [(x[:, 0]) == x0]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_VEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        opt_x = get_nparray_from_matrix(x.value[0, :])
        opt_y = get_nparray_from_matrix(x.value[1, :])
        opt_yaw = get_nparray_from_matrix(x.value[2, :])
        opt_v = get_nparray_from_matrix(u.value[0, :])
        opt_delta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        opt_v, opt_delta, opt_x, opt_y, opt_yaw = None, None, None, None, None,

    return opt_v, opt_delta, opt_x, opt_y, opt_yaw


def callback(data):
	rospy.loginfo("vehicle state x: %lf, y: %lf, yaw: %lf")

def state_listener():
	#rospy.Subscriber('state', state, callback) 此处需要先定义

	return 0



def control_publisher():


	return 0 


if __name__ == "__main__":
	rospy.init_node("control")
	


