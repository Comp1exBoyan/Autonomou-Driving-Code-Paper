"""
Path tracking simulation with LQR steering control and PID speed control.

author : Yifeng Tang  
Ref : Atsushi Sakai (@Atsushi_twi)
"""
from functools import partial
import sys
from typing import Sequence
from numpy.lib.function_base import select

from scipy.sparse.construct import random
sys.path.append("D:\Programming_Station\python_station")  # file path
import cubic_spline
import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.linalg as la


##########################Original Q and R###########################
Q = np.eye(4)
R = np.eye(1)
 
##################Local Searching——GA###########################

J = 0  #Optimization Function
pop_size        = 50
iterations      = 1
chrome_length   = 10 
Prcross         = 0.75
Prmutation      = 0.05
EQ_pop = [[] for i in range(4)]
DQ_pop = [[] for i in range(4)]
fitness         = [] 
draw_e = []
draw_eth = []

def encoder():                  #encoder
    for k in range(4):          #init the population
        for i in range(pop_size):
            individual = []
            for j in range(chrome_length):
                individual.append(np.random.randint(0, 2))
            EQ_pop[k].append(individual) # encoded Q

def decoder():                  #decoder 
    for k in range(4):          
        for i in range(pop_size):
            tmp = 0 
            for j in range(chrome_length):
                tmp += EQ_pop[k][i][j] * (2**(10 - 1 - j))
            DQ_pop[k].append(1+ (tmp * 10 / (2 ** (10) - 1)))

def fitness_function(Q, x, u):
    x = np.array(x)
    u = np.array(u)
    xT = x.reshape(-1, 1)
    uT = u.reshape(-1, 1)
    partial_Q = 0.0
    partial_R = 0.0
    for i in range(4):
        partial_Q += x[i]*Q[i][i]*x[i]
        partial_R += u*u
    dJ= partial_Q + partial_R
    return -dJ


def choicebest(fitness_sequence):
    max_f = fitness[0]
    idx   = 0
    for i in range(pop_size):
        if fitness_sequence[i] > max_f:
            max_f = fitness_sequence[i]
            idx   = i
    return idx, max_f


def crossover():
    for k in range(4):
        for i in range(0, pop_size - 1, 2):
            cross_point  = np.random.randint(0, chrome_length - 1)
            tmp1 = [] 
            tmp2 = []
            tmp1.extend(EQ_pop[k][i][0:cross_point])
            tmp1.extend(EQ_pop[k][i+1][cross_point:])
            tmp2.extend(EQ_pop[k][i+1][0:cross_point])
            tmp2.extend(EQ_pop[k][i][cross_point:])
            EQ_pop[k][i] = tmp1
            EQ_pop[k][i+1] = tmp2   

def mutation():
    for k in range(4):
        for i in range(pop_size):
            if np.random.random() < Prmutation:
                mutation_point  = np.random.randint(0, chrome_length - 1)
                if EQ_pop[k][i][mutation_point] == 0:
                    EQ_pop[k][i][mutation_point] = 1
                else:
                    EQ_pop[k][i][mutation_point] = 0

def roulette_select():    #Roulette Selects 
        proportion = []
        total_fitneess = 0 
        for i in range(pop_size):
            total_fitneess += fitness[i]
        
        for i in range(pop_size):
            proportion.append(fitness[i] / total_fitneess)
        pie_fitness = []
        arrangement = 0.0
        for i in range(pop_size):
            pie_fitness.append(arrangement + proportion[i])
            arrangement += fitness[i]
        pie_fitness[-1]=1
        selection_point = []
        for i in range(pop_size):
            selection_point.append(np.random.random())
        selection_point.sort()
        
        for k in range(4):
            new_pop = []
            random_idx = 0 
            global EQ_pop
            for i in range(pop_size):
                while random_idx < pop_size and  selection_point[random_idx] < pie_fitness[i]:
                    new_pop.append(EQ_pop[k][i])
                    random_idx += 1
            EQ_pop[k] = new_pop

                


#####################################################################
Kp = 1.0  # speed proportional gain

# LQR parameter


# parameters
dt = 0.1  # time tick[s]
L = 0.5  # Wheel base of the vehicle [m]
max_steer = math.radians(45.0)  # maximum steering angle[rad]

show_animation = True
#  show_animation = False


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle): # the unit of angle is in rad;
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T * X * A - A.T * X * B * \
            la.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            X = Xn
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.matrix(la.inv(B.T * X * B + R) * (B.T * X * A))

    eigVals, eigVecs = la.eig(A - B * K)

    return K, X, eigVals


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)   # find the next point
    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])
    #############################init the system model############################
    A = np.matrix(np.zeros((4, 4)))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    # print(A)

    B = np.matrix(np.zeros((4, 1)))
    B[3, 0] = v / L
    ######################################################################
    K, _, _ = dlqr(A, B, Q, R)  # Obtaining the feedback parameters

    x = np.matrix(np.zeros((4, 1)))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    u = -K * x
    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K * x)[0, 0])

    delta = 2*ff + 1 * fb  

    return delta, ind, e, th_e, u, x




def movement_task(cx, cy, cyaw, ck, speed_profile, goal):
    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)
    reg_state = state
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = []
    target_ind = calc_nearest_index(state, cx, cy, cyaw)  #find the next point
    e, e_th = 0.0, 0.0
    encoder()
    while T >= time:
        for i in range(pop_size): # optimize Q in each step
            e, e_th = 0.0, 0.0 # reset the temporal errors
            decoder() #decoder
            Q[0][0] = DQ_pop[0][i]
            Q[1][1] = DQ_pop[1][i]
            Q[2][2] = DQ_pop[2][i]
            Q[3][3] = DQ_pop[3][i]
            state   = reg_state
            
            _, _, _, _, u, state_x = lqr_steering_control(state, cx, cy, cyaw, ck, e, e_th)
            # fitness calculation
            fitness.append(fitness_function(Q, state_x, u))
        best_idx, max_fitness = choicebest(fitness)
        state = reg_state # choose the state with the highest fitness and take action
        dl, target_ind, e, e_th, _, _ = lqr_steering_control(state, cx, cy, cyaw, ck, e, e_th)
        draw_e.append(e)
        draw_eth.append(e_th)
        fintess = []

        ############update the population#########
        roulette_select()
        crossover()
        mutation()
        ##################################
        dl, target_ind, e, e_th, _, _ = lqr_steering_control(state, cx, cy, cyaw, ck, e, e_th)
        ai = PIDControl(speed_profile[target_ind], state.v)
        state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        draw_e.append(e)
        draw_eth.append(e_th)
        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2)) +
                    ",target index:" + str(target_ind))
            plt.pause(0.0001)
            

    return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    #  flg, ax = plt.subplots(1)
    #  plt.plot(speed_profile, "-r")
    #  plt.show()

    return speed_profile


def main():

    # trajectory 
    print("LQR steering control tracking start!!")
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]
    target_speed = 12.0 / 3.6  # simulation parameter km/h -> m/s
    cx, cy, cyaw, ck, s = cubic_spline.calc_spline_course(ax, ay, ds=0.1)   #gengerate
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
    #movement and draw the cubic spline
    for iter in range(iterations):
        t, x, y, yaw, v = movement_task(cx, cy, cyaw, ck, sp, goal)
        if  show_animation:
            plt.close()
            flg, _ = plt.subplots(1)
            plt.plot(ax, ay, "xb", label="input")
            plt.plot(cx, cy, "-r", label="spline")
            plt.plot(x, y, "-g", label="tracking")
            plt.grid(True)
            plt.axis("equal")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.legend()

            flg,_ = plt.subplots(1)
            plt.plot(t, draw_e, "-b", label="dis_error")
            plt.plot(t, draw_eth, "-r", label="theta_error")
            plt.grid(True)
            plt.xlabel("x")
            plt.ylabel("error")
            plt.legend()
            

            plt.show()


if __name__ == '__main__':
    main()