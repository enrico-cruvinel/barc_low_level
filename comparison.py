import re
import matplotlib.pyplot as plt
import numpy as np
from control import step_info
from matplotlib.widgets import Button, Slider

from controller import PID, GainSchedule
from dynamics import Vehicle
from pytypes import VehicleConfig, VehicleState
from utils import run_var_ref, run, plot_response


######### Define initial parameters ##########
# ki = lambda v: -2.682*v**4 + 71.15*v**3 + -651.8*v**2 + 2240*v - 1410
# kp = lambda v: -0.02604*v**4 + 4.062*v**3 + -69.9*v**2 + 368.7*v - 230
ki = lambda v: 990 if (v<3) else 140
kp = lambda v: 260 if (v<3) else 270


ti, tf, dt = 0, 20, 0.01 #s
t = np.arange(ti, tf+dt, dt)
v0 = 0
vref = 10
state_vec = []
vref_vec = []

vehicle_config = VehicleConfig(delay = 5.79838341e-02, offset = 1.52031313e+03, gain = 7.15398572e-03,\
                            sat_poly_3 = 1.63971733e+00, sat_poly_5 = -2.81657345e-01, roll_res = 1.52720337e-01,\
                            drag = 4.90633228e-01, damping =  7.73102813e-06, dt = dt)
model = Vehicle(vehicle_config)


state_1 = VehicleState(v=v0, u_a=1500, t=ti)
state_2 = VehicleState(v=v0, u_a=1500, t=ti)
state_3 = VehicleState(v=v0, u_a=1500, t=ti) 

# kp, ki, kd = 380, 990, 0  
controller_schedule = GainSchedule(kp=kp, ki=ki, dt=dt, u_min=1500, u_max=1750)
controller_pid_10 = PID(270, 140, 0, dt, u_min=1500, u_max=1750) #500 #100
controller_pid_2 = PID(260, 990, 0, dt, u_min=1500, u_max=1750)
# controller_pid_6 = PID(310, 460, 0, dt, u_min=1500, u_max=1700)


# controller_schedule = GainSchedule(kp=kp, ki=ki, dt=dt)
# controller_pid_10 = PID(270, 140, 0, dt)
# controller_pid_2 = PID(260, 990, 0, dt)

######### ref signals #########
# step 1
step_ref_2 = 2

# step 2
step_ref_4 = 4

# step 3
step_ref_6 = 6

# step 4
step_ref_8 = 8

# step 5
step_ref_10 = 10


# ramp
ramp_ref = t * 10/tf + 2


# parabolic
parabolic_ref = t ** 2 / tf


# sine
sine_ref = np.sin(t / (np.pi)) * 5 + 5


# square signal 1 -> ramp up
n = len(t) // 5
sq_ref_1 = np.concatenate((np.ones((n,)) * step_ref_2, np.ones((n,)) * step_ref_4, np.ones((n,)) * step_ref_6,\
                          np.ones((n,)) * step_ref_8, np.ones((len(t) - n*4,)) * step_ref_10))

# square signal 1 -> ramp up
# n = len(t) // 4
# sq_ref_11 = np.concatenate((np.ones((n,)) * 4, np.ones((n,)) * 6, np.ones((n,)) * 7,\
                        #   np.ones((len(t) - n*3,)) * 9))

# square signal 2 -> ramp down
sq_ref_2 = np.concatenate((np.ones((n,)) * step_ref_10, np.ones((n,)) * step_ref_8, np.ones((n,)) * step_ref_6,\
                          np.ones((n,)) * step_ref_4, np.ones((len(t) - n*4,)) * step_ref_2))

# square signal 3 -> ramp up and then down
sq_ref_3 = np.concatenate((sq_ref_1[1:-1:2], sq_ref_1[-1:1:-2], np.ones(len(t) - 5*n) * step_ref_2)) 

# square signal 


######### controllers #########
# ref = 7
# state_vec_1, ref_vec_1 = run(controller_pid_2, model, state_1, ref, tf, dt)
# state_vec_2, ref_vec_2 = run(controller_pid_10, model, state_2, ref, tf, dt)
# state_vec_3, ref_vec_3 = run(controller_schedule, model, state_3, ref, tf, dt)

# ref = [step_ref_2] * len(t)
ref = ramp_ref

state_vec_1, ref_vec_1, u_vec_1 = run_var_ref(controller_pid_2, model, state_1, ref, tf, dt)
state_vec_2, ref_vec_2, u_vec_2 = run_var_ref(controller_pid_10, model, state_2, ref, tf, dt)
state_vec_3, ref_vec_3, u_vec_3 = run_var_ref(controller_schedule, model, state_3, ref, tf, dt)

######### plotting ########
t1 = [state.t for state in state_vec_1]
v1 = [state.v for state in state_vec_1]
ua1 = [state.u_a for state in state_vec_1]

t2 = [state.t for state in state_vec_2]
v2 = [state.v for state in state_vec_2]
ua2 = [state.u_a for state in state_vec_2]

t3 = [state.t for state in state_vec_3]
v3 = [state.v for state in state_vec_3]
ua3 = [state.u_a for state in state_vec_3]

######## plot ############
plt.figure(figsize = (6,6))

plt.subplot(2,1,1)
plt.plot(t1,v1)
plt.plot(t2,v2)
plt.plot(t3,v3)
plt.plot(t1,ref_vec_1,'k--')
plt.ylabel('Velocity (m/s)')
plt.legend(['PID @ 2m/s', 'PID @ 10m/s', 'Gain Scheduling', 'Set Point'],loc='best')
plt.title('System Response')

plt.subplot(2,1,2)
plt.plot(t1, ua1)
plt.plot(t2, ua2)
plt.plot(t3, ua3)
plt.ylabel('Input (PWM)')    
plt.legend(['PID @ 2m/s', 'PID @ 10m/s', 'Gain Scheduling'],loc='best')
plt.xlabel('Time (sec)')

######## plot ############
# plt.figure(figsize = (6,6))

# plt.subplot(2,1,1)
# plt.plot(t1,v1)
# # plt.plot(t2,v2)
# # plt.plot(t3,v3)
# plt.plot(t2,v2)
# plt.plot(t1,ref_vec_1,'k--')
# plt.ylabel('Velocity (m/s)')
# plt.legend(['Response Windup', 'Response Antiwindup', 'Set Point'],loc='best')
# plt.title('System Response')

# plt.subplot(2,1,2)
# plt.plot(t1, u_vec_1)
# plt.plot(t2, u_vec_2)
# plt.plot(t1, ua1)
# # plt.plot(t2, u_vec_2)
# # plt.plot(t2, ua2)
# # plt.plot(t3, u_vec_3)
# # plt.plot(t3, ua3)
# plt.ylabel('Input (PWM)')    
# plt.legend(['Controller Output Windup', 'Controller Output Antiwindup', 'System Input'],loc='best')
# plt.xlabel('Time (sec)')


plt.show()
