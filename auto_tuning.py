import sys
from math import sqrt
from time import sleep, time

import matplotlib.pyplot as plt
import numpy as np
from control import step_info
from matplotlib.widgets import Button, Slider

from controller import PID
from dynamics import Vehicle
from pytypes import VehicleConfig, VehicleState
from utils import run
from visualization import TurtleFig


def plot_response(state_vec, ref_vec):
    
    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]
    u_a_vec = [state.u_a for state in state_vec]

    plt.figure(figsize = (6,6))

    plt.subplot(2,1,1)
    plt.plot(t_vec,v_vec,'b')
    plt.plot(t_vec,ref_vec,'k--')
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Velocity','Set Point'],loc='best')
    plt.title('System Response')

    plt.subplot(2,1,2)
    plt.plot(t_vec,u_a_vec,'r')
    plt.ylabel('Input (PWM)')    
    plt.legend(['Input'])
    plt.xlabel('Time (sec)')

    plt.show()




def main():

    ######## setup #########
    tic = time()
    kp, ki, kd = 0, 0, 0  
    ti, tf, dt = 0, 10, 0.01 #s
    ref = 8
    v0 = 6
    u_min, u_max = 1500, 1800
    state_vec = []
    ref_vec = []
    vehicle_config = VehicleConfig(delay = 5.79838341e-02, offset = 1.52031313e+03, gain = 7.15398572e-03,\
                               sat_poly_3 = 1.63971733e+00, sat_poly_5 = -2.81657345e-01, roll_res = 1.52720337e-01,\
                               drag = 4.90633228e-01, damping =  7.73102813e-06, dt = dt)

    J_min = 200
    kp_opt, ki_opt = 0, 0

    ######## tuning #########
    n0, nf = 100, 1001
    for kp in range(n0, nf, 10):
        
        ### progress bar ###
        sys.stdout.write('\r')
        sys.stdout.write("[%-20s] %d%%" % ('='*(((kp-n0)*20)//(nf-n0-1)), (kp-n0) * 100 // (nf-n0-1) ))
        sys.stdout.flush()

        for ki in range(n0, nf, 10):
            
            state = VehicleState(v=v0, u_a=1500, t=ti) 
            model = Vehicle(vehicle_config)
            controller = PID(kp, ki, kd, dt, u_min=u_min, u_max=u_max)
            
            #loop
            state_vec, ref_vec = run(controller, model, state, ref, tf, dt)

            t_vec = [state.t for state in state_vec]
            v_vec = [state.v for state in state_vec]
            try: 
                S = step_info(v_vec, t_vec, yfinal=ref)
                # J = S['RiseTime']**2 + S['SettlingTime']**2 #check how my gain values change as I change the cost function
                J = S['RiseTime']**2 + S['SettlingTime']**2 + S['Overshoot']**2
            # J = J/sqrt(J)
            except:
                J = 200
            if J < J_min:
                J_min = J
                kp_opt, ki_opt = kp, ki
        
    print()
    #### plotting ####
    state = VehicleState(v=0, u_a=1500, t=ti) 
    model = Vehicle(vehicle_config)
    controller = PID(kp_opt, ki_opt, kd, dt)
    
    #loop
    state_vec, ref_vec = run(controller, model, state, ref, tf, dt)
    print(step_info(v_vec, t_vec, yfinal=ref))
    print('Ref: ', ref, 'Kp: ', kp_opt, 'Ki: ', ki_opt)
    # plot_response(state_vec, ref_vec)
    toc = time() - tic
    print('Elapsed Time: ', toc)
    

if __name__ == "__main__":
    main()

