from controller import PID
from dynamics import Vehicle
from visualization import TurtleFig
from pytypes import VehicleState, VehicleConfig
from utils import run
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button  

def plot_response(state_vec, ref_vec):
    
    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]
    u_vec = [state.u for state in state_vec]

    plt.figure(figsize = (6,6))

    plt.subplot(2,1,1)
    plt.plot(t_vec,v_vec,'b')
    plt.plot(t_vec,ref_vec,'k--')
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Velocity','Set Point'],loc='best')
    plt.title('System Response')

    plt.subplot(2,1,2)
    plt.plot(t_vec,u_vec,'r')
    plt.ylabel('Input')    
    plt.legend(['Input'])
    plt.xlabel('Time (sec)')

    plt.show()




def main():

    #setup
    kp, ki, kd = 100, 130, 10    
    ti, tf, dt = 0, 10, 0.01 #s
    ref = 5
    state_vec = []
    ref_vec = []

    state = VehicleState(v=0, u=1500, t=ti) 

    vehicle_config = VehicleConfig(delay = 5.79838341e-02, offset = 1.52031313e+03, gain = 7.15398572e-03,\
                               sat_poly_3 = 1.63971733e+00, sat_poly_5 = -2.81657345e-01, roll_res = 1.52720337e-01,\
                               drag = 4.90633228e-01, damping =  7.73102813e-06, dt = dt)

    model = Vehicle(vehicle_config)
    controller = PID(kp, ki, kd, dt)

    #loop
    state_vec, ref_vec = run(controller, model, state, ref, tf, dt)

    #plotting
    plot_response(state_vec, ref_vec)
    

if __name__ == "__main__":
    main()

