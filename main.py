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
    u_vec = [state.v for state in state_vec]

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

    # plt.subplots_adjust(bottom = 0.25)
    plt.show()




def main():

    #setup
    kp, ki, kd = 1, 0, 0     
    ti, tf, dt = 0, 5, 0.01 #s
    ref = 10
    state_vec = []
    ref_vec = []

    state = VehicleState(v=0, u=0, t=ti) 
    vehicle_config = VehicleConfig(dt = dt)
    model = Vehicle(vehicle_config)
    controller = PID(kp, ki, kd, dt)

    #loop
    state_vec, ref_vec = run(controller, model, state, ref, tf, dt)

    #plotting
    plot_response(state_vec, ref_vec)
    

if __name__ == "__main__":
    main()

