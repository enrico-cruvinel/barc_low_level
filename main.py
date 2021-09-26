from controller import PID
from dynamics import Vehicle
from visualization import TurtleFig
from pytypes import VehicleState, VehicleConfig
import numpy as np
import matplotlib.pyplot as plt

def plot_response(state_vec, ref_vec):
    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]
    u_vec = [state.v for state in state_vec]

    plt.figure(1,figsize=(5,4))
    plt.subplot(2,1,1)
    plt.plot(t_vec,v_vec,'b-',linewidth=3)
    plt.plot(t_vec,ref_vec,'k',linewidth=2)
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Velocity','Set Point'],loc=2)
    plt.subplot(2,1,2)
    plt.plot(t_vec,u_vec,'r',linewidth=3)
    plt.ylabel('Input')    
    plt.legend(['Input (%)'])
    plt.xlabel('Time (sec)')
    plt.show()



def main():
    #setup
    kp, ki, kd = 1, 1, 1     
    
    ti, tf, dt = 0, 60, 0.01 #s
    # steps = (tf-ti)/dt 
    # t_vec = np.linspace(ti, tf, steps)
    state_vec = []
    ref_vec = []

    state = VehicleState(v=0, u=0, t=ti) 
    vehicle_config = VehicleConfig()
    model = Vehicle(vehicle_config)
    controller = PID(kp, ki, kd, dt)

    #loop
    while state.t < tf:
        state_vec.append(state.copy())
        ref_vec.append(controller.ref)
        controller.set_ref(10)
        state.u = controller.step(state.v)
        model.step(state)
        state.t += dt

    plot_response(state_vec, ref_vec)


if __name__ == "__main__":
    main()

