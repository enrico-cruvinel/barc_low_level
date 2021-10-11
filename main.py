import matplotlib.pyplot as plt
import numpy as np
from control import step_info
from matplotlib.widgets import Button, Slider

from controller import PID
from dynamics import Vehicle
from pytypes import VehicleConfig, VehicleState
from utils import run, plot_response
from visualization import TurtleFig





def main():

    #setup
    kp, ki, kd = 270, 140, 0    
    ti, tf, dt = 0, 10, 0.01 #s
    ref = 10
    state_vec = []
    ref_vec = []

    state = VehicleState(v=0, u_a=1500, t=ti) 

    vehicle_config = VehicleConfig(delay = 5.79838341e-02, offset = 1.52031313e+03, gain = 7.15398572e-03,\
                               sat_poly_3 = 1.63971733e+00, sat_poly_5 = -2.81657345e-01, roll_res = 1.52720337e-01,\
                               drag = 4.90633228e-01, damping =  7.73102813e-06, dt = dt)

    model = Vehicle(vehicle_config)
    controller = PID(kp, ki, kd, dt)

    #loop
    state_vec, ref_vec = run(controller, model, state, ref, tf, dt)

    #plotting
    plot_response(state_vec, ref_vec)
    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]
    print(step_info(v_vec, t_vec, yfinal=ref))

if __name__ == "__main__":
    main()

