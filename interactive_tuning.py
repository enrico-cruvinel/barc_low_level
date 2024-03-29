import control as ct
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider

from controller import PID
from dynamics import Vehicle
from pytypes import VehicleConfig, VehicleState
from utils import run
from visualization import TurtleFig

######### Define initial parameters ##########
kp, ki, kd = 260, 1000, 0     
ti, tf, dt = 0, 10, 0.01 #s
v0 = 0
vref = 6
u_min, u_max = 1500, 1800
state_vec = []
vref_vec = []

# values from Thomas' data
# 5.79838341e-02  1.52031313e+03  7.15398572e-03  1.52720337e-01
# 7.73102813e-06  4.90633228e-01  1.63971733e+00 -2.81657345e-01
  
state = VehicleState(v=v0, u_a=1500, t=ti) 

vehicle_config = VehicleConfig(delay = 5.79838341e-02, offset = 1.52031313e+03, gain = 7.15398572e-03,\
                               sat_poly_3 = 1.63971733e+00, sat_poly_5 = -2.81657345e-01, roll_res = 1.52720337e-01,\
                               drag = 4.90633228e-01, damping =  7.73102813e-06, dt = dt)

model = Vehicle(vehicle_config)
# controller = PID(kp, ki, kd, dt)
controller = PID(kp, ki, kd, dt, u_min=u_min, u_max=u_max)

######### run simulation ##########
state_vec, vref_vec = run(controller, model, state, vref, tf, dt)

t_vec = [state.t for state in state_vec]
v_vec = [state.v for state in state_vec]
u_a_vec = [state.u_a for state in state_vec]

S = ct.step_info(v_vec, t_vec, vref)

print('RiseTime, ', S['RiseTime'])
print('SettlingTime, ', S['SettlingTime'])
print('Overshoot, ', S['Overshoot'])
print()

############ plotting #############
fig, ax = plt.subplots(nrows=2,ncols=1, sharex=True)
vax, uax = ax[0], ax[1] 
vline, = vax.plot(t_vec, v_vec, lw=2)
vrefline, = vax.plot(t_vec,vref_vec,'k--')
uline, = uax.plot(t_vec, u_a_vec)
uax.set_xlabel('Time (s)')
vax.set_ylabel('Speed (m/s)')
uax.set_ylabel('Input (PWM)')
vax.relim()
vax.autoscale_view()
uax.relim()
uax.autoscale_view()

############ update plot #############
# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.25, bottom=0.3)

axcolor = 'lightgoldenrodyellow'
vrefax = plt.axes([0.1, 0.25, 0.0225, 0.63], facecolor=axcolor)
vref_slider = Slider(
    ax=vrefax,
    label='vref',
    valmin=0,
    valmax=13,
    valinit=vref,
    orientation='vertical'
)

kpax = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
kp_slider = Slider(
    ax=kpax,
    label='Kp',
    valmin=0,
    valmax=1000,
    valinit=kp,
)

kiax = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)
ki_slider = Slider(
    ax=kiax,
    label='Ki',
    valmin=0,
    valmax=1000,
    valinit=ki,
)

kdax = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor=axcolor)
kd_slider = Slider(
    ax=kdax,
    label='Kd',
    valmin=0,
    valmax=1000,
    valinit=kd,
)


# function to be called anytime a slider's value changes
def update(val):

    state = VehicleState(v=v0, u_a=0, t=ti) 
    model = Vehicle(vehicle_config)
    controller = PID(kp_slider.val, ki_slider.val, kd_slider.val, u_min=u_min, u_max=u_max)

    state_vec, vref_vec = run(controller, model, state, vref_slider.val, tf, dt)

    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]
    u_a_vec = [state.u_a for state in state_vec]
    
    S = ct.step_info(v_vec, t_vec, vref)
    print('RiseTime, ', S['RiseTime'])
    print('SettlingTime, ', S['SettlingTime'])
    print('Overshoot, ', S['Overshoot'])
    print()

    # need to use set_data to change both x and y
    # in case array size changes from previous iterations
    vline.set_data(t_vec, v_vec)
    uline.set_data(t_vec, u_a_vec)
    vrefline.set_data(t_vec, vref_vec)
    # vline.set_ydata()
    vax.relim()
    vax.autoscale_view()
    uax.relim()
    uax.autoscale_view()
    fig.canvas.draw_idle()


# register the update function with each slider
kp_slider.on_changed(update)
ki_slider.on_changed(update)
kd_slider.on_changed(update)
vref_slider.on_changed(update)

# Button to reset the sliders to initial values
resetax = plt.axes([0.82, 0.05, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')

def reset(event):
    kp_slider.reset()
    ki_slider.reset()
    kd_slider.reset()
    vref_slider.reset()
button.on_clicked(reset)

plt.show()
