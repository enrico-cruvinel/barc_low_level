from controller import PID
from dynamics import Vehicle
from visualization import TurtleFig
from pytypes import VehicleState, VehicleConfig
from utils import run
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button  


# Define initial parameters
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

t_vec = [state.t for state in state_vec]
v_vec = [state.v for state in state_vec]

# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
line, = plt.plot(t_vec, v_vec, lw=2)
plt.plot(t_vec,ref_vec,'k--')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (m/s)')
ax.relim()
ax.autoscale_view()

axcolor = 'lightgoldenrodyellow'
ax.margins(x=0)

# adjust the main plot to make room for the sliders
plt.subplots_adjust(bottom=0.3)

# Make a horizontal slider to control the frequency.
kpax = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
kp_slider = Slider(
    ax=kpax,
    label='Kp',
    valmin=0,
    valmax=100,
    valinit=kp,
)

# Make a horizontal slider to control the frequency.
kiax = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)
ki_slider = Slider(
    ax=kiax,
    label='Ki',
    valmin=0,
    valmax=100,
    valinit=ki,
)

kdax = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor=axcolor)
kd_slider = Slider(
    ax=kdax,
    label='Kd',
    valmin=0,
    valmax=100,
    valinit=kd,
)


# The function to be called anytime a slider's value changes
def update(val):
    
    state = VehicleState(v=0, u=0, t=ti) 
    vehicle_config = VehicleConfig(dt = dt)
    model = Vehicle(vehicle_config)
    controller = PID(kp_slider.val, ki_slider.val, kd_slider.val)

    state_vec, ref_vec = run(controller, model, state, ref, tf, dt)

    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]


    line.set_ydata(v_vec)
    line.set_xdata(t_vec)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()


# register the update function with each slider
kp_slider.on_changed(update)
ki_slider.on_changed(update)
kd_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = plt.axes([0.82, 0.05, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


def reset(event):
    kp_slider.reset()
    ki_slider.reset()
    kd_slider.reset()
button.on_clicked(reset)

plt.show()