from pytypes import VehicleConfig, VehicleState
from collections import deque
import casadi as ca
import numpy as np

class Vehicle():

    def __init__(self, vehicle_config = VehicleConfig):


  #      5.79838341e-02  1.52031313e+03  7.15398572e-03  1.52720337e-01
  #      7.73102813e-06  4.90633228e-01  1.63971733e+00 -2.81657345e-01

        self.delay = vehicle_config.delay
        self.offset = vehicle_config.offset
        self.gain = vehicle_config.gain
        self.sat_poly_3 = vehicle_config.sat_poly_3
        self.sat_poly_5 = vehicle_config.sat_poly_5
        self.roll_res = vehicle_config.roll_res
        self.drag = vehicle_config.drag
        self.damping = vehicle_config.damping
        
        self.u_min_a = vehicle_config.u_min_a
        self.u_max_a = vehicle_config.u_max_a

        self.u_min_y = vehicle_config.u_min_y
        self.u_max_y = vehicle_config.u_max_y

        self.dt = vehicle_config.dt

        # buffer for delay in input 
        self.buf_length = int(vehicle_config.delay/vehicle_config.dt)+2
        self.t_buf = deque([-self.buf_length+k  for k in range(self.buf_length)])
        self.u_buf = deque([0]*self.buf_length)
        self.F = self.setup_acceleration()
        return

    def coerce_input_limits(self, state: VehicleState):

        state.u_a = min(max(state.u_a, self.u_min_a), self.u_max_a)
        state.u_y = min(max(state.u_y, self.u_min_y), self.u_max_y)
        
        return

    def setup_acceleration(self):

        t = ca.SX.sym('t')
        ti = ca.SX.sym('ti', self.buf_length)
        ui = ca.SX.sym('ui', self.buf_length+1)
        
        ut = ca.pw_const(t-self.delay, ti, ui)
        x = ca.SX.sym('x')
        v = ca.SX.sym('v')

        t_dot = 1
        x_dot = v

        smooth_sign = lambda x : x / ca.sqrt(x**2 + 1e-6**2)
            
        a0 = (ut - self.offset) * self.gain

        v_dot =  a0 + a0**3*self.sat_poly_3 + a0**5 * self.sat_poly_5 \
                - self.roll_res * smooth_sign(v)\
                - self.drag * v * smooth_sign(v)\
                - self.damping * v

        state = ca.vertcat(t,x,v)
        input = ca.vertcat(ti,ui)
        ode   = ca.vertcat(t_dot, x_dot, v_dot)
        
        prob    = {'x':state, 'p':input, 'ode':ode}
        setup   = {'t0':0, 'tf':self.dt}
        F = ca.integrator('int','idas',prob, setup) 
        
        return F


    def steer():
        
        return
        
    def step(self, state: VehicleState):
        
        self.coerce_input_limits(state)

        self.t_buf.append(state.t)
        self.u_buf.append(state.u_a)
        self.t_buf.popleft()
        self.u_buf.popleft()
        
        sol = np.array(self.F(np.array([state.t, state.x, state.v]), [*self.t_buf, *self.u_buf, 0], 0, 0, 0, 0)[0]).squeeze()
        state.t = sol[0]
        state.x = sol[1]
        state.v = sol[2]

        return
        