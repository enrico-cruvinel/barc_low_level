from math import sqrt
from pytypes import VehicleConfig, VehicleState

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

        return

    def coerce_input_limits(self, state: VehicleState):

        state.u_a = min(max(state.u_a, self.u_min_a), self.u_max_a)
        state.u_y = min(max(state.u_y, self.u_min_y), self.u_max_y)
        
        return

    def accelerate(self, state: VehicleState):
        
        smooth_sign = lambda x : x / sqrt(x**2 + 1e-6**2)
        
        a0 = (state.u_a - self.offset) * self.gain

        state.a =  a0 + a0**3*self.sat_poly_3 + a0**5 * self.sat_poly_5 \
                - self.roll_res * smooth_sign(state.v)\
                - self.drag * state.v * smooth_sign(state.v)\
                - self.damping * state.v

      
        return

    def steer():
        


        return
        
    def step(self, state: VehicleState):
        
        v_prev = state.v
        self.coerce_input_limits(state)
        self.accelerate(state)
        state.v = v_prev + state.a * self.dt
        state.x = state.x + state.v * self.dt

        return