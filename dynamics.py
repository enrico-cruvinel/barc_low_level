from math import sqrt
from pytypes import VehicleConfig, VehicleState

class Vehicle():

    def __init__(self, vehicle_config = VehicleConfig):

        self.delay = vehicle_config.delay
        self.offset = vehicle_config.offset
        self.gain = vehicle_config.gain
        self.sat_poly_3 = vehicle_config.sat_poly_3
        self.sat_poly_5 = vehicle_config.sat_poly_5
        self.roll_res = vehicle_config.roll_res
        self.drag = vehicle_config.drag
        self.damping = vehicle_config.damping
        
        self.u_min = vehicle_config.u_min
        self.u_max = vehicle_config.u_max

        return

    def coerce_input_limits(self, u):

        return min(max(u, self.u_min), self.u_max)

    def step(self, u, v):

        smooth_sign = lambda x : x / sqrt(x**2 + 1e-6**2)
        
        a0 = (u - self.offset) * self.gain

        a =  a0 + a0**3*self.sat_poly_3 + a0**5 * self.sat_poly_5 \
                - self.roll_res * self.smooth_sign(v)\
                - self.drag * v * self.smooth_sign(v)\
                - self.damping * v

        return a