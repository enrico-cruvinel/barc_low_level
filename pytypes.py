from dataclasses import dataclass, field

@dataclass
class VehicleState():
    
    v :float = field(default = 0) #vehicle speed 
    u :float = field(default = 0) #pwm input

@dataclass
class VehicleConfig():

    delay      :float = field(default = 0)
    offset     :float = field(default = 0)
    gain       :float = field(default = 0)
    sat_poly_3 :float = field(default = 0)
    sat_poly_5 :float = field(default = 0)
    roll_res   :float = field(default = 0)
    drag       :float = field(default = 0)
    damping    :float = field(default = 0)
    
    u_min      :float = field(default = 0)
    u_max      :float = field(default = 0)