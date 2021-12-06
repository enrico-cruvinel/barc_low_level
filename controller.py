from matplotlib.pyplot import xcorr


class PID():
    def __init__(self, kp = 1, ki = 0, kd = 0, dt = 0.01, u_min = -float('inf'), u_max = float('inf')):
        self.ki = ki
        self.kd = kd
        self.kp = kp
        self.dt = dt
        
        self.last_e = 0
        self.cum_e = 0
        self.ref = 0

        self.u_min = u_min 
        self.u_max = u_max
        return
        
    def set_ref(self, ref):
        self.ref = ref
        return
    
    def anti_windup(self, y):
        ur = self.cum_e * self.ki
        e = self.ref - y

        if (ur > self.u_max and self.ki * e > 0)\
            or (ur < self.u_min and self.ki * e < 0):
            new_e = 0
        else:
            new_e = e
    
        return new_e

    def step(self, y):
        e = self.ref - y
        self.cum_e += self.anti_windup(y) * self.dt
        der_e = (self.last_e - e) / self.dt
        
        u = self.kp * e + self.ki * self.cum_e + self.kd * der_e
        
        self.last_e = e
        return u

class GainSchedule(): 
    def __init__(self, kp = lambda x: 1, ki = lambda x: 0, kd = lambda x: 0, dt = 0.01, u_min = -float('inf'), u_max = float('inf')):
        self.ki = ki
        self.kd = kd
        self.kp = kp
        self.dt = dt
        
        self.last_e = 0
        self.cum_e = 0
        self.ref = 0

        self.u_min = u_min
        self.u_max = u_max
        return
        
    def set_ref(self, ref):
        self.ref = ref
        return
    
    def anti_windup(self, y):
        
        e = self.ref - y
        ki = self.ki(self.ref)
        # ki = self.ki(e)
        ur = self.cum_e * ki

        if (ur > self.u_max and ki * e > 0)\
            or (ur < self.u_min and ki * e < 0):
            new_e = 0
        else:
            new_e = e
    
        return new_e

    def step(self, y):
        e = self.ref - y
        self.cum_e += self.anti_windup(y) * self.dt
        der_e = (self.last_e - e) / self.dt
        
        a = self.kp(self.ref)
        b = self.ki(self.ref)
        c = self.kd(self.ref)
        
        u = self.kp(self.ref) * e + self.ki(self.ref) * self.cum_e + self.kd(self.ref) * der_e
        
        # u = self.kp(e) * e + self.ki(e) * self.cum_e + self.kd(e) * der_e
        
        self.last_e = e
        
        return u