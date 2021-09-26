class PID():
    def __init__(self, kp = 1, ki = 0, kd = 0, dt = 0.01):
        self.ki = ki
        self.kd = kd
        self.kp = kp
        self.dt = dt
        
        self.last_e = 0
        self.cum_e = 0
        self.ref = 0
        return
        
    def set_ref(self, ref):
        self.ref = ref
        return
    
    def step(self, y):
        e = self.ref - y
        self.cum_e += e * self.dt
        der_e = (self.last_e - e) / self.dt
        
        u = self.kp * e + self.ki * self.cum_e + self.kd * der_e
        
        self.last_e = e
        return u