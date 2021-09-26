class PID():
    def __init__(self, kp, ki, kd, dt):
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
    
    def step(self, x):
        e = self.ref - x
        self.cum_e += e * self.dt
        der_e = (self.last_e - e) / self.dt
        
        u = self.kp * e + self.ki * self.cum_e + self.kd * der_e
        
        self.last_e = e
        return u