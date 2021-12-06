#include <stdio.h>



class PID{
    private: 

        double _kp, _ki, _kd ; 
        double _u, _u_max, _u_min;
        long _previousTime, _currentTime ; 
        double _last_e = 0, _cum_e = 0, _der_e = 0; 
        double _ref ; 

    public: 

    PID(double kp, double ki, double kd, double u_min, double u_max){
        
        _kp = kp ; _ki = ki ; _kd = kd ; 
        _u_min = u_min ; _u_max = u_max ; 
        _previousTime = micros() ; 
        _currentTime = micros() ; 

    }

    void set_ref(double ref){
        _ref = ref ; 
    }

    double anti_windup(double y){
        double new_e ; 
        double ur = _cum_e * _ki;
        double e = _ref - y;

        if ((ur > _u_max && _ki * e > 0) || (ur < _u_min && _ki * e < 0)){
                new_e = 0 ; 
        }
        else{
            new_e = e ;
        }
        return new_e;
    }

    double step(double y){
        double u ; 
        double e = _ref - y ; 
        double dt = micros() - _previousTime ;

        _cum_e += self.anti_windup(y) * dt ; 
        // _der_e = (_last_e - e) / dt ; 
        
        u = _kp * e + _ki * _cum_e ; // _kd * _der_e

        _last_e = e ;
        _previousTime = _currentTime ; 
        _currentTime = micros() ; 
        return u ; 
    }

}; 



int main(){


    return 0 ;
}