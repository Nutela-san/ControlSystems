#include "PIDControl.h"

void SimplePID::begin(time_scale time_unit = MILISECONDS, ulong min_periodo = 1){
    last_t = millis();
    last_e = 0;
    r_int = 0;
    control_out = 0;
    min_step_t = min_periodo;

    type_t = time_unit;

    switch(time_unit){
        case MILISECONDS:{
            scale_t = 1.0/1000.0;
            break;
        }
        case MICROISECONDS:{
            scale_t = 1.0/1000.0 ;
            break;
        }
    }
}

void SimplePID::setGains(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void SimplePID::setOutLimits(float r_min, float r_max){
    rang_out[0] = r_min;
    rang_out[1] = r_max;
    en_integral_limit = true;
}

void SimplePID::setOutLimits(float r){
    setOutLimits(-r,r);
}

void SimplePID::setIntegralLimits(float r_min, float r_max){
    rang_out[0] = r_min;
    rang_out[1] = r_max;
    en_integral_limit = true;
}

void SimplePID::setIntegralLimits(float r){
    setIntegralLimits(-r,r);
}

void SimplePID::doDerivate(float e, ulong inc_t){
    r_dev = (e - last_e)/((float)inc_t *scale_t);
}

void SimplePID::doIntegral(float e, ulong inc_t){

    r_int = r_int + e*(float)inc_t*scale_t;

    if(en_integral_limit){
        if(r_int > rang_integral[1]){
            r_int = rang_integral[1];
        }
        else if(r_int < rang_integral[0]){
            r_int = rang_integral[0];
        }
    }
}

float SimplePID::calulate_out(float e){
    switch(type_t){
        case MILISECONDS:{
            current_t = millis();
            break;
        }
        case MICROISECONDS:{
            current_t = micros();
            break;
        }
    }

    ulong increse_t = current_t - last_t;
    if(increse_t >= min_step_t){
        
        doDerivate(e,increse_t);
        doIntegral(e,increse_t);

        control_out = e*kp + r_int*ki + r_dev*kd;

        if(en_out_limit){
            if(control_out > rang_out[1]){
                control_out = rang_out[1];
            }
            else if(control_out < rang_out[0]){
                control_out = rang_out[0];
            }
        }

    }
    
    return control_out;
}

