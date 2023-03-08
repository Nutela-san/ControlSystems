#ifndef _PIDCONTROL_H
#define _PIDCONTROL_H

#include <Arduino.h>

enum time_scale{
    MILISECONDS,
    MICROISECONDS
};

#define ulong unsigned long

class SimplePID
{
private:
    float r_int, r_dev, control_out, last_e;
    float rang_integral[2], rang_out[2];
    ulong last_t, current_t, min_step_t;
    bool en_out_limit = false, en_integral_limit = false;

    float scale_t;

    time_scale type_t;

    void doIntegral(float e, ulong inc_t);
    void doDerivate(float e, ulong inc_t);

public:
    float kp, ki, kd;

    SimplePID(){}

    void begin(time_scale time_unit = MICROISECONDS, ulong min_periodo = 1);

    void setGains(float kp, float ki, float kd);

    void setOutLimits(float r_min, float r_max);

    void setOutLimits(float r);

    void setIntegralLimits(float r_min, float r_max);

    void setIntegralLimits(float r);

    float calulate_out(float e);


    ~SimplePID(){}
};


#endif