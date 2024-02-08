#ifndef _SIMPLEPID_H_
#define _SIMPLEPID_H_

#include <Arduino.h>

enum time_scale{
    MILLISECONDS,
    MICROSECONDS
};

#define ulong unsigned long

class SimplePID
{
private:
    float r_prop = 0, r_int = 0, r_dev = 0, control_out = 0, last_e = 0;
    float range_integral[2] = {0}, range_out[2] = {0};
    ulong last_t = 0, current_t = 0, min_step_t = 10;
    bool en_out_limit = false;

    float scale_t = 0.001f;

    time_scale type_t = time_scale::MILLISECONDS;

public:
    float kp = 1.0f, ki = 0.0f, kd = 0.0f;

    SimplePID(){}

    void begin(time_scale time_unit = MILLISECONDS, ulong min_periodo = 10);

    void setGains(float kp, float ki, float kd);

    void setOutLimits(float r_min, float r_max);

    void setOutLimits(float r);

    float calulate_out(float e);

    void reset();


    ~SimplePID(){}
};


#endif