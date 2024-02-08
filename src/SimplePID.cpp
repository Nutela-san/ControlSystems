#include "SimplePID.h"

void SimplePID::begin(time_scale time_unit, ulong min_periodo){
  last_e = 0;
  r_int = 0;
  control_out = 0;
  min_step_t = min_periodo;

  type_t = time_unit;

  switch (time_unit){
    case MILLISECONDS:{
      scale_t = 1.0f / 1000.0f;
      last_t = micros();
      break;
    }
    case MICROSECONDS:{
      scale_t = 1.0f / 1000000.0f;
      last_t = millis();
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
  range_out[0] = r_min;
  range_out[1] = r_max;
  en_out_limit = true;
}

void SimplePID::setOutLimits(float r){
  setOutLimits(-r, r);
}


float SimplePID::calulate_out(float e){
  switch (type_t){
    case MILLISECONDS:{
      current_t = millis();
      break;
    }
    case MICROSECONDS:{
      current_t = micros();
      break;
    }
  }

  ulong increse_t = current_t - last_t;
  if (increse_t >= min_step_t){

    r_prop = e * kp;
    r_dev = kd*(e - last_e) / ((float)increse_t * scale_t);
    r_int = ki*(r_int + e * (float)increse_t * scale_t);

    if(en_out_limit){
      (range_out[1] > r_prop) ? range_integral[1] = range_out[1] - r_prop : range_integral[1] = 0.0f;
      (range_out[0] < r_prop) ? range_integral[0] = range_out[0] - r_prop : range_integral[0] = 0.0f;
      r_int = constrain(r_int,range_integral[0],range_integral[1]);
    } 

    control_out = r_prop + r_int + r_dev;

    if (en_out_limit){
      control_out = constrain(control_out,range_out[0],range_out[1]);
    }
    last_t = current_t;
  }

  return control_out;
}

void SimplePID::reset(){
  r_int = 0.0f;
  control_out = 0.0f;
  switch (type_t){
    case MILLISECONDS:{
      last_t = millis();
      break;
    }
    case MICROSECONDS:{
      last_t = micros();
      break;
    }
  }
}