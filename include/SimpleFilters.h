#ifndef _SIMPLEFILTERS_H_
#define _SIMPLEFILTERS_H_

#include <Arduino.h>

enum class Filter_t{
  LPF = false,
  HPF = true
};

template <typename data_t, size_t buffer_size = 10>
class MA_Filter{
  private:
    data_t samples_buffer[buffer_size];
    size_t currentIndex; // Índice de asignación actual en el buffer

    void addData(data_t new_sample){
      samples_buffer[currentIndex] = new_sample; // Asigna el nuevo dato en la posición actual
      currentIndex = (currentIndex + 1) % buffer_size; // Actualiza el índice circularmente
    }

  public:
    explicit MA_Filter(){ //costructor default
      currentIndex = 0;
    }

    data_t doFilter(data_t new_sample , Filter_t filter_type = Filter_t::LPF){
      addData(new_sample);
      data_t sum = 0;
      for (size_t i=0;i<buffer_size; i++) {
        sum += samples_buffer[i];
      }
      data_t prom = sum / (data_t)buffer_size;
      if((bool)filter_type) prom = new_sample - prom;
      return prom; // Calcula el promedio
    }

    void reset(){
      for (size_t i=0;i<buffer_size; i++) {
        samples_buffer[i] = 0;
      }
    }
};

template <typename data_t>
class EMA_Filter{
  private:
    data_t last_value = 0;
  public:
    float alpha = 1;

    explicit EMA_Filter(){}

    explicit EMA_Filter(float alpha){
      setAlpha(alpha);
    }
    
    void setAlpha(float alpha){
      this->alpha = constrain(alpha,0.0f,1.0f);
    }

    data_t doFilter(data_t new_sample , Filter_t filter_type = Filter_t::LPF){
      data_t value = alpha*new_sample + (1-alpha)*last_value;
      last_value = value;
      if ((bool)filter_type) value = new_sample-value;
      return(value);
    }

    void reset(){
      last_value = 0;
    }
};

template <typename data_t>
class RC_FirstO_Filter{
  private:
    float k[2] = {1.0f,0.0f}; //gains first order filter
    data_t last_value = 0;
    float Ts = 1;
    float f_cutout = 1;

    void update_Ks(){
      float RC = 1.0f/(6.28318f * f_cutout);
      k[0] = Ts/(Ts + RC);
      k[1] = RC/(Ts + RC);
    }

  public:
    explicit RC_FirstO_Filter(){}

    explicit RC_FirstO_Filter(float f_cutout, float Ts){
      this->Ts = Ts;
      this->f_cutout = f_cutout;
      update_Ks();
    }

    void setFreqCutout(float f_cutout){
      this->f_cutout = f_cutout;
      update_Ks();
    }

    void setSampleTime(float Ts){
      this->Ts = Ts;
      update_Ks();
    }

    data_t doFilter(data_t new_sample , Filter_t filter_type = Filter_t::LPF){
      data_t value = (data_t)(k[0]*(float)new_sample + k[1]*(float)last_value);
      last_value = value;
      if ((bool)filter_type) value = new_sample-value;
      return(value);
    }

    void reset(){
      last_value = 0;
    }
};

#endif