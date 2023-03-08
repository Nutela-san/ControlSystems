#include <Arduino.h>
#include <PIDControl.h>

SimplePID control;

float error = 0 ;

void setup() {
  control.begin();
  control.setGains(1,0,0);
  control.setOutLimits(255);
  control.setIntegralLimits(50);

}

void loop() {
  control.calulate_out(error);
}