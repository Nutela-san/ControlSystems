#include <PIDControl.h>

SimplePID control;  //crear objeto para manejar el control

float error = 0;

void setup(){

  /*
  Confuguracion de los demas modulos y pines
  */

  control.begin();  // inicializando controlador,
                    // configura por default la escala de tiempo en MILISECONDS y un tiempo minimo de 1ms

  control.setGains(1, 0, 0);  // configura las ganacias kp, ki y kd

  control.setOutLimits(255);  // activa y configura los limites de
                              // la salida del controlador, en este caso
                              // de forma simetrica (min_limit = -255 , max_limit = 255)
                              // la forma asimetrica es control.setOutLimits(min_limit, max_limit)
                              // selecionando de forma individual rl limite minimo y el maximo.

  control.setIntegralLimits(50);  // De igual forma que setOutLimits, activa y configura
                                  // los limites en para la variable que guarda la integral.
                                  // El metodo se puede usar de forma simetrica y asimetrica.
}

void loop(){
  /*
    Lectura del los sensores y comparacion con el setpoit para
    obtener el error, ejemplo:
    error =  setpoint - leerSensor();
  */

  float out_valor = control.calulate_out(error);

  /*
    Asignar el valor de la salida de controlador a los actualdores, ejemplo:
    move_motor(out_valor);
  */
}