#include <SimplePID.h>

SimplePID control;  //crear objeto para manejar el control

float error = 0;

void setup(){

  /*
  Confuguracion de los demas modulos y pines
  */

  control.setGains(1, 0, 0);  // configura las ganacias kp, ki y kd

  control.setOutLimits(255);  // activa y configura los limites de
                              // la salida del controlador, en este caso
                              // de forma simetrica (min_limit = -255 , max_limit = 255)
                              // la forma asimetrica es control.setOutLimits(min_limit, max_limit)
                              // selecionando de forma individual rl limite minimo y el maximo.

  control.begin();  // inicializando controlador,
                    // configura por default la escala de tiempo en MILLISECONDS y un tiempo minimo de 10ms

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