/*
  Posicion de una bola en un riel (Ejemplo Basico de Control)

  Usando un sensor ultrasonico se mide la distancia entre el sensor y
  la bola, se calcula el error mediante el setpoit (la distancia deseada) y la
  medicion del sensor de la forma error =  setpoit - medida; se introduce el
  error en la implementación del PID y la salida del controlador se utiliza como
  entrada de angulo para un servo motor que varia la inclinacion del riel.

  Se utilizara la libreria de "InterCom.h" para ajustar las ganacias del controlador

*/

#include <PIDControl.h>
#include <InterCom.h>
//#include <Servo.h>

//--- Defincion de PINES (para arduino UNO)---
uint8_t trig = 2, echo = 3, pin_servo = 5;

float error = 0, setpoint;
int servo_value = 0;

SimplePID control;

SimpleComand comandos;

//Servo actuador;

void config_pines()
{
  pinMode(pin_servo, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  //actuador.attach(pin_servo);
}

void info()
{
  Serial.println("--Informacion Controlador--");
  Serial.print("Setpoint = ");
  Serial.println(setpoint);

  Serial.print("error = ");
  Serial.println(error);

  Serial.print("kp = ");
  Serial.print(control.kp);
  Serial.print(", ki = ");
  Serial.print(control.ki);
  Serial.print(", kd = ");
  Serial.println(control.kd);

  Serial.print("Controlador OUT = ");
  Serial.println(servo_value);
}

void config_comandos()
{
  comandos.begin(115200);
  comandos.enable_echo(true);

  comandos.addComand("t", &setpoint);
  comandos.addComand("p", &control.kp);
  comandos.addComand("i", &control.ki);
  comandos.addComand("d", &control.kd);
  comandos.addComand("control info", info);
}

float leerDistancia()
{
  digitalWrite(trig, HIGH);
  digitalWrite(trig, LOW);

  ulong fly_time = pulseIn(echo, HIGH, 272); // 272 es el valor en us (micro segundos) que el
                                             // que el sonido viaja 2.4 metros de ida y vuelta
  float distancia = ((float)(34e-3) * (float)fly_time);

  return distancia;
}

void move_servo(float degrees)
{
  //actuador.write(90 + degrees);
}

void setup()
{

  config_pines();

  control.begin(MILISECONDS, 30); // inicializando controlador,
                                  // configura por default la escala de tiempo en MILISECONDS y un tiempo minimo de 1ms

  control.setGains(1, 0, 0); // configura las ganacias kp, ki y kd

  control.setOutLimits(80); // limites simetricos (minimo = -80 , maximo = 80) que usaremos como ° para el servo

  control.setIntegralLimits(20); // limites simetricos (minimo = -20 , maximo = 20)

  config_comandos();
}

void loop()
{
  float medida = leerDistancia();

  error = setpoint - medida;

  servo_value = control.calulate_out(error);

  move_servo(servo_value);

  /*
    Puede reducirse a
      move_servo(control.calculate_out(error));
  */
}