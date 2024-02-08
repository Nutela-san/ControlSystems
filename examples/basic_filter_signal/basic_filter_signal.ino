#include <SimpleFilters.h>

uint8_t pot_pin = A0; //pin para leer una señal analogica a filtrar

//Todas las clases de filtros son un template, y el template necesita el tipo de dato
//como primer parametro.

//La template de MA_filter tambien necesita tambien un parametro de numero de muestras para hacer
//el promedio.
MA_Filter<int,15> filter1;

//EMA_filter pide un parametro alpha (valor entre 0 y 1, generalmente entre 0.2 y 0.6)
EMA_Filter<int> filter2(0.2f);

//La clase de filtro de primer orden RC pide un parametro de frecuencia de corte (Hz)
//y un parametro de periodo(segundos) que es el tiempo entre muestras
RC_FirstO_Filter<int> filter3(1.0f,10.0f*0.001f);

void setup(){
  pinMode(pot_pin,INPUT);

  Serial.begin(115200);
  filter1.reset();  //se resetea el estado interno del filtro
  filter2.reset();
  filter3.reset();

  Serial.println("Init. Filters");
}

void loop(){
  int pot_raw_value = analogRead(pot_pin); //Se obtine la muestra de la señal (valor de lectura del adc)
  int pot_filtered1_value = filter1.doFilter(pot_raw_value);  //Metodo que recibe el valor y lo filtra,
                                                              //LPF esta por default (LPF = filtro pasa bajas)
  int pot_filtered2_value = filter2.doFilter(pot_raw_value,Filter_t::HPF);  //se da otro parametro para definir
                                                                            //si es pasa bajas o pasa altas
  int pot_filtered3_value = filter3.doFilter(pot_raw_value,Filter_t::LPF);

  //Se imprimen los valores, para poder monitorearlos en el monitor serial o en el Serial plot;
  Serial.print("Pot_v = ");
  Serial.print(pot_raw_value);
  Serial.print(", filter_MA = ");
  Serial.print(pot_filtered1_value);
  Serial.print(", filter_EMA = ");
  Serial.print(pot_filtered2_value);
  Serial.print(", filter_RC = ");
  Serial.println(pot_filtered3_value);
  delay(10); //leyendo A0 aprox. 100Hz
}