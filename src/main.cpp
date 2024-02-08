#include <Arduino.h>
#include <SimpleFilters.h>

#ifdef ARDUINO_ESP32C3_DEV
  HWCDC SerialUSB;
  #define port SerialUSB
#else
  #define port Serial
#endif

uint8_t pot_pin = A0;


MA_Filter<int,15> filtro1;

EMA_Filter<int> filtro2(0.2f);

RC_FirstO_Filter<int> filtro3(1.0f,10.0f*0.001f);

void setup(){
  pinMode(pot_pin,INPUT);

  port.begin(115200);
  filtro1.reset();

  port.println("Filtros inicializados");
}

void loop(){
  int pot_raw_value = analogRead(pot_pin);
  int pot_filtered1_value = filtro1.doFilter(pot_raw_value); //LPF esta por default
  int pot_filtered2_value = filtro2.doFilter(pot_raw_value,Filter_t::HPF);
  int pot_filtered3_value = filtro3.doFilter(pot_raw_value,Filter_t::LPF);
  port.print("Pot_v = ");
  port.print(pot_raw_value);
  port.print(", filter_MA = ");
  port.print(pot_filtered1_value);
  port.print(", filter_EMA = ");
  port.print(pot_filtered2_value);
  port.print(", filter_RC = ");
  port.println(pot_filtered3_value);
  delay(10);
}