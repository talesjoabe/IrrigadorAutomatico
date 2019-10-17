/*
Entradas e Saídas                     Pinagem (Arduino)

ADC0 - Potenciometro (Controle de fluxo de água)                      Pino A0

*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

// configuração do ADC
void set_ADC(void)
{
  // Registrador de Seleção
  ADMUX |= 0b01000000; // 01 ref(Vcc); 0 (ADC - SEM ADLAR); 0 (RESERVADO); 0000 (MUX p/ ADC0)
  // Registrador de Status
  ADCSRA |= 0b11000111; // 1 (ADEN: Enable); 10 (ADSC: Start Conversion e ADATE: sem auto trigger); 00 (ADIF: Flag de interrupção e ADIE: Interrupt Enable); 111 (Prescaler - Divisão por 128)
  // Habilita uso de interrupção
  sei();
}

long mapFunction(long adc, long in_min, long in_max, long out_min, int out_max)
{
  return (adc - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main()
{
  Serial.begin(9600);
  uint8_t ad_value;

  DDRB &= 0b11110011; // entrada (PB2 e PB3)
  DDRD |= 0b11110000; // saída (PD4,PD5,PD6,PD7)

  PORTB |= 0b00001100; // pull up (PB2 e PB3)
  PORTD &= 0b00001111; // Saída em nível baixo (PD4 até PD7)

  while (true)
  {
    set_ADC();
    while (!(ADCSRA & 0b00010000)) // aguarda conversao
      ;
    ad_value = mapFunction(ADC, 0, 1023, 0, 255);
    Serial.println(ad_value);
  }
}
