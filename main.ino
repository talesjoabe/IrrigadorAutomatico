
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define F_CPU 16000000UL /*define a frequência do microcontrolador 16MHz (necessário
 para usar as rotinas de atraso)*/
#include <avr/io.h> //definições do componente especificado
#include <util/delay.h> //bibliot. para as rotinas de _delay_ms() e delay_us()
//Definições de macros - para o trabalho com os bits de uma variável
#define LED1 PD2 // PIN 2 (LED VERMELHO) - REPRESENTA QUE O SISTEMA ESTÁ DESLIGADO 
#define LED2 PD3 // PIN 3 (LED VERDE) - REPRESENTA QUE O SISTEMA ESTÁ LIGADO
#define botao PD4
#define botaoPressionado(PIND,botao) !tst_bit(PIND,botao)

#define turnLedOn(PORT, bits) PORT&=~(1<<bits)
#define turnLedOff(PORT,bits) PORT|=(1<<bits)
#define tst_bit(PIN,bits)(PIN&(1<<bits)) //testa o bit x da variável Y (retorna 0 ou 1)

int cont = 0;
bool state = false;
//------------------------------------------------------------------------------------

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

void set_counter()
{
  /*
   * Configura-se o contador
   * 1. COM0A1 - PWM NÃO INVERTIDO;
   * 2. WGM01 e WGM00 - MODO DE OPERAÇÃO DE 10 BITS;
   * 3. APÓS SETAR WGM12, PWM OPERA NO MODO FAST; 
   * 4. CS02 - HABILITA-SE UM DIVISOR DE 256 (DEFINE A TAXA DE SÍMBOLO DE TRANSMISSÃO);
   * 5. ativa a interrupção por estouro do TIMER 0.
   */
  cli();
  TCNT0 =  0b00000000;
  //TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(WGM02) | _BV(CS02);
  TIMSK0 = _BV(TOIE0);
  sei();
}

ISR(TIMER0_OVF_vect){
  cont++;
  if(cont == 1225){
    state = !state;
    cont = 0;
  }

  // -- Configura Interrupção do Timer0 --
  //
  // T0_OVF = (256 - timer0) x prescaler x ciclo de máquina
  //        = (256 -    0  ) x    256    x      62,5E-9
  //        =~ 4 ms
  //
  // Para 60 ms: 4ms x 15
} 

void setPwmOutput(){
   // iniciamos zerando o valor do output do pwm de TCCR1A
  OCR1A = 0;

  // configuracao do PWM
  //FAST PWM 10 bits (Modo 7) sem inversão
  /*
   * Configura-se a PWM
   * 1. COM1A1 - PWM NÃO INVERTIDO;
   * 2. WGM10 - MODO DE OPERAÇÃO DE 8 BITS;
   * 3. CS11 - HABILITA-SE UM DIVISOR DE 8 (DEFINE A TAXA DE SÍMBOLO DE TRANSMISSÃO);
   * 4. APÓS SETAR WGM12, PWM OPERA NO MODO FAST;
   */
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  DDRB = _BV(PB1);
}

void potenciometroEnable(uint8_t ad_value){
     while(true){
       set_ADC();
       while (!(ADCSRA & 0b00010000)); // aguarda conversao
       ad_value = mapFunction(ADC, 0, 1023, 0, 255);
       Serial.println(ad_value);
      
       if(state) {
        OCR1A = ad_value; 
       } else OCR1A=0;
       
       if(botaoPressionado(PIND, botao)){
         // sei();
          OCR1A= 0;
          state = !state;
          break;
       }
     }
}

int main()
{
  Serial.begin(9600);
  uint8_t ad_value;
 /* Configuração do DDRD
  * 1. PD2, PD3 saídas;
  */
 DDRD = 0b00001100; 
 /*/
  * Configura todas as portas como 
  */
 PORTD= 0b11111111; 
 bool system_on = false;

 set_counter();
 
  while(1){    
    if(!system_on){
      turnLedOn(PORTD,LED2);
      system_on = true; 
    }
    
     if(botaoPressionado(PIND, botao))
     {
       while(botaoPressionado(PIND, botao)); // enquanto eu não soltar o botão
       cont = 0;
       if(tst_bit(PORTD,LED1)){ 
         turnLedOn(PORTD, LED1);
         turnLedOff(PORTD,LED2);
          
         setPwmOutput();
         potenciometroEnable(ad_value);
       }
       else{ 
         turnLedOff(PORTD,LED1);
         turnLedOn(PORTD,LED2);
       }
     }
   }
}
