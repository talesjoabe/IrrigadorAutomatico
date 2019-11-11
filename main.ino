
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define F_CPU 16000000UL /*define a frequência do microcontrolador 16MHz (necessário
 para usar as rotinas de atraso)*/
#define BAUD   9600    //taxa de 9600 bps
#define MYUBRR  F_CPU/16/BAUD-1

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
unsigned char *chptr;
//------------------------------------------------------------------------------------


void USART_Inic(unsigned int ubrr)
{
  UBRR0H = (unsigned char)(ubrr >> 8); //Ajusta a taxa de transmissão
  UBRR0L = (unsigned char)ubrr;

  UCSR0A = 0;//desabilitar velocidade dupla (no Arduino é habilitado por padrão)
  UCSR0B = _BV(RXEN0) | _BV(TXEN0); //Habilita tanto o transmissor quanto o receptor
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /*modo assíncrono, 8 bits de dados, 1 bit de parada, sem paridade*/
}

void USART_Transmit(unsigned char info)
{
  while(!(UCSR0A & (1<<UDRE0))); // espera a limpeza do registrador de transmissão
  UDR0 = info; // envia o dado
}

void escreverMensagem(char *c) {  
  for (; *c != 0; c++) USART_Transmit(*c);
}

uint8_t rxByte()
{
  //Bit RXC sinaliza quando existem bytes não lidos no buffer
  while(!(UCSR0A & (1<<RXC0)));
  return UDR0;
}

/*
 * Limpa o registrador de entrada - quando ocorre um erro, por exemplo - 
 */
void USART_Flush( void )
{
 unsigned char dummy;
 while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

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
       //chptr = (unsigned char *) &ad_value;
       //txByte(*chptr);
       //txByte(*chptr++);
       //txByte(*chptr++);
       //txByte(*chptr++);
       
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
  //Serial.begin(9600);
  uint8_t ad_value;
  USART_Inic(MYUBRR);
  
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

      escreverMensagem("Para ligar o sistema, pressione o botão\n\0!");
    }
   
     
     if(botaoPressionado(PIND, botao))
     {
       while(botaoPressionado(PIND, botao)); // enquanto eu não soltar o botão
       cont = 0;
       if(tst_bit(PORTD,LED1)){ 
         turnLedOn(PORTD, LED1);
         turnLedOff(PORTD,LED2);
         
         escreverMensagem("Sistema ligado! \n\0!");
         
         setPwmOutput();
         potenciometroEnable(ad_value);
       }
       else{ 
         turnLedOff(PORTD,LED1);
         turnLedOn(PORTD,LED2);

         escreverMensagem("Sistema desligado!\n\0!");
       }
     }
   }
}
