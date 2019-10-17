#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000
#define echo1 PORTC7 //Arduino - 30
#define trigger1 PORTC3 //Arduino - 34
#define echo2 PORTA0 //Arduino - 22
#define trigger2 PORTA2 //Arduino - 24
#define motorEF PORTD7 //Arduino - 38
#define motorET PORTG1 //Arduino - 40
#define motorDF PORTL7 //Arduino - 42
#define motorDT PORTL5 //Arduino - 44
#define velocidadeME PORTB5 //Arduino - 11
#define velocidadeMD PORTB6 //Arduino - 12


  
//definição de macros auxiliares
#define set_bit(reg,bit) (reg |= (1<<bit))
#define reset_bit(reg, bit) (reg &= ~(1<<bit))

int main(){
  Serial.begin(9600);
  set_bit(DDRD, motorEF); 
  set_bit(DDRG, motorET); 
  set_bit(DDRL, motorDF); 
  set_bit(DDRL, motorDT); 
  set_bit(DDRB, velocidadeME); 
  set_bit(DDRB, velocidadeMD); 
  set_bit(DDRC, echo1);
  set_bit(DDRC, trigger1);
  set_bit(DDRA, echo2);
  set_bit(DDRA, trigger2);
  
  uint16_t val, pwm, aleatorio, estado = 0, lastTimeComando = 0;
  int8_t comando = '0';

  while(1){
    uint32_t sensor1, sensor2;

    sensor1 



    
  }

  
} //fim main

uint16_t pegaPulsoEcho(Byte_8 variavel){                //Função para captura do pulso de echo gerado pelo sensor de ultrassom

  uint32_t i, resultado;                //Variáveis locais auxiliares
  
  for(i=0;i<600000;i++)               //Laço for para aguardar o início da borda de subida do pulso de echo
  {
    if(!(variavel & (1<<echo)))             //Pulso continue em nível baixo?
    continue;                   //Aguarda
    else                                            //Pulso em nível alto?
    break;                      //Interrompe
  } //end for
  
  
  //Configuração do Timer2 para contar o tempo em que o pulso de echo permanecerá em nível lógico alto
  
  TCCR1A = 0x00;                    //Desabilita modos de comparação A e B, Desabilita PWM
  TCCR1B = (1<<CS11);                 //Configura Prescaler (F_CPU/8)
  TCNT1  = 0x00;                    //Inicia contagem em 0
  
  
  for(i=0;i<600000;i++)               //Laço for para aguardar que ocorra a borda de descida do pulso de echo
  {
    if(PIND & (1<<echo))              //Pulso continua em nível alto?
    {
      if(TCNT1 > 60000) break;          //Interrompe se TCNT2 atingir o limite da contagem
      else continue;                //Senão, continua
    }
    else
    break;                      //Interrompe quando encontrar a borda de descida
    
  } //end for
  
  resultado = TCNT1;                  //Salva o valor atual de TCNT2 na variável resultado (tempo que o echo ficou em high)
  
  TCCR1B = 0x00;                    //Interrompe Timer
  
  
  return (resultado>>1);                //Função retornará o tempo em microssegundos
  
  
} 

