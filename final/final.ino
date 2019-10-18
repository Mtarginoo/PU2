#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
<<<<<<< HEAD
#include <Ultrasonic.h>
#include <ArduinoJson.h>

#define F_CPU 16000000
#define pino_echo1 PORTC7 //Arduino - 30
#define pino_trigger1 PORTC3 //Arduino - 34
#define pino_echo2 PORTA0 //Arduino - 22
#define pino_trigger2 PORTA2 //Arduino - 24
=======

#define F_CPU 16000000
#define echo1 PORTC7 //Arduino - 30
#define trigger1 PORTC3 //Arduino - 34
#define echo2 PORTA0 //Arduino - 22
#define trigger2 PORTA2 //Arduino - 24
>>>>>>> cac1043b1c153b604b7a827adbb95777d680c04c
#define motorEF PORTD7 //Arduino - 38
#define motorET PORTG1 //Arduino - 40
#define motorDF PORTL7 //Arduino - 42
#define motorDT PORTL5 //Arduino - 44
#define velocidadeME PORTB5 //Arduino - 11
#define velocidadeMD PORTB6 //Arduino - 12
<<<<<<< HEAD
#define potenciometro A0


#define set_bit(reg,bit) (reg |= (1<<bit))
#define reset_bit(reg, bit) (reg &= ~(1<<bit))

int main(){
  Serial.begin(115200);
=======

void HCSR04Trig();
volatile int cont = 0;
int aux = 0;
int counter = 0x00;

  
//definição de macros auxiliares
#define set_bit(reg,bit) (reg |= (1<<bit))
#define reset_bit(reg, bit) (reg &= ~(1<<bit))

ISR(TIMER0_OVF_vect){
  cont++;
  if(cont == 980){
    if(aux == 1){
      aux = 0;
    }
    cont = 0;
  }

  // -- Configura Interrupção do Timer0 --
  //
  // T0_OVF = (256 - timer0) x prescaler x ciclo de máquina
  //        = (256 -    0  ) x    256    x      62,5E-9
  //        =~ 4 ms
  //
  // Para 60 ms: 4ms x 15
  
  counter++; 
  if(counter == 15){             
    HCSR04Trig();               //pulso de trigger
    counter = 0; 
  }
  
} //end ISR Timer0

int main(){
  Serial.begin(9600);
>>>>>>> cac1043b1c153b604b7a827adbb95777d680c04c
  set_bit(DDRD, motorEF); 
  set_bit(DDRG, motorET); 
  set_bit(DDRL, motorDF); 
  set_bit(DDRL, motorDT); 
  set_bit(DDRB, velocidadeME); 
  set_bit(DDRB, velocidadeMD); 
<<<<<<< HEAD
  set_bit(DDRC, pino_echo1);
  set_bit(DDRC, pino_trigger1);
  set_bit(DDRA, pino_echo2);
  set_bit(DDRA, pino_trigger2);

  Ultrasonic ultrasonic1(pino_trigger1, pino_echo1);
  Ultrasonic ultrasonic2(pino_trigger2, pino_echo2);

=======
  set_bit(DDRC, echo1);
  set_bit(DDRC, trigger1);
  set_bit(DDRA, echo2);
  set_bit(DDRA, trigger2);
>>>>>>> cac1043b1c153b604b7a827adbb95777d680c04c
  
  uint16_t val, pwm, aleatorio, estado = 0, lastTimeComando = 0;
  int8_t comando = '0';

  while(1){
<<<<<<< HEAD
    uint32_t sensor1, sensor2, microsec1, microsec2;

    microsec1 = ultrasonic1.timing();
    microsec2 = ultrasonic2.timing();
    sensor1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
    sensor2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);

    val = analogRead(potenciometro);
    pwm = map(val, 0, 1023, 0, 255);
    comando = Serial.available() > 0 ? Serial.read() : comando;

    DynamicJsonBuffer jBuffer;
    JsonObject &root = jBuffer.createObject();

    root["Sensor1"] = sensor1;
    root["Sensor2"] = sensor2;

    root.printTo(Serial);
    Serial.println();

    if (comando == '1')
    { //andando para frente
        digitalWrite(motorEF, HIGH);
        digitalWrite(motorDF, HIGH);
        digitalWrite(motorET, LOW);
        digitalWrite(motorDT, LOW);
        analogWrite(velocidadeMD, pwm);
        analogWrite(velocidadeME, pwm);
        if (sensor1 < 15)
        {
            aleatorio = random(0, 3);
            while (sensor1 < 15)
            {
                if (aleatorio == 1)
                { //curva para esquerda
                    analogWrite(velocidadeME, pwm * 0.1);
                    analogWrite(velocidadeMD, pwm);
                    digitalWrite(motorEF, HIGH);
                    digitalWrite(motorDF, HIGH);
                    digitalWrite(motorET, LOW);
                    digitalWrite(motorDT, LOW);
                }
                else if (aleatorio == 2)
                { //curva para direita
                    analogWrite(velocidadeMD, pwm * 0.1);
                    analogWrite(velocidadeME, pwm);
                    digitalWrite(motorEF, HIGH);
                    digitalWrite(motorDF, HIGH);
                    digitalWrite(motorET, LOW);
                    digitalWrite(motorDT, LOW);
                }
                microsec1 = ultrasonic1.timing();
                sensor1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
            }
        }
    }
    else if (comando == '2')
    {
        digitalWrite(motorET, HIGH);
        digitalWrite(motorDT, HIGH);
        digitalWrite(motorEF, LOW);
        digitalWrite(motorDF, LOW);
        analogWrite(velocidadeMD, pwm);
        analogWrite(velocidadeME, pwm);
        if (sensor2 < 15)
        {
            aleatorio = random(0, 3);
            while (sensor2 < 15)
            {
                if (aleatorio == 1)
                { //curva para esquerda
                    analogWrite(velocidadeMD, pwm * 0.1);
                    analogWrite(velocidadeME, pwm);
                    digitalWrite(motorET, HIGH);
                    digitalWrite(motorDT, HIGH);
                    digitalWrite(motorEF, LOW);
                    digitalWrite(motorDF, LOW);
                }
                else if (aleatorio == 2)
                { //curva para direita
                    analogWrite(velocidadeME, pwm * 0.1);
                    analogWrite(velocidadeMD, pwm);
                    digitalWrite(motorET, HIGH);
                    digitalWrite(motorDT, HIGH);
                    digitalWrite(motorEF, LOW);
                    digitalWrite(motorDF, LOW);
                }
                microsec2 = ultrasonic2.timing();
                sensor2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);
            }
        }
    }
  }
}


/*
int val;
int pwm;
int estado = 0;
int aleatorio;
int lastTimeComando = 0;
char comando = '0';
Ultrasonic ultrasonic1(pino_trigger1, pino_echo1);
Ultrasonic ultrasonic2(pino_trigger2, pino_echo2);

void setup()
{
    Serial.begin(115200);
    pinMode(motorEF, OUTPUT);
    pinMode(motorET, OUTPUT);
    pinMode(motorDF, OUTPUT);
    pinMode(motorDT, OUTPUT);
    pinMode(velocidadeMD, OUTPUT);
    pinMode(velocidadeME, OUTPUT);
    pinMode(potenciometro, INPUT);
}

void loop()
{
    float sensor1, sensor2;
    long microsec1;
    long microsec2;

    microsec1 = ultrasonic1.timing();
    microsec2 = ultrasonic2.timing();
    sensor1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
    sensor2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);

    val = analogRead(potenciometro);
    pwm = map(val, 0, 1023, 0, 255);
    comando = Serial.available() > 0 ? Serial.read() : comando;

    DynamicJsonBuffer jBuffer;
    JsonObject &root = jBuffer.createObject();

    root["Sensor1"] = sensor1;
    root["Sensor2"] = sensor2;

    root.printTo(Serial);
    Serial.println();

    if (comando == '1')
    { //andando para frente
        digitalWrite(motorEF, HIGH);
        digitalWrite(motorDF, HIGH);
        digitalWrite(motorET, LOW);
        digitalWrite(motorDT, LOW);
        analogWrite(velocidadeMD, pwm);
        analogWrite(velocidadeME, pwm);
        if (sensor1 < 15)
        {
            aleatorio = random(0, 3);
            while (sensor1 < 15)
            {
                if (aleatorio == 1)
                { //curva para esquerda
                    analogWrite(velocidadeME, pwm * 0.1);
                    analogWrite(velocidadeMD, pwm);
                    digitalWrite(motorEF, HIGH);
                    digitalWrite(motorDF, HIGH);
                    digitalWrite(motorET, LOW);
                    digitalWrite(motorDT, LOW);
                }
                else if (aleatorio == 2)
                { //curva para direita
                    analogWrite(velocidadeMD, pwm * 0.1);
                    analogWrite(velocidadeME, pwm);
                    digitalWrite(motorEF, HIGH);
                    digitalWrite(motorDF, HIGH);
                    digitalWrite(motorET, LOW);
                    digitalWrite(motorDT, LOW);
                }
                microsec1 = ultrasonic1.timing();
                sensor1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
            }
        }
    }
    else if (comando == '2')
    {
        digitalWrite(motorET, HIGH);
        digitalWrite(motorDT, HIGH);
        digitalWrite(motorEF, LOW);
        digitalWrite(motorDF, LOW);
        analogWrite(velocidadeMD, pwm);
        analogWrite(velocidadeME, pwm);
        if (sensor2 < 15)
        {
            aleatorio = random(0, 3);
            while (sensor2 < 15)
            {
                if (aleatorio == 1)
                { //curva para esquerda
                    analogWrite(velocidadeMD, pwm * 0.1);
                    analogWrite(velocidadeME, pwm);
                    digitalWrite(motorET, HIGH);
                    digitalWrite(motorDT, HIGH);
                    digitalWrite(motorEF, LOW);
                    digitalWrite(motorDF, LOW);
                }
                else if (aleatorio == 2)
                { //curva para direita
                    analogWrite(velocidadeME, pwm * 0.1);
                    analogWrite(velocidadeMD, pwm);
                    digitalWrite(motorET, HIGH);
                    digitalWrite(motorDT, HIGH);
                    digitalWrite(motorEF, LOW);
                    digitalWrite(motorDF, LOW);
                }
                microsec2 = ultrasonic2.timing();
                sensor2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);
            }
        }
    }
}
*/
=======
    uint16_t sensor1;
  
    sensor1 = pegaPulsoEcho(); 
    Serial.println(sensor1);
  }


  
} //fim main

uint16_t pegaPulsoEcho(){                //Função para captura do pulso de echo gerado pelo sensor de ultrassom

  uint32_t i, resultado;                //Variáveis locais auxiliares
  
  for(i=0;i<600000;i++)               //Laço for para aguardar o início da borda de subida do pulso de echo
  {
    if(!(PORTC & (1<<echo1)))             //Pulso continue em nível baixo?
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
    if(PINC & (1<<echo1))              //Pulso continua em nível alto?
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

void HCSR04Trig()                //gera pulso de Trigger
{
  
  set_bit(PORTC,trigger1);
  _delay_us(10);
  reset_bit(PORTC,trigger1);
  
} //end HCSR04Trig

>>>>>>> cac1043b1c153b604b7a827adbb95777d680c04c
