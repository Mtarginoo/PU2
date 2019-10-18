#include <Ultrasonic.h>
#include <ArduinoJson.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000
#define pino_echo1 30 //PORTC7 //Arduino - 30
#define pino_trigger1 34 //PORTC3 //Arduino - 34
#define pino_echo2 22 //PORTA0 //Arduino - 22
#define pino_trigger2 24 //PORTA2 //Arduino - 24
#define motorEF PORTD7 //Arduino - 38
#define motorET PORTG1 //Arduino - 40
#define motorDF PORTL7 //Arduino - 42
#define motorDT PORTL5 //Arduino - 44
#define velocidadeME PORTB5 //Arduino - 11
#define velocidadeMD PORTB6 //Arduino - 12
#define botao PORTB3 // Arduino - 50
#define potenciometro PORTF0 // Arduino - A0


#define set_bit(reg,bit) (reg |= (1<<bit))
#define reset_bit(reg, bit) (reg &= ~(1<<bit))
#define myDigitalWrite(reg, bit, level) ((level == 1) ? set_bit(reg,bit) : reset_bit(reg,bit))
#define myDigitalRead(pino, bit) (pino & (1<<bit) ? 1 : 0)

void adc_init()
{
    // AREF = AVcc
    ADMUX = (1<<REFS0);
    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t myAnalogRead(uint8_t ch)
{
  // select the corresponding channel 0~7
  // ANDing with ’7′ will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
 
  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);
 
  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));
 
  return (ADC);
}

double qlqrcoisaai = 0;

int main() {
  Serial.begin(115200);
  
  sei();
  
  set_bit(DDRD, motorEF);
  set_bit(DDRG, motorET);
  set_bit(DDRL, motorDF);
  set_bit(DDRL, motorDT);
  set_bit(DDRB, velocidadeME);
  set_bit(DDRB, velocidadeMD);
  reset_bit(DDRF, potenciometro);
  
  adc_init();

  TCCR0A = (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);
  TIMSK0 = (1<<TOIE0);
  OCR0A = (qlqrcoisaai/100.0) * 255;
  TCCR0B = (1<<CS00);
  
  Ultrasonic ultrasonic1(pino_trigger1, pino_echo1);
  Ultrasonic ultrasonic2(pino_trigger2, pino_echo2);


  uint16_t val, pwm, aleatorio, estado = 0, lastTimeComando = 0;
  int8_t comando = '2';

  while (1) {
    uint32_t sensor1, sensor2, microsec1, microsec2;

    microsec1 = ultrasonic1.timing();
    microsec2 = ultrasonic2.timing();
    sensor1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
    sensor2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);

    val = myAnalogRead(0);
    pwm = map(val, 0, 1023, 0, 255);
    Serial.println(pwm);
    comando = Serial.available() > 0 ? Serial.read() : comando;

    DynamicJsonBuffer jBuffer;
    JsonObject &root = jBuffer.createObject();

    root["Sensor1"] = sensor1;
    root["Sensor2"] = sensor2;

    root.printTo(Serial);
    Serial.println();

    if (comando == '1')
    { //andando para frente

      myDigitalWrite(PORTD, motorEF, 1);
      myDigitalWrite(PORTL, motorDF, 1);
      myDigitalWrite(PORTG, motorET, 0);
      myDigitalWrite(PORTL, motorDT, 0);

      analogWrite(velocidadeMD, pwm);
      analogWrite(velocidadeME, pwm);
      if (sensor1 < 15)
      {
        aleatorio = random(0, 3);
        while (sensor1 < 15)
        {
          if (aleatorio == 1)
          { //curva para esquerda
            analogWrite(velocidadeMD, pwm * 0.1);
            analogWrite(velocidadeME, pwm);
            myDigitalWrite(PORTD, motorEF, 1);
            myDigitalWrite(PORTL, motorDF, 1);
            myDigitalWrite(PORTG, motorET, 0);
            myDigitalWrite(PORTL, motorDT, 0);
          }
          else if (aleatorio == 2)
          { //curva para direita
            analogWrite(velocidadeMD, pwm * 0.1);
            analogWrite(velocidadeME, pwm);
            myDigitalWrite(PORTD, motorEF, 1);
            myDigitalWrite(PORTL, motorDF, 1);
            myDigitalWrite(PORTG, motorET, 0);
            myDigitalWrite(PORTL, motorDT, 0);
          }
          microsec1 = ultrasonic1.timing();
          sensor1 = ultrasonic1.convert(microsec1, Ultrasonic::CM);
        }
      }
    }
    else if (comando == '2')
    {
      myDigitalWrite(PORTG, motorET, 1);
      myDigitalWrite(PORTL, motorDT, 1);
      myDigitalWrite(PORTD, motorEF, 0);
      myDigitalWrite(PORTL, motorDF, 0);
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
            myDigitalWrite(PORTG, motorET, 1);
            myDigitalWrite(PORTL, motorDT, 1);
            myDigitalWrite(PORTD, motorEF, 0);
            myDigitalWrite(PORTL, motorDF, 0);
          }
          else if (aleatorio == 2)
          { //curva para direita
            analogWrite(velocidadeMD, pwm * 0.1);
            analogWrite(velocidadeME, pwm);
            myDigitalWrite(PORTG, motorET, 1);
            myDigitalWrite(PORTL, motorDT, 1);
            myDigitalWrite(PORTD, motorEF, 0);
            myDigitalWrite(PORTL, motorDF, 0);
          }
          microsec2 = ultrasonic2.timing();
          sensor2 = ultrasonic2.convert(microsec2, Ultrasonic::CM);
        }
      }
    }
  }
}

ISR(TIMER0_OVF_vect)
{
  OCR0A = (qlqrcoisaai/100.0) * 255;
  
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
