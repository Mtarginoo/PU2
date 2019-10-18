#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <Ultrasonic.h>
#include <ArduinoJson.h>

#define F_CPU 16000000
#define pino_echo1 PORTC7 //Arduino - 30
#define pino_trigger1 PORTC3 //Arduino - 34
#define pino_echo2 PORTA0 //Arduino - 22
#define pino_trigger2 PORTA2 //Arduino - 24
#define motorEF PORTD7 //Arduino - 38
#define motorET PORTG1 //Arduino - 40
#define motorDF PORTL7 //Arduino - 42
#define motorDT PORTL5 //Arduino - 44
#define velocidadeME PORTB5 //Arduino - 11
#define velocidadeMD PORTB6 //Arduino - 12
#define potenciometro A0


#define set_bit(reg,bit) (reg |= (1<<bit))
#define reset_bit(reg, bit) (reg &= ~(1<<bit))

int main(){
  Serial.begin(115200);
  set_bit(DDRD, motorEF);
  set_bit(DDRG, motorET);
  set_bit(DDRL, motorDF);
  set_bit(DDRL, motorDT);
  set_bit(DDRB, velocidadeME);
  set_bit(DDRB, velocidadeMD);
  set_bit(DDRC, pino_echo1);
  set_bit(DDRC, pino_trigger1);
  set_bit(DDRA, pino_echo2);
  set_bit(DDRA, pino_trigger2);

  Ultrasonic ultrasonic1(pino_trigger1, pino_echo1);
  Ultrasonic ultrasonic2(pino_trigger2, pino_echo2);

 
  uint16_t val, pwm, aleatorio, estado = 0, lastTimeComando = 0;
  int8_t comando = '0';

  while(1){
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

