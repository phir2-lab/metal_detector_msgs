#include <avr/io.h>
#include <avr/interrupt.h>

volatile double temp;
volatile int freqL = 0;
volatile int freqR = 0;
long int swit = 0;
bool Rstate = 0;

//Interrupt vector for INT0 (pin D2)
ISR (INT0_vect)
{
  if(swit >= 400)
  {
  	//Bobina direita
    if(Rstate == true){
      freqR = (1/((micros() - temp)/1000000));
  	}
  	//Bobina esquerda
    else{
      freqL = (1/((micros() - temp)/1000000));
  	}
  }
  temp = micros();
}

void setup()
{
  pinMode(13,OUTPUT);	//Relay pin (D13)
  temp = micros();

  Serial.begin(9600);
  
  cli(); //Disable interrupts for setup

  //Interrupt (on Arduino Uno pin 2)
  DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
  PORTD |= (1 << PORTD2);    // turn On the Pull-up
  EICRA |= (1 << ISC01);    // set INT0 to trigger on falling edge
  EIMSK |= (1 << INT0);     // Turns on INT0
  
  sei();	//Re-enable interruptsd
}

void loop()
{
  //After some time, switch coils
  if(swit >= 800)
  { 
    Rstate = not(Rstate); 
    swit = 0;
    digitalWrite(13,Rstate);
    freqL = 0;
    freqR = 0;
  }
  
  Serial.print(freqL);
  Serial.print(",");
  Serial.print(freqR);
  Serial.print("\n");

  swit++;
}
