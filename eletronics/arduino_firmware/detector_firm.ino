#include <avr/io.h>
#include <avr/interrupt.h>

volatile double temp;
volatile int freq1 = 0;
volatile int freq2 = 0;
long int swit = 0;
bool Rstate = 0;

//Interrupt vector for INT0 (pin D2)
ISR (INT0_vect)
{
  if(swit >= 10)
  {
    if(Rstate == true){
      freq1 = (1/((micros() - temp)/1000000));
  	  freq2 = 0;
  	}
    else{
      freq2 = (1/((micros() - temp)/1000000));
      freq1 = 0;
  	}
  }
  temp = micros();
}

void setup()
{
  pinMode(13,OUTPUT);	//Relay pin (D13)
  temp = micros();
  
  cli(); //Disable interrupts for setup

  //Interrupt (on Arduino Uno pin 2)
  DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
  PORTD |= (1 << PORTD2);    // turn On the Pull-up
  EICRA |= (1 << ISC01);    // set INT0 to trigger on falling edge
  EIMSK |= (1 << INT0);     // Turns on INT0
  
  sei();	//Re-enable interrupts
}

void loop()
{
  //After some time, switch coils
  if(swit >= 80000)
  { 
    Rstate = not(Rstate); 
    swit = 0;
    digitalWrite(13,Rstate);
  }
  
  //Send frequencies through serial as soon as possible
  Serial.print(freq1);
  Serial.print(",");
  Serial.print(freq2);
  Serial.print("\n");

  swit++;
}