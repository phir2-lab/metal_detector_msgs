#define USB_CON

#include <avr/io.h>
#include <avr/interrupt.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <metal_detector/Coil.h>

volatile double temp;
volatile int32_t freq1 = 0;
volatile int32_t freq2 = 0;
long int swit = 0;
bool Rstate = 0;

//ROS handler
ros::NodeHandle nh;

//ROS messages
metal_detector::Coil coil_msg;

//ROS publishers and subscribers
ros::Publisher detector("metal_detector", &coil_msg);

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

  //Initialize ROS node
  nh.initNode();
  nh.advertise(detector);

  //Fill out the static parameters of the coil message type
  coil_msg.header.frame_id = "Coil";
  coil_msg.header.seq = 0;
  coil_msg.channel_length = 2;
  
  cli(); //Disable interrupts for setup

  //Interrupt (on Arduino Uno pin 2)
  DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
  PORTD |= (1 << PORTD2);    // turn On the Pull-up
  EICRA |= (1 << ISC01);    // set INT0 to trigger on falling edge
  EIMSK |= (1 << INT0);     // Turns on INT0
  
  sei();	//Re-enable interrupts
}

//Local vector to be passed to hold frequencies to be passed as pointers to the coil message
int32_t freqs[2];

void loop()
{
  //After some time, switch coils
  if(swit >= 20)
  { 
    Rstate = not(Rstate); 
    swit = 0;
    digitalWrite(13,Rstate);
  }
  swit++;

  //Populate message and publish it aprox every 100 ms (10 Hz)
  coil_msg.header.stamp = nh.now();
  freqs[0] = freq1; 
  freqs[1] = freq2;
  coil_msg.channel = freqs;
  detector.publish(&coil_msg);
  nh.spinOnce();
  delay(100);
}