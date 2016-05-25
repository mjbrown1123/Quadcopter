// 5 = button
// 5v = button
// 10k = ground (button)
// A1, A2, A3 (potentiometers)
//999 code indicates quadcopter shutdown

//pins for tuning potentiometers
int pPin = 0;
int iPin = 1;
int dPin = 2;

//these longs store the gain send values
long pSend = 0;
long iSend = 0;
long dSend = 0;

//these longs store input values
long kP = 0;
long kI = 0;
long kD = 0;

//define LED pin
int ledPin = 2;

//emergency stop button
int but = 5;
int state = 0;

//radio libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//define pins for the radio
#define CE_PIN 9
#define CSN_PIN 10

//define the transmit pipe
const uint64_t pipe = 0xE8E8F0F0E1LL; 

//initialize a radio object
RF24 radio(CE_PIN, CSN_PIN); 

void setup() {
  //to initialize part of the radio
  radio.begin();
  radio.openWritingPipe(pipe);

  //begin serial communication (for debugging)
  Serial.begin(115200);

  //set the button as an input
  pinMode(but, INPUT);

  //make sure the LED is off
  digitalWrite(ledPin, LOW);
}

void loop() {

  long deg = 0;

  //check button state
  state = digitalRead(but);
  
  //read the values from all tuning potentiometers
  kP = analogRead(pPin);
  kI = analogRead(iPin);
  kD = analogRead(dPin);

  //map each value from 0 to 150
  pSend = map(kP, 0, 1023, 0, 150);
  iSend = map(kI, 0, 1023, 0, 150);
  dSend = map(kD, 0, 1023, 0, 150);

  //increase each gain by a certain factor to pack for sending
  iSend *= 1000;
  dSend *= 1000000;

  //add signals together
  deg = deg + pSend + iSend + dSend; 

  //if the emergency stop button is clicked...
  if(state == HIGH) {

    //send stop command
    deg = 999;

    //light the LED indicating the button is pressed
    digitalWrite(ledPin, HIGH);
  }
  else {

    //turn the indicator LED off
    digitalWrite(ledPin,LOW);
   }
   
  //send signal to the quad
  radio.write( &deg, sizeof(deg));

  //print signal values for debug
  Serial.print("Sent: ");
  Serial.print(deg);
  Serial.print("      Proportional gain: ");
  Serial.print((double)pSend / 100);
  Serial.print("      Integral Gain: ");
  Serial.print((double)iSend / 100000);
  Serial.print("      Derivative Gain: ");
  Serial.println((double)dSend / 100000000);

 
  
}


