// PID library
#include <PID_v1.h>

//gain values for the roll
double rki = 0;
double rkd = 0;
double rkp = 0;

//assign pin numbers to the motors
int motorPin1 = 3;
int motorPin2 = 5;
int motorPin3 = 6;
int motorPin4 = 7;

//set PID sample time (in milliseconds)
int sampleTime = 10;

//set PID output limits 
int outputLimit = 400;

//gain values for the pitch
double pki = 0;
double pkd = 0;
double pkp = 0;

//input for the pitch and roll PID
double pIn = 0;
double rIn = 0;

//output from the pitch and roll PID
double rOut = 0;
double pOut = 0;

//setpoint for the quadcopter (0 degrees because we want the quadcopter to be stable)
double rSet = 0;
double pSet = 0;

//create PID objects for roll and pitch
PID roll(&rIn, &rOut, &rSet, rkp, rki, rkd, DIRECT);
PID pitch(&pIn, &pOut, &pSet, pkp, pki, pkd, DIRECT);

//variable stores the 
long kvals = 0;

//final values for pitch and roll
int pFinal = 0;
int rFinal = 0;

//set base value for the motors
int base = 0;

//include servo library
#include <Servo.h>;

//create servo objects for the four motors
Servo s1, s2, s3, s4;

//values that go to each motor
int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;

//startup
int start = 1;

//radio libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//define the radio pins
#define CE_PIN 9
#define CSN_PIN 53

//transmit pipe
const uint64_t pipe = 0xE8E8F0F0E1LL;

//define radio object
RF24 radio(CE_PIN, CSN_PIN);

//gyro libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//create gyro object
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//interrupt pin
volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {

  //attach the servo objects
  s1.attach(motorPin1);
  s2.attach(motorPin2);
  s3.attach(motorPin3);
  s4.attach(motorPin4);

  //initialize radio
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();;



  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(115200);

  // initialize gyro
  mpu.initialize();

  //empty buffer
  while (Serial.available() && Serial.read());

  // load the DMP
  devStatus = mpu.dmpInitialize();

  //set the gyro offsets
  mpu.setXGyroOffset(114);
  mpu.setYGyroOffset(32);
  mpu.setZGyroOffset(22);
  mpu.setZAccelOffset(1812);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);

    //get the interrupt status
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();


  }

  //make PID update automatically
  roll.SetMode(AUTOMATIC);
  pitch.SetMode(AUTOMATIC);

  //set limits to the outputs
  roll.SetOutputLimits(-outputLimit, outputLimit);
  pitch.SetOutputLimits(-outputLimit, outputLimit);

  //update every 10 milliseconds
  roll.SetSampleTime(sampleTime);
  pitch.SetSampleTime(sampleTime);

  //initialize motors by sending them to 0
  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);

  //stop for a second
  delay(1000);

}



void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;


  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {


  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

    // reset fifo buffer
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt
  } else if (mpuIntStatus & 0x02) {

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    //get orientation values from gyro
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //read values from the radio
    radioRead();

    //turn angles into degrees for PID
    pIn = (ypr[1] * 180 / M_PI);
    rIn = (ypr[2] * 180 / M_PI);


    //compute PID values
    roll.Compute();
    pitch.Compute();

    //get motor values from the computer. For debugging 
    if (Serial.available()) {

      //get speed value
      int baseRead = Serial.parseInt();

      if(baseRead < 2000) {
          //set it to the base
          base = baseRead;
      }
    }

    //make final pitch and roll values
    pFinal = (int)pOut;
    rFinal = (int)rOut;


    // negative roll forward, positive roll backward, negative pitch right, positive pitch left
    val1 = base - rFinal;
    val2 = base - pFinal;
    val3 = base + rFinal;
    val4 = base + pFinal;

    //turn motors 2 and 4 off for tuning
    s1.write(0);
    s3.write(0);

    //send values to motors 1 and 3
    //s1.writeMicroseconds(val1);
    s2.writeMicroseconds(val2);
    //s3.writeMicroseconds(val3);
    s4.writeMicroseconds(val4);

    //debug statements
    Serial.print("base: ");
    Serial.print(base);
   /* Serial.print("\t y: ");
    Serial.print(ypr[0] * 180 / M_PI, 4);*/
    Serial.print("\t p: ");
    Serial.print(ypr[1] * 180 / M_PI, 4);
    Serial.print("\t r: ");
    Serial.print(ypr[2] * 180 / M_PI, 4);
    /*Serial.print("Val1: ");
    Serial.print(val1);*/
     Serial.print("\t Val2: ");
     Serial.print(val2);
    // Serial.print("\t Val3: ");
     //Serial.print(val3);
     Serial.print("\t Val4: ");
     Serial.print(val4);
    Serial.print("\t pkp: ");
    Serial.print(pkp);
    Serial.print("\t pki: ");
    Serial.println(pki);
    /*Serial.print("\t pkd: ");
    Serial.println(pkd);*/


  }
}

//safety button loop, if 999 code is received, turn quadcopter off
void endProgram() {

  //indicate to the user that the endProgram() method has been called
  Serial.println("Program stopped!");
  
  //get quadcopter stuck in this loops
  while (true) {
    
  }
}

//for reading values from the radio
void radioRead() {

  //the radio has information available
  if (radio.available()) {

    // Dump the payloads until we've gotten everything
    unsigned long got_time;

    bool done = false;


    // Fetch the payload, and see if this was the last one.
    done = radio.read( &got_time, sizeof(unsigned long) );

    //store what the radio received in the kvals variable
    kvals = got_time;

    //pull the PID gain values from the received long
    pkd = kvals / 1000000;
    kvals = kvals - (pkd * 1000000);

    pki = kvals / 1000;
    kvals = kvals - (pki * 1000);

    pkp = kvals;
    kvals = kvals - pkp;


    //if the kill code is received...
    if (pkp == 999) {

      //turn off all motors
      val1 = 0;
      val2 = 0;
      val3 = 0;
      val4 = 0;
      s1.write(val1);
      s2.write(val2);
      s3.write(val3);
      s4.write(val4);

      //end program function, get stuck in a while loop
      endProgram();

    }

    //adjust the gain values to create decimals
    pkp = pkp / 100;
    pki = pki / 100;
    pkd = pkd / 100;

    //set the gain values for the pitch and roll PID object
    pitch.SetTunings(pkp, pki, pkd);
    roll.SetTunings(pkp, pki, pkd);

  }


}


