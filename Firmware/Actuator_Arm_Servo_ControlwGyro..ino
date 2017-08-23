/*BASED ON EXAMPLES:
* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

// Include libraries
#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

// ================================================================
// ===               MODES OF GYRO AQUISITION                   ===
// ================================================================


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float alpha=0.005; //filter coefficient
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

VectorInt16 aaReal0;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_prev;     // [x, y, z]            gravity-free accel sensor measurements previous
VectorInt16 aaWorld_curr;     // [x, y, z]            gravity-free accel sensor measurements previous
VectorInt16 vvWorld_curr;     // [x, y, z]            gravity-free accel sensor measurements previous
VectorInt16 vvWorld_prev;     // [x, y, z]            gravity-free accel sensor measurements previous
VectorInt16 ppWorld_curr;     // [x, y, z]            gravity-free accel sensor measurements previous
VectorInt16 aaReal_filt;      // [x, y, z]            gravity-free accel sensor measurements previous


unsigned long prevTime = millis();
unsigned long currTime;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===               SERVO VARIABLES                            ===
// ================================================================

Servo table_servo;  // create servo object to control a servo
Servo shoulder_servo;  // create servo object to control a servo
Servo elbow_servo;  // create servo object to control a servo
Servo wrist_servo;  // create servo object to control a servo

// twelve servo objects can be created on most boards

int pos1 = 90;    // variable to store the servo position
int pos2 = 90;
int pos3 = 90;
int pos4 = 90;
boolean stringComplete = false;
boolean stringStart = false;
const double SERIAL_BAUD = 115200;

String inputString = "";
int lims;

namespace serv_pos {
  float lim_s = 90;
  float lim_e = 90;
  float lim_w = 90;
  float lim_b = 90;
}

int move_to_pos(int pos, int lim, Servo servo_x){
  
  if (pos<lim) {
        pos=pos+1;
        servo_x.write(pos);              // tell servo to go to position in variable 'pos'
        delay(5);                       // waits 15ms for the servo to reach the position
  } else if (pos>lim) {
        pos=pos-1;
        servo_x.write(pos);
        delay(5); 
        }                      // waits 15ms for the servo to reach the positio};
       
        return pos;
}     

void manual_input(String input) {
  // process the manual request recieved via serial
  
  switch( input.charAt(0) ) {
    case 's' :
      // you sent v,###
      serv_pos::lim_s = input.substring(2).toFloat();
      Serial.println("shoulder set!");
      break;
  
    case 'e' :
      // you sent f,###
      serv_pos::lim_e = input.substring(2).toFloat();
       Serial.println("elbow set!");
      break;
      
    case 'w' :
      // you sent q,###
      serv_pos::lim_w = input.substring(2).toFloat();
      Serial.println("wrist set!");
      Serial.println(serv_pos::lim_w);
      break;

      case 'b' :
      // you sent q,###
      serv_pos::lim_b = input.substring(2).toFloat();
      Serial.println("base set!");
      break;
  }
}

int read_acc_real(int n){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    aaReal0=aaReal
    
    
  }
  
void get_serial() {
  // gets manual input from the serial port
  #if DEBUG
    Serial.println("getting serial input...");
  #endif

  boolean stringComplete = false;  // whether the string is complete

  while (Serial.available() > 0) {

    // get the new byte:
    char inChar = (char)Serial.read();
    
    #if DEBUG
      Serial.println(String(inChar));
    #endif
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      break;
    }
  }

  if ( stringComplete == true && inputString.length() > 0 ) {
    // if you have a non-blank input string and it's complete...
    #if DEBUG
      Serial.print("got serial input: ");
      Serial.print(inputString);
      Serial.println();
    #endif
    // process it with your manual_input function!
    manual_input(inputString);
    inputString = "";
  }
}

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // set servo pins
  shoulder_servo.attach(10);  // attaches the servo on pin 9 to the servo object
  elbow_servo.attach(11);  // attaches the servo on pin 9 to the servo object
  wrist_servo.attach(12);
  table_servo.attach(8);  // attaches the servo on pin 9 to the servo object

  Serial.begin(SERIAL_BAUD);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
 // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

 aaReal_filt.x=0;
 aaReal_filt.y=0;
 aaReal_filt.z=0;

 aaWorld_prev.x=0;
 aaWorld_prev.y=0;
 aaWorld_prev.z=0;
 
 vvWorld_curr.x=0;
 vvWorld_curr.y=0;
 vvWorld_curr.z=0;
 
 vvWorld_prev.x=0;
 vvWorld_prev.y=0;
 vvWorld_prev.z=0;
 
 ppWorld_curr.x=0;
 ppWorld_curr.y=0;
 ppWorld_curr.z=0;
}

void loop() {
  
  if (Serial.available() > 0) {
    get_serial();
  }
 
  if (!dmpReady) return;
  
 // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

   // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print(",");
    Serial.print(q.x);
    Serial.print(",");
    Serial.print(q.y);
    Serial.print(",");
    Serial.println(q.z);
    Serial.print("\n");

#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
  //  Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print(",");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print(",");
    Serial.println(euler[2] * 180 / M_PI);
    Serial.print("\n");

#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   // Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(",");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(",");
    Serial.println(ypr[2] * 180 / M_PI);
    Serial.print("\n");

#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//    Serial.print("areal,");

    aaReal.y=aaReal.y;
    aaReal.z=aaReal.z;
    aaReal_filt.x=(1-alpha)*aaReal_filt.x+(alpha)*aaReal.x;
    aaReal_filt.y=(1-alpha)*aaReal_filt.y+(alpha)*aaReal.y;
    aaReal_filt.z=(1-alpha)*aaReal_filt.z+(alpha)*aaReal.z;

    Serial.print(aaReal_filt.x);
    Serial.print(",");
    Serial.print(aaReal_filt.y);
    Serial.print(",");
    Serial.println(aaReal_filt.z);
    Serial.print("\n");

#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    //Serial.print("aworld\t");
    
    currTime = millis();
    aaWorld_curr=aaWorld;
    vvWorld_curr.x+= (aaWorld_curr.x)*(currTime - prevTime)/1000;
    ppWorld_curr.x =(vvWorld_curr.x +  vvWorld_prev.x)/2*(currTime - prevTime)/1000;

    vvWorld_prev=vvWorld_curr;
    aaWorld_prev=aaWorld_curr;

    Serial.print(aaWorld.x);
    Serial.print(",");
    Serial.print(aaWorld.y);
    Serial.print(",");
    Serial.println(aaWorld.z);
    Serial.print("\n");
#endif
  
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

  }
  pos1=move_to_pos(pos1,serv_pos::lim_s,shoulder_servo);
  pos2=move_to_pos(pos2,serv_pos::lim_e,elbow_servo);
  pos3=move_to_pos(pos3,serv_pos::lim_w,wrist_servo);
  pos4=move_to_pos(pos4,serv_pos::lim_b,table_servo);
}

   
