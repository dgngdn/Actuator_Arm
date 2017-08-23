/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo table_servo;  // create servo object to control a servo
Servo shoulder_servo;  // create servo object to control a servo
Servo elbow_servo;  // create servo object to control a servo
Servo wrist_servo;  // create servo object to control a servo

// twelve servo objects can be created on most boards

int pos1 = 90;    // variable to store the servo position
int pos2 = 90;
int pos3 = 90;
int pos4 = 50;
int analogPin=3;
boolean stringComplete = false;
boolean stringStart = false;
const double SERIAL_BAUD = 38400;
//byte PWM_PIN = 3;
//int pwm_value;

String inputString = "";
int lims;

namespace serv_pos {
  float lim_s = 90;
  float lim_e = 90;
  float lim_w = 90;
  float lim_b = 50;
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

//int manual_input(String in_str){
//    if (in_str.substring(0,4)=="rest"){
//       serv_pos::lim_s=30;
//       serv_pos::lim_e=30;
//       serv_pos::lim_w=90;
//       serv_pos::lim_b=90;
//       Serial.println("Resting...");}
//    else if (in_str.substring(0,4)=="wake"){
//       serv_pos::lim_s=90;
//       serv_pos::lim_e=90;
//       serv_pos::lim_w=90;
//       serv_pos::lim_b=90;}
//    else {
//      serv_pos::lim_s=in_str.substring(0,3).toFloat();
//      serv_pos::lim_e=in_str.substring(3,6).toFloat();
//      serv_pos::lim_w=in_str.substring(6,9).toFloat();
//      serv_pos::lim_b=in_str.substring(9,12).toFloat();
//      Serial.println("interpreted");}
//}


//int get_serial() {
//   while (Serial.available() > 0){
//   
//    char inChar = (char)Serial.read();
//
//    inputString += inChar;
//    Serial.print(inputString);
// 
//    if (inChar == '\n') {
//      stringComplete = true;
//      break;
//    }
//   
//
//    if (stringComplete == true && inputString.length() > 0 ) {
//    // if you have a non-blank input string and it's complete...
//    manual_input(inputString);
//    inputString = "";
//    stringComplete = false;
//  }
//}
//}

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
  shoulder_servo.attach(10);  // attaches the servo on pin 9 to the servo object
  elbow_servo.attach(11);  // attaches the servo on pin 9 to the servo object
  wrist_servo.attach(12);
  table_servo.attach(8);  // attaches the servo on pin 9 to the servo object
//  pinMode(PWM_PIN, INPUT);

  Serial.begin(SERIAL_BAUD);
}

void loop() {
  
  if (Serial.available() > 0) {
    get_serial();
  }
//    Serial.println(serv_pos::lim_s);
//    Serial.println(serv_pos::lim_e);
//    Serial.println(serv_pos::lim_w);
//    Serial.println(serv_pos::lim_b);
//    Serial.println();

  pos1=move_to_pos(pos1,serv_pos::lim_s,shoulder_servo);
  pos2=move_to_pos(pos2,serv_pos::lim_e,elbow_servo);
  pos3=move_to_pos(pos3,serv_pos::lim_w,wrist_servo);
  pos4=move_to_pos(pos4,serv_pos::lim_b,table_servo);

  //pwm_value = pulseIn(PWM_PIN, HIGH);
  Serial.println(analogRead(analogPin));
  delay(30);
 
}

   
