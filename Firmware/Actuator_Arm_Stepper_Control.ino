#define IN1 22
#define IN2 24
#define IN3 26
#define IN4 28

int Steps = 0;

void setup()

{

Serial.begin(115200);

pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);

// delay(1000);

}

void loop() {

while (Serial.available()>0){
int st = Serial.parseInt();
if(Serial.read()=='\n'){

String message = "STEPS ";

message+=st;

Serial.println(message);

stepper(st);

delay(500);

digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}
}
}

void stepper(int doSteps){

int xw =abs(doSteps);

for (int x=0;x<xw;x++){

switch(Steps){

case 0:

digitalWrite(IN1, LOW);

digitalWrite(IN2, LOW);

digitalWrite(IN3, LOW);

digitalWrite(IN4, HIGH);

break;

case 1:

digitalWrite(IN1, LOW);

digitalWrite(IN2, LOW);

digitalWrite(IN3, HIGH);

digitalWrite(IN4, HIGH);

break;

case 2:

digitalWrite(IN1, LOW);

digitalWrite(IN2, LOW);

digitalWrite(IN3, HIGH);

digitalWrite(IN4, LOW);

break;

case 3:

digitalWrite(IN1, LOW);

digitalWrite(IN2, HIGH);

digitalWrite(IN3, HIGH);

digitalWrite(IN4, LOW);

break;

case 4:

digitalWrite(IN1, LOW);

digitalWrite(IN2, HIGH);

digitalWrite(IN3, LOW);

digitalWrite(IN4, LOW);

break;

case 5:

digitalWrite(IN1, HIGH);

digitalWrite(IN2, HIGH);

digitalWrite(IN3, LOW);

digitalWrite(IN4, LOW);

break;

case 6:

digitalWrite(IN1, HIGH);

digitalWrite(IN2, LOW);

digitalWrite(IN3, LOW);

digitalWrite(IN4, LOW);

break;

case 7:

digitalWrite(IN1, HIGH);

digitalWrite(IN2, LOW);

digitalWrite(IN3, LOW);

digitalWrite(IN4, HIGH);

break;

default:

digitalWrite(IN1, LOW);

digitalWrite(IN2, LOW);

digitalWrite(IN3, LOW);

digitalWrite(IN4, LOW);

break;

}

delay(1);

if(doSteps>=0){ Steps++;}

if(doSteps<0){ Steps--; }

if(Steps>7){Steps=0;}

if(Steps<0){Steps=7; }

}

}
