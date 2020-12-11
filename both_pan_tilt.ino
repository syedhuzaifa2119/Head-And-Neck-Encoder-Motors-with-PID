int val;
int encoder0PinA = 2;
int encoder0PinB = 3;
int encoder1PinA = 20;
int encoder1PinB = 21;
volatile long int encoder0Pos = 0;
volatile long int encoder1Pos = 0;
int encoder0PinALast = LOW;
int encoder1PinALast = LOW;
int in1=6;
int in2=7;
int in3=8;
int in4=9;
float speedvalue=0;
float anglevalue=0;
float anglevalue2=0;
int count;
int n = LOW;
int prepulses=0;
float encpulses;
float encpulses2;
int prevalue;
char Motor;
float angle1=0;
float angle2=0;
void setup() {
    Serial.begin (9600);
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  pinMode (encoder1PinA, INPUT);
  pinMode (encoder1PinB, INPUT);
  pinMode (in1,OUTPUT);
  pinMode (in2,OUTPUT);
  pinMode (in3,OUTPUT);
  pinMode (in4,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), encoder1, CHANGE);

}

void loop() {
// prevalue=encpulses;
 Serial.print(prevalue);
 Serial.print (encoder0Pos);
 Serial.print ("\n");
 Serial.print (encoder1Pos);
 Serial.print ("\n");
anglevalue=-180;
anglevalue2=-180;
encpulses=(anglevalue*(1600.0/360.0));
encpulses2=(anglevalue2*(1600.0/360.0));
      if(encpulses>0){
       motorClockwise(encpulses);
       if(prevalue<encpulses){
        Serial.print("hello");}
       }
       if(encpulses<0){
        motorCounterClockwise(encpulses);
        } 
        if(encpulses2>0){
       motortClockwise(encpulses2);
       }
       if(encpulses2<0){
        motortCounterClockwise(encpulses2);
        }
}
void encoder ()
{
    n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos++;
    } else {
      encoder0Pos--;//CCW
    }
} 
}
void encoder1 ()
{
    n = digitalRead(encoder1PinA);
  if ((encoder1PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Pos++;
    } else {
      encoder1Pos--;//CCW
    }
}
}
void motorClockwise(int pulses)
{
  while (pulses>encoder0Pos){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  if(pulses<encoder0Pos){break;}
  }
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  }
 void motorCounterClockwise(int pulses)
 {
  while (pulses<encoder0Pos){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  if(pulses>encoder0Pos){break;}
  }
    digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  }
   void motortCounterClockwise(int pulses)
 {
  while (pulses<encoder1Pos){
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  if(pulses>encoder1Pos){break;}
  }
 
    digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  }
  void motortClockwise(int pulses)
{
  while (pulses>encoder1Pos){
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  if(pulses<encoder1Pos){break;}
  }
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  }
