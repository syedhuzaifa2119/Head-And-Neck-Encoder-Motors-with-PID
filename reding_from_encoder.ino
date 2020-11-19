int val;
int encoder0PinA = 2;
int encoder0PinB = 3;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int in1=6;
int in2=7;
float speedvalue=0;
float anglevalue=0;
int count;
int n = LOW;
float encpulses=0;

void setup() {
    Serial.begin (9600);
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  pinMode (in1,INPUT);
  pinMode (in2,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), encoder, CHANGE);
}

void loop() {

  Serial.print (encoder0Pos);
 Serial.print ("\n");
     if (Serial.available()>0)
    {
      anglevalue=0;
   
      Serial.print("Enter the Rotation Angle ");
      anglevalue=Serial.parseInt();
      Serial.print("The Angle is ");
      Serial.println(anglevalue);
      encpulses=(anglevalue*(1600.0/360.0))-100;
      Serial.print("No. of encoder pulses");
      Serial.println(encpulses);
      
      }
       motorClockwise(encpulses);
//      if(encpulses>0){
//         motorClockwise(encpulses);
//       }
//        if(encpulses<0){
//          motorCounterClockwise(encpulses);
//          }
//       

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
// void motorCounterClockwise(int pulses)
// {
//  while (pulses<encoder0Pos){
//  digitalWrite(in1,LOW);
//  digitalWrite(in2,HIGH);
//  if(pulses>encoder0Pos){break;}
//  }
// 
//    digitalWrite(in1,LOW);
//  digitalWrite(in2,LOW);
//  }
