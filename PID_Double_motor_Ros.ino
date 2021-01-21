#include <PID_v1.h>                       //PID Library
#include <ros.h>                          //Header Files of Ros 
#include <std_msgs/String.h>              //Header Files for Msgs
ros::NodeHandle  nh;                      //intiate node handle to create publisher and subscriber
String x;
//Define PWM and Control Pins for Both motors

#define MotEnable 4                       //PAN motor PWM Pin
#define MotFwd  6                         //Pan motor CW
#define MotRev  7                         //Pan motor CCw
#define Mot2Enable 5                      //Tilt motor PWM Pin
#define Mot2Fwd 8                         //Tilt motor CW
#define Mot2Rev  9                        //Tilt motor CCw
String readStrings;                       //String to store input Value form ROS Publisher
char *ptr;                      
int User_Input1=0;                        //To store PAN Angle 
int User_Input2=0;                        //To store Tilt Angle 
//Encoder Variables
int encoderPin1 = 2;                      //Channel A Pan Motor 
int encoderPin2 = 3;                      //Channel B Pan Motor
int encoder2Pin1 = 20;                    //Channel A Tilt Motor 
int encoder2Pin2 = 21;                    //Channel B Tilt Motor

volatile int lastEncoded = 0; 
volatile long encoderValue = 0; 
volatile int lastEncoded2 = 0; 
volatile long encoderValue2 = 0; 
volatile long encoder_Prev=0;
volatile long encoder_Prev2=0;
volatile long dt_Value=0;                //Measures the Difference b/w Desired Angle and Angle Achieved by Pan motor 
volatile long dt_Value2=0;               //Measures the Difference b/w Desired Angle and Angle Achieved by tilt motor 
int count=0;
int PPR = 6400;                          // Encoder Pulse per revolution.
int angle = 360; 
int REV1 = 0;    
int REV2 = 0;                            // Set point REQUIRED ENCODER VALUE
int lastMSB = 0;
int lastLSB = 0;
int lastMSB2 = 0;
int lastLSB2 = 0;
char yourstring[32];
//double kp = 0.1250 , ki = 0.55, kd = 0.0002;//Pan Parameter with load(Belt mounted)
double kp = 0.3550 , ki = 0.25, kd = 0.0529;//Pan Parameter ok0,0009
double kp2 = 0.1450 , ki2 = 0.25, kd2 = 0.0529;//Tilt Parameters with tilt(Camera mounted)                 
double input = 0, output = 0,input2 = 0, output2 = 0, setpoint[3];
PID myPID(&input, &output, &setpoint[1], kp, ki, kd, DIRECT);             //Intiallizing PID Controller I/O parameter For PAN Motor
PID myPID2(&input2, &output2, &setpoint[2], kp2, ki2, kd2, DIRECT);       //Intiallizing PID Controller I/O parameter For Tilt Motor

//Call Back Fucntion For Subscriber 
void messageCb( const std_msgs::String& msg){
      x=msg.data;
      digitalWrite(LED_BUILTIN,HIGH-digitalRead(LED_BUILTIN));
}


ros::Subscriber<std_msgs::String> s("your_topic", &messageCb);          //Initialize Subscriber we will be using   

void setup() {
    nh.initNode();                                                     //Initilaize the node handle 
    nh.subscribe(s);                                                   //Subscribe Any Topic Being Published 
    
   //Motor Control Pins Configuration 
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(MotEnable, OUTPUT);
    pinMode(MotFwd, OUTPUT); 
    pinMode(MotRev, OUTPUT); 
    pinMode(Mot2Enable, OUTPUT);
    pinMode(Mot2Fwd, OUTPUT); 
    pinMode(Mot2Rev, OUTPUT); 
    Serial.begin(57600);                                               //Baud Rate
   //Defining Encoder Pins Configuration 
    pinMode(encoderPin1, INPUT_PULLUP); 
    pinMode(encoderPin2, INPUT_PULLUP);
    pinMode(encoder2Pin1, INPUT_PULLUP); 
    pinMode(encoder2Pin2, INPUT_PULLUP);
   //turn pullup resistor on  
    digitalWrite(encoderPin1, HIGH);  
    digitalWrite(encoderPin2, HIGH);
    digitalWrite(encoder2Pin1, HIGH); 
    digitalWrite(encoder2Pin2, HIGH);
    
   //Selected pins for interrupt 
    attachInterrupt(0, updateEncoder, CHANGE); 
    attachInterrupt(1, updateEncoder, CHANGE); 
    attachInterrupt(3, updateEncoder2, CHANGE); 
    attachInterrupt(2, updateEncoder2, CHANGE);  
    
    TCCR1B = TCCR1B & 0b11111000 | 1;  
    
    
    myPID.SetMode(AUTOMATIC);   
    myPID.SetSampleTime(50);                      //Refresh Rate for PD Controller
    myPID.SetOutputLimits(-70,70); //for pan
    myPID2.SetMode(AUTOMATIC);   
    myPID2.SetSampleTime(50);                  
    myPID2.SetOutputLimits(-60,75);//for tilt 
}
void loop() {
  
    // while (Serial.available()) {          
        // char c = Serial.read();  
        // readStrings += c;          
    //  }
    // 
    // if (readStrings.length() >0) { 
        // Serial.println(readStrings.toInt());  
        // User_Input = readStrings.toInt();  
    //}
    
    readStrings=x;                                //Stores the Publised Data
    nh.spinOnce();                                //To process ROS Callbacks
    delay(1);
    //Serial.print(testString);
    //while (Serial.available()) { //Check if the serial data is available.              
    //char c = Serial.read(); 
    //yourstring[count] = c;         
    //count++;
    //}
    if (readStrings.length() >0) {               //Checks for Data in Published Data on Topic
         
        int str_len=readStrings.length()+1;      
        readStrings.toCharArray(yourstring,str_len);    //Converts String To Char Arrat    
           //To seperate Angles seperated by comma and to Convert them to int
        ptr = strchr(yourstring, ',');                  
        *ptr = '\0';
          //convert to integer,  first character
        User_Input1 = atoi(&yourstring[0]);
        *ptr = ',';
          // increment pointer to point to character after comma
        ptr++;
          // convert to integer
        User_Input2 = atoi(ptr);
        //Serial.println(User_Input1);
        //Serial.print('\n');
        //Serial.print(User_Input2);
        //delay(10);
         
       }
    dt_Value=abs(encoderValue-encoder_Prev);          //Measures the Difference b/w Desired Angle and Angle Achieved by Pan motor 
    dt_Value2=abs(encoderValue2-encoder_Prev2);       //Measures the Difference b/w Desired Angle and Angle Achieved by Pan motor 
    REV1 = map(User_Input1, 0, 360, 0, 6400);         //Maps the user input of no. of encoder pulses to achieve
    REV2 = map(User_Input2, 0, 360, 0, 6400); 
    //delay(10);               
    setpoint[1] = REV1;                               //Target Value for PID Controller
    setpoint[2] = REV2;                   
    input = encoderValue ;                            //Input Value to PID Conttroller
    input2 = encoderValue2;      
    myPID.Compute();                                  //Compute PAN PID Controller Output
    myPID2.Compute();                                 // calculate new output for Tilt Motor
    pwmOut(output);                                   //Gives Controiller output to motors
    pwmOut2(output2);
    //Serial.print("REV:"); 
    //Serial.print(REV1);
    //Serial.print("  ");
    //Serial.print("encoderValue:");
    //Serial.print(encoderValue);
    //Serial.print("  ");
    //Serial.print("REV2:"); 
    //Serial.print(REV2);
    //Serial.print("  ");
    //Serial.print("encoder2Value:");
    //Serial.print(encoderValue2); 
    //Serial.print("  ");
    //Serial.println("");
    encoder_Prev=encoderValue;                      //To Strore Prev Value
    encoder_Prev2=encoderValue2;
}
// PWM Ouput to motors
void pwmOut(int out){                               
  if (out > 0) {                        
      analogWrite(MotEnable, out);         
      forward();                          
  }
  else {
      analogWrite(MotEnable, abs(out));                                
      reverse();                           
  }
 //yourstring[0]=""; // Cleaning User input, ready for new Input
}
void pwmOut2(int out2){                               
  if (out2 > 0) {                        
      analogWrite(Mot2Enable, out2);         
      forward2();                          
  }
  else {
      analogWrite(Mot2Enable, abs(out2));                                
      reverse2();                           
  }
 //yourstring[1]=""; // Cleaning User input, ready for new Input
}
//Encoder Pulses Count for Pan Motor
void updateEncoder(){
    int MSB = digitalRead(encoderPin1); 
    int LSB = digitalRead(encoderPin2);
    int encoded = (MSB << 1) |LSB; 
    int sum  = (lastEncoded << 2) | encoded; 
  
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
  
    lastEncoded = encoded; //store this value for next time
    
}
//Encoder Pulses Count for Tilt motor
void updateEncoder2(){
    int MSB2 = digitalRead(encoder2Pin1); 
    int LSB2 = digitalRead(encoder2Pin2);
    int encoded2 = (MSB2 << 1) |LSB2; 
    int sum2  = (lastEncoded2 << 2) | encoded2; 
  
    if(sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2 ++;
    if(sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2 --;
  
    lastEncoded2 = encoded2; //store this value for next time
}
void forward () {
     digitalWrite(MotFwd, HIGH); 
     digitalWrite(MotRev, LOW); 
}
void reverse () {
     digitalWrite(MotFwd, LOW); 
     digitalWrite(MotRev, HIGH); 
  
}
void finish () {
     digitalWrite(MotFwd, LOW); 
     digitalWrite(MotRev, LOW); 
  
}
void forward2 () {
     digitalWrite(Mot2Fwd, HIGH); 
     digitalWrite(Mot2Rev, LOW); 
  
}
void reverse2 () {
     digitalWrite(Mot2Fwd, LOW); 
     digitalWrite(Mot2Rev, HIGH); 
}
void finish2 () {
     digitalWrite(Mot2Fwd, LOW); 
     digitalWrite(Mot2Rev, LOW);  
}
