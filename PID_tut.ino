
//objective- write a program to impliment PID logic into a robot equipped with ultrasonic sensor facing forward to make it stop exactly 30 cm from a wall.
//say the distance measured is x, then the robot must move a distance of x-30. (current distance from wall - 30) is the actual current distance from set point (error)
//the code is meant for atmega328p (uno, pro mini). if you have a mega, then uncomment the pulseIn() line in trigger() function and comment out the 
// portion marked as INTERRUPT STUFF in setup and the ISR function after the loop

//reading pwms using interrupts: video explaining this : https://www.youtube.com/watch?v=bENjl1KQbvo . My code is slightly different from it but does the same job.
//#define the pins for left motor, right motor and the trigger for the ultrasonic sensor. 
#define LMR 2//left motor red wire
#define LMB 4//left motor black wire
#define RMR 5
#define RMB 7
#define TRIG 9//trigger pin

#define LMANALOG 3 //analogWrite pins for controlling motor speed. you can chose them to be whatever you want as long as they are actually analog pins
#define RMANALOG 5

#define dt 0.025 //cycle time in seconds. the maximum distance that the HC-SR04 can measure is 400cms, which means a total flight time of 800cms which takes about 23.5ms, I m round it off to 25ms
                //i haven't tested this but hopefully this works
#define LoopFreq 40 //1/dt = 40

//these gains aint loyal. find them yourself
#define Kp 1
#define Kd 0.1
#define Ki 0.1 

#define MAX_OUTPUT 255 //maximum output that can be sent to the motors

#define STOPPING_DISTANCE 30 //stopping distance from the wall in cms.

float lastError=0,integralError = 0;//the proper way of doing these things is to create a class for PID to hold the values in the memory but this is just a tutorial so it makes no difference if i use global variables instead.

//variables for reading pwm using interrupts.
volatile unsigned long timer[2];
volatile byte last_channel = 0;
volatile long input = 0;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(LMR,OUTPUT);
  pinMode(LMB,OUTPUT);
  pinMode(RMR,OUTPUT);   
  pinMode(RMB,OUTPUT);
  pinMode(TRIG,OUTPUT);

// INTERRUPT STUFF
  PCICR |= (1 << PCIE0);   
  PCMSK0 |= (1 << PCINT0); //pin change interrupt enabled on pin 8 of arduino uno (atmega328p)
//-------------------------------

  Serial.begin(9600); // for debugging.
}

void forward(int val)// hey I just met you and this is crazy! But here's my function, so call me maybe?
{
  digitalWrite(LMR,HIGH);
  digitalWrite(LMB,LOW);
  analogWrite(LMANALOG,val);
  digitalWrite(RMR,HIGH);
  digitalWrite(RMB,LOW);
  analogWrite(RMANALOG,val);
}

void back(int val) // hey I just met you and this is crazy! But here's my function, so call me maybe?
{
  digitalWrite(LMR,LOW);
  digitalWrite(LMB,HIGH);
  analogWrite(LMANALOG,val);
  digitalWrite(RMR,LOW);
  digitalWrite(RMB,HIGH);
  analogWrite(RMANALOG,val);
}

void stahp() // hey I just met you and this is crazy! But here's my function, so call me maybe?
{       
  digitalWrite(LMR,LOW);
  digitalWrite(LMB,LOW);
  digitalWrite(RMR,LOW);
  digitalWrite(RMB,LOW);
}

long microsecondsToCms(long microseconds) //function to convert time into distance(cms)
{
  return microseconds*0.017; //launde ko physics ati hai
}

int limiter(float x) //helper function for limiting the output
{
  if(x>MAX_OUTPUT)
  {
    return MAX_OUTPUT;
  }
  if(x< -MAX_OUTPUT)
  {
    return -MAX_OUTPUT;
  }
  return int(x);
}

int PID(float error)
{
  float Output,dError;
  dError = (error - lastError)*LoopFreq; //prefer multiplication over division for speed
  lastError = error;
  
  integralError += error*dt; //prefer var += change over var = var + change for speed

  Output = Kp*error + Kd*dError + Ki*integralError; //PID equation

  limiter(Output); //limit the output as well as convert it's data type to int
  
  return Output;
}

void trigger()
{
  digitalWrite(TRIG,LOW);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);//this shouldn't actually be required since digitalWrite already takes 12us to execute but whatever
  digitalWrite(TRIG,LOW);
//  input = pulseIn(8,HIGH,23500);
}

long now; //time keeping variable. its long for the reason that we are measuring time in milliseconds and we'd over-run the unsigned integer limit of 65k in about 65 seconds.

void loop()
{
  now = millis();//get time stamp
  float error = microsecondsToCms(input) - STOPPING_DISTANCE;//setpoint - current. here, setpoint - current would come out negative, hence, to flip the sign, i have subtracted setpoint from current instead of current from setpoint.
  trigger(); //trigger HCSR04
  
  int output = PID(error);//calculate PID

  if(output > 0) //figure out if the PID is trying to go forwards or backwards.
  {
    forward(output);
  }
  if(output < 0)
  {
    back(output);
  }
  if(output == 0)
  {
    stahp();
  }
  
  while(millis()-now<25); //make sure that we don't start the next loop before 25ms
}


ISR(PCINT0_vect) //interrupt service routine for port B (pins 8 to 13 on atmega328p)
{
  timer[0]=micros();//taking the time stamp at the exact moment the ISR was called is more accurate than taking it later.
  
  if(last_channel==0&& PINB & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //(PINB & B00000001) is equivalent to digitalRead(8) but faster
    last_channel=1;
    timer[1]=timer[0];           
  }
  else if(last_channel==1 && !(PINB & B00000001)) //when pin falls low,
  {
    last_channel=0;
    input=timer[0]-timer[1]; //subtract the last time stamp from current time.
  }
}





