//https://www.youtube.com/channel/UCnu1c_5OqBfo7j2bG_4IYow
//https://www.youtube.com/watch?v=xCJw69S169M video of PID controlled LFR
//Followe ZM Networks and support the Channel.
#include <QTRSensors.h>

#define Kp 0.02// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed  
#define Kd 3
// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define Ki 0
#define MaxSpeed 100 // max speed of the robot
#define BaseSpeed 70 // this is the speed at which the motors should spin when the robot is perfectly on the line
//#define NUM_SENSORS  8     // number of sensors used

#define speedturn 80

#define rightMotor1 A1
#define rightMotor2 A2
#define rightMotorPWM 10
#define leftMotor1 A4
#define leftMotor2 A5
#define leftMotorPWM 11


QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//QTRSensorsRC qtrrc((unsigned char[]) {2,3,4,5,6,7,8,9} ,NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

//unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  qtr.setEmitterPin(12);
  
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  delay(3000);
    Serial.begin(9600);
    Serial.println();

 // sensor_calibrate();

int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  {
  
   //comment this part out for automatic calibration 
    if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
    {
      move(1, 70, 1);//motor 
      move(0, 70, 0);//motor  
    }
    else
    {
      move(1, 70, 0);//motor 
      move(0, 70, 1);//motor   
    }
    qtr.calibrate();   
    delay(20);
  
  }
  wait();
  delay(3000); // wait for 3s to position the bot before entering the main loop 

}  

int lastError = 0;
//unsigned int sensors[8];
uint16_t position = qtr.readLineBlack(sensorValues);

  
void loop()
{  
  position = qtr.readLineBlack(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
/*  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  */
 if(position>6500){
    move(1, speedturn, 1);//motor 
    move(0, speedturn, 0);//motor 
    return;
  }
  if(position<500){ 
    move(1, speedturn, 0);//motor 
    move(0, speedturn, 1);//motor 
    return;
  } 

  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) {rightMotorSpeed = MaxSpeed;} // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) {leftMotorSpeed = MaxSpeed;} // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)  {rightMotorSpeed = 0;}    
  if (leftMotorSpeed < 0) {leftMotorSpeed = 0;}
    
  move(1, rightMotorSpeed, 1);//motor 
  move(0, leftMotorSpeed, 1);//motor 
  
}
  
void wait(){

  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
  
}

void move(int motor, int speed, int direction){
  
  boolean inPin1;
  boolean inPin2;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}

void sensor_calibrate()
{

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);  
  
  
}
