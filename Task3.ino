/*
* This Program is to make the robot move inside a box of black
* boundary and white surface while avoiding obstacles
*/

// array to store all distance sensor values
int distance[5] = {};

long randNumber;

// Threshold value to distinguish between black and white surface, 
// anything above 880 sensor value will be considered black and below 880 as white
int Threshold_T3 = 880;

// Threshold value to distinguish between obstacle and no obstacle, 
// anything below 650 sensor value will be considered obstacle and
// above 650 as free space
int DistThreshold_T3 = 650;
  
void TASK_3() {

  // put your main code here, to run repeatedly:
  // First step is to update the ground sensor values by reading the 
  // ground sensors
  UpdateGroundSensors_T3();

  // ground array is filled with values of all sensors,
  // starting from ground[0] to ground[3] values are 
  // assigned to sensors from left to right i.e. 4 sensors
  LeftMostSensor = ground[0];
  LeftSensor = ground[1];
  RightSensor = ground[2];
  RightMostSensor = ground[3];

  // This function will read all distance sensor values
  UpdateDistanceSensors_T3();

  // Based on the values of distance and ground sensors,
  // we will set the robot state of motion
  ProcessRobotState_T3(LeftMostSensor, LeftSensor, RightSensor, RightMostSensor, distance); //give us state   

}

  // Updates values of all ground sensors
  // and makes them binary based on threshold
  // value. anything greater than threshold is
  // 1 and lower than threshold is 0.
void UpdateGroundSensors_T3() {
  for (int i = 0; i<=3; i++) {
     ReadGroundSensor_T3(i);
     ConvertSensorToBinary_T3(i);
  }
}

// Updates values of all distance sensors
// and makes them binary based on threshold
// value. anything greater than threshold is
// 0 and lower than threshold is 1.
void UpdateDistanceSensors_T3()
{
  for (int i = 0; i<8; i++) {
    if(i==3 | i==4 | i==5)
      continue;
     
     ReadDistanceSensor_T3(i);
     
     if(i==6 | i==7)
     {
     ConvertDistSensorToBinary_T3(i-3);
    }
    else
    {
      ConvertDistSensorToBinary_T3(i);
    }
  }
}

// Reads the Distance Sensor whose number is provided e.g 0, 1, 2 or 3
void ReadDistanceSensor_T3(int sensornum) {
  // Turn on respective IR Led so that light shines away from robot
  // and we are able to check if light comes back after hitting 
  // something that means obstacle in front.
  proxLEDon_T3(sensornum);
  delay(10);

  // Stores value received from analogReading the sensor in RawSensorValue Variable
  RawSensorValue = analogRead(sensornum); //corresponding ground sensor
 
//  Serial.print(sensornum);
//  Serial.print("  ");
//  Serial.println(RawSensorValue);

  // Turn off IR led after reading the sensor
   proxLEDoff_T3(sensornum);
  
}

// This function checks value in RawSensorValue Variable,
// if that is less than DistThreshold value then distance sensors
// array's corresponding value is changed to 1, if not then it is
// given 0 value.
void ConvertDistSensorToBinary_T3(int sensornum) {
  if (RawSensorValue < DistThreshold_T3) {
    distance[sensornum] = 1; //obstacle
  }
  else {
    distance[sensornum] = 0;
  }
  
}

// Reads the Ground Sensor whose number is provided e.g 0, 1, 2 or 3
void ReadGroundSensor_T3(int sensornum) {
  // Turn on respective IR Led so that light shines on surface
  // and we are able to read light coming back from ground.
  GroundLedOn_T3(sensornum);
  delay(10);

  // Stores value received from analogReading the sensor in RawSensorValue Variable
  // Adding 8 to sensornumber as ground sensors are attached to pins 8,9,10 and 11
  RawSensorValue = analogRead(sensornum+8); //corresponding ground sensor
  
//  Serial.print(sensornum);
//  Serial.print("  ");
//  Serial.println(RawSensorValue);

  // Turn off IR led after reading the sensor
  GroundLedOff_T3(sensornum);
  
}

// This function checks value in RawSensorValue Variable,
// if that is greater than Threshold value then ground sensors
// array's corresponding value is changed to 1, if not then it is
// given 0 value.
void ConvertSensorToBinary_T3(int sensornum) {
  if (RawSensorValue > Threshold_T3) {
    ground[sensornum] = 1;
  }
  else {
    ground[sensornum] = 0;
  }
  
}

// This function decides the movement of robot after checking values of all 4 ground sensors
// and front + sides distance sensors.
void ProcessRobotState_T3(bool LeftMostSensor, bool LeftSensor, bool RightSensor, bool RightMostSensor, int distance[5]) {
// if any of the ground sensors detect black boundary, Robot will stop,
// move back a little and Take a Right turn of random value.  
  if((LeftMostSensor == 1) | (LeftSensor == 1) | (RightMostSensor == 1) | (RightSensor == 1))
  {
    //MotorsStop_T3();
    //delay(500);
    //MotorsBackward_T3();
    //delay(200);
    //MotorsStop_T3();
    //delay(400);
    randNumber = random(350, 450);
    RightTurnBothMotors_T3();
    delay(randNumber);
    MotorsStop_T3();
    delay(200);
  }
  // if any of the distance sensors detect obstacle, Robot will
  // move back a little and Take a Right turn of random value
  else if(distance[0] == 1 | distance[1] == 1 | distance[2] == 1 | distance[3] == 1 | distance[4] == 1)
  {
    MotorsBackward_T3();
    delay(150);
   MotorsStop_T3();
   delay(400); 
   MotorsBackward_T3();
    delay(200);
    MotorsStop_T3();
    delay(400);
    randNumber = random(300, 450);
    RightTurnBothMotors_T3();
    delay(randNumber);
    MotorsStop_T3();
    delay(200);
  }
  // in other case i.e all ground sensors on white and distance sensors
  // dont see any obstacle robot will move forward
  else
  {
    MotorsForward_T3();
  }
}

// Turn Robot Left by moving Right wheel Forward
void LeftTurn_T3() {
  LeftMotorForward_T3(0);
  RightMotorForward_T3(13);
}

// Turn Robot Right by moving Left wheel Forward and right backward
void RightTurnBothMotors_T3(){
  analogWrite(5, 0);
  analogWrite(2,12 *2.55);
  analogWrite(6, 8 * 2.55);
  analogWrite(7,0);
}

// Turn Robot Right by moving Left wheel Forward
void RightTurn_T3() {
  LeftMotorForward_T3(13);
  RightMotorForward_T3(0);
}

// Stop both wheels by giving no power to motors
void MotorsStop_T3(){
  analogWrite(2,255);
  analogWrite(5,255);
  analogWrite(6,255);
  analogWrite(7,255);
}

// Function that makes Left wheel move Forward
void LeftMotorForward_T3(float duty) {
  analogWrite(6, duty * 2.55);
  analogWrite(7,0);
}

// Function that makes Right wheel move Forward
void RightMotorForward_T3(float duty) {
  analogWrite(5, duty *2.55);
  analogWrite(2,0);
}

// This function makes both wheels move Forward
void MotorsForward_T3() {
  LeftMotorForward_T3(6.7);
  RightMotorForward_T3(9);
}

// This function makes both wheels move Backward
void MotorsBackward_T3() {
  analogWrite(7, 18);
  analogWrite(6,0);
  analogWrite(2, 23);
  analogWrite(5,0);
}

// Turns the IR led off of ground sensor whose Index is provided
void GroundLedOff_T3(unsigned char LedIndex) {
  if ((LedIndex ==0) |(LedIndex ==1) | (LedIndex ==2)| (LedIndex ==3)){
    PORTJ |= (1<<LedIndex);
  }
 }

// Turns the IR led on of ground sensor whose Index is provided 
void GroundLedOn_T3(unsigned char LedIndex) {
  if ((LedIndex ==0) |(LedIndex ==1) | (LedIndex ==2)| (LedIndex ==3)){
    PORTJ &= ~(1<<LedIndex);
  }
 }

// Turns the IR led of Distance sensor on whose Index is provided
 void proxLEDon_T3(unsigned char proxIndex)
{
  PORTA |= (1<<proxIndex);
}

// Turns the IR led off of Distance sensor on whose Index is provided
void proxLEDoff_T3(unsigned char proxIndex)
{
  PORTA &= ~(1<<proxIndex);
}
