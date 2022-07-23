/*
 * This Program is to make the robot move in a given arena with
 * line following, wall following and obstacle avoidance.
 */


// Variable to check count of junctions
int count = 0;

// Variables to differentiate between clockwise and counter cw
int cw = 0;
int ccw = 0;

  // Threshold value to distinguish between black and white surface, 
  // anything above 780 sensor value will be considered black and below 780 as white
  int Threshold_T4 = 780;

  // Threshold value to distinguish between obstacle and no obstacle, 
  // anything below 580 sensor value will be considered obstacle and
  // above 580 as free space
  int DistThreshold_T4 = 580;


void TASK_4() {

  // First step is to update the ground sensor values by reading the 
  // ground sensors
  UpdateGroundSensors();

  // ground array is filled with values of all sensors,
  // starting from ground[0] to ground[3] values are 
  // assigned to sensors from left to right i.e. 4 sensors
  LeftMostSensor = ground[0];
  LeftSensor = ground[1];
  RightSensor = ground[2];
  RightMostSensor = ground[3];

  // Based on the values of distance and ground sensors,
  // we will set the robot state of motion
  ProcessRobotState(LeftMostSensor, LeftSensor, RightSensor, RightMostSensor); //give us state  
  
}

// This function is useful when robot is crossing juntion
// as at that time we do not want junction count to keep increasing
void OnlyLineFollowingLast()
{
  for(int i=0; i<20; i++)
  {
    // Reading ground sensors again
    UpdateGroundSensors();
    
    LeftMostSensor = ground[0];
    LeftSensor = ground[1];
    RightSensor = ground[2];
    RightMostSensor = ground[3];

  // if all ground sensors detect black and robot is at end
  // then stop
  if ((LeftSensor == 1) && (RightSensor == 1)  && (LeftMostSensor == 1) && (RightMostSensor == 1) && count >4)
  {
    MotorsStop();
    delay(900);
  }
  // if front sensors see white move forward
   else if ((LeftSensor == 0) && (RightSensor == 0)) {
      MotorsForwardSlow();
    }
  
    // if front left sensor is on black and front right on white move left
    else if ((LeftSensor == 1) && (RightSensor == 0)) {
      LeftTurn();
    }
  
    // if front left sensor is on white and front right on black move right
    else if ((LeftSensor == 0) && (RightSensor ==1)) {
      RightTurn();
    }
  
    // in any other case also move forward
     else {
      MotorsForwardSlow();
    }
  }

  // Turn on RGB led
  digitalWrite(12, HIGH);
}


// This function is useful when robot is crossing juntion and black line following
// as at that time we do not want junction count to keep increasing
void OnlyLineFollowing()
{
  for(int i=0; i<15; i++)
  {
    UpdateGroundSensors();
    
    LeftMostSensor = ground[0];
    LeftSensor = ground[1];
    RightSensor = ground[2];
    RightMostSensor = ground[3];
  if ((LeftSensor == 1) && (RightSensor == 1)  && (LeftMostSensor == 1) && (RightMostSensor == 1) && count >4)
  {
    MotorsStop();
    delay(900);
  }
   else if ((LeftSensor == 0) && (RightSensor == 0)) {
      MotorsForward();
    }
  
    // if front left sensor is on black and front right on white move left
    else if ((LeftSensor == 1) && (RightSensor == 0)) {
      LeftTurn();
    }
  
    // if front left sensor is on white and front right on black move right
    else if ((LeftSensor == 0) && (RightSensor ==1)) {
      RightTurn();
    }
  
    // in any other case also move forward
     else {
      MotorsForward();
    }
  }
  digitalWrite(12, HIGH);
}

// Updates values of all ground sensors
// and makes them binary based on threshold
// value. anything greater than threshold is
// 1 and lower than threshold is 0. 
void UpdateGroundSensors() {
  for (int i = 0; i<=3; i++) {
     ReadGroundSensor(i);
     ConvertSensorToBinary(i);
  }
}

// Reads the Ground Sensor whose number is provided e.g 0, 1, 2 or 3
void ReadGroundSensor(int sensornum) {
  // Turn on respective IR Led so that light shines on surface
  // and we are able to read light coming back from ground.
  GroundLedOn(sensornum);
  delay(10);

  // Adding 8 to sensornumber as ground sensors are attached to pins 8,9,10 and 11
  RawSensorValue = analogRead(sensornum+8); //corresponding ground sensor
  
//  Serial.print(sensornum);
//  Serial.print("  ");
//  Serial.println(RawSensorValue);

  // Turn off IR led after reading the sensor
  GroundLedOff(sensornum);
  
}

// This function checks value in RawSensorValue Variable,
// if that is greater than Threshold value then ground sensors
// array's corresponding value is changed to 1, if not then it is
// given 0 value.
void ConvertSensorToBinary(int sensornum) {
  if (RawSensorValue > Threshold_T4) {
    ground[sensornum] = 1;
  }
  else {
    ground[sensornum] = 0;
  }
  
}

// This function decides the movement of robot after checking values of all 4 ground sensors
void ProcessRobotState(bool LeftMostSensor, bool LeftSensor, bool RightSensor, bool RightMostSensor) {
  if(RightMostSensor == 1 || LeftMostSensor == 1) {

// if first junction is detected by right sensor it means robot
// has to go on clockwise path
    if(RightMostSensor == 1 && count == 1)
      {cw = 1;}
      
// if first junction is detected by left sensor it means robot
// has to go on counter-clockwise path
    if(LeftMostSensor == 1 && count == 1)
      {ccw = 1;}

 // turn off rgb led
 digitalWrite(12, LOW);   

     // for clockwise on first junction take right turn
     if(count == 1 && cw ==1)
     {
      MotorsForward();
      delay(60);
       RightTurn();
       delay(320);
        MotorsStop();
      delay(400);
      while(analogRead(10) < Threshold_T4)
      { RightTurn(); }
      while(analogRead(9) < Threshold_T4)
      { RightTurn(); }
      
     }
     // for clockwise on third junction take left turn
     else if(count == 3 && cw ==1)
      {
        MotorsForward();
      delay(80);
         LeftTurn();
         delay(900);
          MotorsStop();
        delay(400);
        while(analogRead(9) < Threshold_T4)
        { LeftTurn(); }
        while(analogRead(10) < Threshold_T4)
        { LeftTurn(); }     
       }
       // for clockwise on fourth junction take right turn
       else if(count == 4 && cw ==1)
     {
      MotorsForward();
      delay(60);
       RightTurn();
       delay(320);
        MotorsStop();
      delay(400);
      while(analogRead(10) < Threshold_T4)
      { RightTurn(); }
      while(analogRead(9) < Threshold_T4)
      { RightTurn(); }
      
      MotorsStop();
      delay(500);
     }

     // for counter-clockwise on first junction take left turn
     if(count == 1 && ccw ==1)
     {
      MotorsForward();
      delay(60);
         LeftTurn();
         delay(1000);
          MotorsStop();
        delay(400);
        while(analogRead(9) < Threshold_T4)
        { LeftTurn(); }
        while(analogRead(10) < Threshold_T4)
        { LeftTurn(); }
     }
     // for counter-clockwise on third junction take right turn
     else if(count == 3 && ccw ==1)
      {
      MotorsForward();
      delay(60);
       RightTurn();
       delay(320);
        MotorsStop();
      delay(400);
      while(analogRead(10) < Threshold_T4)
      { RightTurn(); }
      while(analogRead(9) < Threshold_T4)
      { RightTurn(); }
             
       }
       // for counter-clockwise on fourth junction take left turn
       else if(count == 4 && ccw ==1)
     {
      MotorsForward();
      delay(30);
         LeftTurn();
         delay(1000);
          MotorsStop();
        delay(400);
        while(analogRead(9) < Threshold_T4)
        { LeftTurn(); }
        while(analogRead(10) < Threshold_T4)
        { LeftTurn(); }
     }
     // increase value of count variable by one
    count++;

    // After detecting a junction move to only line following
    // code so that junction count value does not keeps increasing
    OnlyLineFollowing();
    MotorsStop();
    delay(200);
  }

    // if junction count becomes greater than total junctions in path
    // start wall following
if(count > 4)
     {
      while(1)
      {
        Wallfollowing();
        OnlyLineFollowingLast();
      }
     }
  // if both extreme sensors(ground) detect black move straight
  if (RightMostSensor == 1 && LeftMostSensor == 1) {
    MotorsForward();
  }
  
  // if both front sensors are on white move straight
  else if ((LeftSensor == 0) && (RightSensor == 0)) {
    MotorsForward();
  }

  // if front left sensor is on black and front right on white move left
  else if ((LeftSensor == 1) && (RightSensor == 0)) {
    LeftTurn();
  }

  // if front left sensor is on white and front right on black move right
  else if ((LeftSensor == 0) && (RightSensor ==1)) {
    RightTurn();
  }

  // in any other case also move forward
   else {
    MotorsForward();
  }
}

// function to perform wall following at end of task
void Wallfollowing()
  {
    // This function will read all distance sensor values
    UpdateDistanceSensors();

    // if right distance sensor detects wall turn left
    if(distance[1]==1 ){
      digitalWrite(10, LOW);
      MotorsStop();
      delay(500);
      LeftTurn();
      delay(500);
      MotorsForwardSlow();
      delay(300);
      digitalWrite(10, HIGH);
    }

    // if left distance sensor detects wall turn left
    else if( distance[4]==1){
      digitalWrite(11, LOW);
      MotorsStop();
      delay(500);
      RightTurn();
      delay(150);
      MotorsForwardSlow();
      delay(300);
      digitalWrite(11, HIGH);
    }

    // if front distance sensor detects wall turn left
    else if(distance[0] == 1)
    {
      digitalWrite(12, LOW);
      MotorsStop();
      delay(500);
      MotorsBackward();
      delay(300);
      MotorsStop();
      delay(500);
      LeftTurn();
      delay(800);
      MotorsStop();
      delay(500);
      digitalWrite(12, HIGH);
    }
  }

// Updates values of all distance sensors
// and makes them binary based on threshold
// value. anything greater than threshold is
// 0 and lower than threshold is 1. 

  void UpdateDistanceSensors()
{
  for (int i = 0; i<8; i++) {
    if(i==3 | i==4 | i==5)
      continue;
     
     ReadDistanceSensor(i);
     
     if(i==6 | i==7)
     {
     ConvertDistSensorToBinary(i-3);
    }
    else
    {
      ConvertDistSensorToBinary(i);
    }
  }
}
// Reads the Distance Sensor whose number is provided e.g 0, 1, 2 or 3
void ReadDistanceSensor(int sensornum) {
  // Turn on respective IR Led so that light shines away from robot
  // and we are able to check if light comes back after hitting 
  // something that means obstacle in front.
  proxLEDon(sensornum);
  delay(10);
  // Stores value received from analogReading the sensor in RawSensorValue Variable
  RawSensorValue = analogRead(sensornum); //corresponding ground sensor
 
//  Serial.print(sensornum);
//  Serial.print("  ");
//  Serial.println(RawSensorValue);

   // Turn off IR led after reading the sensor
   proxLEDoff(sensornum); 
}

// This function checks value in RawSensorValue Variable,
// if that is less than DistThreshold value then distance sensors
// array's corresponding value is changed to 1, if not then it is
// given 0 value.

void ConvertDistSensorToBinary(int sensornum) {
  if (RawSensorValue < DistThreshold_T4) {
    distance[sensornum] = 1; //obstacle
  }
  else {
    distance[sensornum] = 0;
  }
  
}

// Turn Robot Left by moving Right wheel Forward and Left backward
void LeftTurn() {
  analogWrite(6, 0);
  analogWrite(7, 6.7 * 2.55);
  RightMotorForward(9);
}

// Turn Robot Right by moving Left wheel Forward and right backward
void RightTurn() {
  LeftMotorForward(7.5);
  analogWrite(5, 0);
  analogWrite(2, 10 * 2.55);
}

// This function makes both wheels move Backward
void MotorsBackward() {
  analogWrite(7, 18);
  analogWrite(6,0);
  analogWrite(2, 23);
  analogWrite(5,0);
}

// Stop both wheels by giving no power to motors
void MotorsStop(){
  analogWrite(2,1);
  analogWrite(5,1);
  analogWrite(6,1);
  analogWrite(7,1);
}

// Function that makes Left wheel move Forward
void LeftMotorForward(float duty) {
  analogWrite(6, duty * 2.55);
  analogWrite(7,0);
}

// Function that makes Right wheel move Forward
void RightMotorForward(float duty) {
  analogWrite(5, duty *2.55);
  analogWrite(2,0);
}

// This function makes both wheels move Forward but at
// slow speed for wall following
void MotorsForwardSlow() {
  LeftMotorForward(6.5);
  RightMotorForward(9.2);
}

// This function makes both wheels move Forward
void MotorsForward() {
  LeftMotorForward(6.7);
  RightMotorForward(9.7);
}

// Turns the IR led off of ground sensor whose Index is provided
void GroundLedOff(unsigned char LedIndex) {
  if ((LedIndex ==0) |(LedIndex ==1) | (LedIndex ==2)| (LedIndex ==3)){
    PORTJ |= (1<<LedIndex);
  }
 }

// Turns the IR led on of ground sensor whose Index is provided 
void GroundLedOn(unsigned char LedIndex) {
  if ((LedIndex ==0) |(LedIndex ==1) | (LedIndex ==2)| (LedIndex ==3)){
    PORTJ &= ~(1<<LedIndex);
  }
 }

// Turns the IR led of Distance sensor on whose Index is provided
 void proxLEDon(unsigned char proxIndex)
{
  PORTA |= (1<<proxIndex);
}

// Turns the IR led off of Distance sensor on whose Index is provided
void proxLEDoff(unsigned char proxIndex)
{
  PORTA &= ~(1<<proxIndex);
}
