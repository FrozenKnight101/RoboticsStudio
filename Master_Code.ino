//Reference: 
// https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/examples/Me_LineFollower/LineFollowerTest/LineFollowerTest.ino
// https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/examples/Me_UltrasonicSensor/UltrasonicSensorTest/UltrasonicSensorTest.ino
// https://github.com/pololu/zumo-shield/blob/master/ZumoExamples/examples/MazeSolver/MazeSolver.ino



#include "MeAuriga.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>


MeRGBLed led;

MeUltrasonicSensor sonar_center(8);
MeUltrasonicSensor sonar_right(9);

MeLineFollower linefollower_left(6);
MeLineFollower linefollower_center(7);
MeLineFollower linefollower_right(10);

MeEncoderOnBoard right_motor(SLOT1);
MeEncoderOnBoard left_motor(SLOT2);

MeGyro gyro_0(0, 0x69);

int currLeftMotorSpeed = -1;
int currRightMotorSpeed = -1;

int SENSOR_THRESHOLD = 300;
int SPEED = 50;
int TURN_SPEED = 50;
int OS_DELAY = 300; // OVERSHOOT DELAY TIME IN MILLISECONDS

char path[100] = "";
unsigned int path_length = 0; // the length of the path

bool whiteLine = false; // tells if center line i.e, path is white
int sensorVals[6]; // stores the values of the 6 ir sensors

void setup()
{
  Serial.begin(9600);
  led.setpin(44);
  turnOffLights();
}

void loop()
{
  // solveMaze() explores every segment
  // of the maze until it finds the finish
  // line.
  solveMaze();
    
  // The maze has been solved. When the user
  // places the Zumo at the starting line
  // and pushes the Zumo button, the Zumo
  // knows where the finish line is and
  // will automatically navigate.
  delay(20000);
  
  goToFinishLine();
}


void setLights(int x, int r, int g, int b) {
  led.setColor(0,0,0,0);
  led.setColor(x, r, g, b);
  led.show();
}

void turnOffLights() {
  setLights(0,0,0,0);
}

void stopMoving() {
   if (currLeftMotorSpeed != 0 || currRightMotorSpeed != 0) {
       currLeftMotorSpeed = 0;
       currRightMotorSpeed = 0;
       right_motor.setMotorPwm(0);
       left_motor.setMotorPwm(0);
   }
}

void startMoving(int left, int right) {
    if (currLeftMotorSpeed != left || currRightMotorSpeed != right) {
       currLeftMotorSpeed = left;
       currRightMotorSpeed = right;
       right_motor.setMotorPwm(-right);
       left_motor.setMotorPwm(left);
    }
}

void turn(int direction, int angle){
    left_motor.setTarPWM(0);
    right_motor.setTarPWM(0);
    _delay(0.5);
    if(direction == 0){
        expectedYaw = gyro_0.getAngle(3);
        expectedYaw += angle * -1;
        while(!(gyro_0.getAngle(3) < expectedYaw))
        {
          startMoving(-TURN_SPEED, TURN_SPEED);
        }
        left_motor.setTarPWM(0);
        right_motor.setTarPWM(0);
        delay(500);
    }else{
        expectedYaw = gyro_0.getAngle(3);
        expectedYaw += angle;

        while(!(gyro_0.getAngle(3) > expectedYaw))
        {
          startMoving(TURN_SPEED, -TURN_SPEED);
        }
        left_motor.setTarPWM(0);
        right_motor.setTarPWM(0);
        delay(500);
    }
}

void obstacleAvoidance()
{
  if (sonar_center.distanceCm() <= 15)
  {
    turn(0, 90);
    while(sonar_right.distanceCm() <= 12)
    {
      startMoving(SPEED, SPEED);
    }
    stopMoving();
    turn(1, 90);
    while(sonar_right.distanceCm() <= 12)
    {
      startMoving(SPEED, SPEED);
    }
    delay(500);
    stopMoving();
    while(1)
    {
      startMoving(SPEED, SPEED);
      if(ABOVE_LINE(sensorVals[0]) && ABOVE_LINE(sensorVals[1]) && ABOVE_LINE(sensorVals[2]) && ABOVE_LINE(sensorVals[3]) && ABOVE_LINE(sensorVals[4]) && ABOVE_LINE(sensorVals[5]))
      {
        break;
      }
    }
    stopMoving();
    turn(0, 90);
  }
}

int ABOVE_LINE(int sensor){
  if(whiteLine == false) {
    if(sensor > SENSOR_THRESHOLD)
      return 1;
    else
      return 0;
  }
  else if(whiteLine == true) {
    if(sensor < SENSOR_THRESHOLD)
      return 1;
    else
      return 0;
  }
}

int readSensorVals() {
  int sensorState[3];
  sensorState[0] = linefollower_left.readSensors();
  sensorState[1] = linefollower_center.readSensors();
  sensorState[2] = linefollower_right.readSensors();

  int sno = 0;  
  for(int s = 0; s < 3; s++)
  {    
    switch(sensorState[s])
    {
      case 0: 
              Serial.println("Sensor 1 and 2 are inside of black line"); // BOTH BLACK
              sensorVals[sno] = 1000;
              sensorVals[sno+1] = 1000;
              break; 
      case 1: 
              Serial.println("Sensor 2 is outside of black line"); // LEFT BLACK RIGHT WHITE
              sensorVals[sno] = 1000;
              sensorVals[sno+1] = 0;
              break; 
      case 2: 
              Serial.println("Sensor 1 is outside of black line"); // LEFT WHITE RIGHT BLACK
              sensorVals[sno] = 0;
              sensorVals[sno+1] = 1000;
              break; 
      case 3: 
              Serial.println("Sensor 1 and 2 are outside of black line"); 
              sensorVals[sno] = 0;
              sensorVals[sno+1] = 0;
              break; //BOTH WHITE
      default: break;
    }
    sno += 2;
  }

  if(sensorVals[0] == 1 && (sensorState[2] == 0 || sensorState[3] == 0) && sensorVals[5] == 1) // Left and Right on White, Center is Black
  {
    whiteLine = false;
  }
  else if(sensorVals[0] == 0 && (sensorState[2] == 1 || sensorState[3] == 1) && sensorVals[5] == 0) // Left and Right on Black, Center is White
  {
    whiteLine = true;
  }

  return readValues();
}

int readValues()
{
    int on_line = 0;
    unsigned long weighted_sum = 0;
    unsigned int sum = 0;
    int value;
    for (int i = 0; i < 6; i++) {
          value = sensorVals[i];
        
        if (whiteLine) {
            value = 1000 - value;
        }

        if (value > 200) {
            on_line = 1;
        }
        weighted_sum += value * i * 1000;
        sum += value;
    }
    if (!on_line) {
        return 0;
    }
    return weighted_sum / sum;
}

void followSegment()
{
  unsigned int position;
  int offset_from_center;
  int power_difference;
  
  while(1)
  {     
    obstacleAvoidance();
    // Get the position of the line.
    position = readSensorVals();
     
    // The offset_from_center should be 0 when we are on the line.
    offset_from_center = ((int)position) - 2500;
     
    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the left.  If it is a negative number, the robot will
    // turn to the right, and the magnitude of the number determines
    // the sharpness of the turn.
    power_difference = offset_from_center / 3;
     
    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    // if(power_difference > SPEED)
    //   power_difference = SPEED;
    // if(power_difference < -SPEED)
    //   power_difference = -SPEED;

    int left_pd = 0, right_pd = 0;

    if(ABOVE_LINE(sensorVals[2]) && ABOVE_LINE(sensorVals[3]))
    {
        left_pd = 0;
        right_pd = 0;
        setLights(0,0,255,0);
        startMoving(SPEED, SPEED);  
    }
    else if (!ABOVE_LINE(sensorVals[2]) && ABOVE_LINE(sensorVals[3]))
    {
        setLights(0,0,0,255);
        startMoving(TURN_SPEED, -TURN_SPEED);
    }
    else if (ABOVE_LINE(sensorVals[2]) && !ABOVE_LINE(sensorVals[3]))
    {
        setLights(0,0,0,255);
        startMoving(-TURN_SPEED, TURN_SPEED);
    }
    else if(!ABOVE_LINE(sensorVals[2]) && !ABOVE_LINE(sensorVals[3]))
    {
        setLights(0,255,0,0);
        startMoving(-TURN_SPEED, TURN_SPEED);
    }
     
    // if(power_difference < 0) {
    //     startMoving(SPEED + power_difference, SPEED);
    // }
    // else {
    //     startMoving(SPEED, SPEED - power_difference);
    // }
     
    // We use the inner four sensors (1, 2, 3, and 4) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 5 for detecting lines going to the left and
    // right.
    // CHECK FOR INTERSECTION/DEAD-END
    if(!ABOVE_LINE(sensorVals[0]) && !ABOVE_LINE(sensorVals[1]) && !ABOVE_LINE(sensorVals[2]) && !ABOVE_LINE(sensorVals[3]) && !ABOVE_LINE(sensorVals[4]) && !ABOVE_LINE(sensorVals[5]))
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.            
      return;
    }
    else if(ABOVE_LINE(sensorVals[0]) || ABOVE_LINE(sensorVals[5]))
    {
      // Found an intersection.
      return;
    }
  }
}

char selectTurn(char found_left, char found_straight, char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if(found_left)
    return 'L';
  else if(found_straight)
    return 'S';
  else if(found_right)
    return 'R';
  else
    return 'B';
}

// Turns according to the parameter dir, which should be 
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
void turn(char dir)
{

  // count and last_status help
  // keep track of how much further
  // the Zumo needs to turn.
  unsigned short count = 0;
  unsigned short last_status = 0;
  unsigned int sensors[6];
  
  // dir tests for which direction to turn
  switch(dir)
  {
    // Since we're using the sensors to coordinate turns instead of timing them, 
    // we can treat a left turn the same as a direction reversal: they differ only 
    // in whether the zumo will turn 90 degrees or 180 degrees before seeing the 
    // line under the sensor. If 'B' is passed to the turn function when there is a
    // left turn available, then the Zumo will turn onto the left segment.
    case 'L':
    // startMoving(-50, 50); 
    // delay(0.5);
    // stopMoving();
    // break;
	case 'B':
      // Turn left.
      startMoving(-TURN_SPEED, TURN_SPEED); 
    // startMoving(-50, 50); 
    // delay(500);
    // stopMoving();
    // delay(500);
    // startMoving(-50, 50); 
    // delay(500);
    // stopMoving();
      // This while loop monitors line position
      // until the turn is complete. 


      while(count < 2)
      {
        readSensorVals();
        // Increment count whenever the state of the sensor changes 
		// (white->black and black->white) since the sensor should 
		// pass over 1 line while the robot is turning, the final 
		// count should be 2
        count += ABOVE_LINE(sensors[1]) ^ last_status; 
        last_status = ABOVE_LINE(sensors[1]);
      }
    break;
    
    case 'R':
      // Turn right.
      startMoving(TURN_SPEED, -TURN_SPEED);
    //   startMoving(50, -50); 
    // delay(0.5);
    // stopMoving();
      // This while loop monitors line position
      // until the turn is complete. 
      while(count < 2)
      {
        readSensorVals();
        count += ABOVE_LINE(sensors[4]) ^ last_status;
        last_status = ABOVE_LINE(sensors[4]);
      }
    break;
	
    case 'S':
    // Don't do anything!
    break;
  }
}

void solveMaze()
{
    while(1)
    {
        // Navigate current line segment
        followSegment();
         
        // These variables record whether the robot has seen a line to the
        // left, straight ahead, and right, while examining the current
        // intersection.
        unsigned int found_left = 0;
        unsigned int found_straight = 0;
        unsigned int found_right = 0;
         
        // Now read the sensors and check the intersection type
        readSensorVals();
         
        // Check for left and right exits.
        if(ABOVE_LINE(sensorVals[0]))
            found_left = 1;
        if(ABOVE_LINE(sensorVals[5]))
            found_right = 1;
            
        // Drive straight a bit more, until we are
        // approximately in the middle of intersection.
        // This should help us better detect if we
        // have left or right segments.
        // motors.setSpeeds(SPEED, SPEED);
        startMoving(SPEED, SPEED);

        // TODO : REPLACE OVERSHOOT DELAY
        // delay(OVERSHOOT(LINE_THICKNESS)/2);
        // delay(OS_DELAY);
        
        readSensorVals();
         
        // Check for left and right exits.
        if(ABOVE_LINE(sensorVals[0]))
            found_left = 1;
        if(ABOVE_LINE(sensorVals[5]))
            found_right = 1;
        
        // After driving a little further, we
        // should have passed the intersection
        // and can check to see if we've hit the
        // finish line or if there is a straight segment
        // ahead.  

        // TODO : REPLACE OVERSHOOT DELAY
        // delay(OVERSHOOT(LINE_THICKNESS)/2);
        delay(OS_DELAY);
        
        // Check for a straight exit.
        readSensorVals();
        
        // Check again to see if left or right segment has been found
        // if(ABOVE_LINE(sensorVals[0]))
        //     found_left = 1;
        // if(ABOVE_LINE(sensorVals[5]))
        //     found_right = 1;
        
        if(ABOVE_LINE(sensorVals[2]) || ABOVE_LINE(sensorVals[3]))
            found_straight = 1;
         
        // Check for the ending spot.
        // If all four middle sensors are on dark black, we have
        // solved the maze.
        if(ABOVE_LINE(sensorVals[1]) && ABOVE_LINE(sensorVals[2]) && ABOVE_LINE(sensorVals[3]) && ABOVE_LINE(sensorVals[4]))
        {
          stopMoving();
          break;
        }
         
        // Intersection identification is complete.
        char dir = selectTurn(found_left, found_straight, found_right);
        
        // Make the turn indicated by the path.
        turn(dir);
         
        // Store the intersection in the path variable.
        path[path_length] = dir;
        path_length++;
         
        // You should check to make sure that the path_length does not
        // exceed the bounds of the array.  We'll ignore that in this
        // example.
        // Simplify the learned path.
        simplifyPath();
    }
}

void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(path_length < 3 || path[path_length - 2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for(i = 1; i <= 3; i++)
  {
    switch(path[path_length - i])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;
  // Replace all of those turns with a single one.
  switch(total_angle)
  {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  // The path is now two steps shorter.
  path_length -= 2;
}


void goToFinishLine()
{
  int i = 0;

  // Turn around if the Zumo is facing the wrong direction.
  if(path[0] == 'B')
  {
    turn('B');
    i++;
  }
  
  for(;i<path_length;i++)
  {
    followSegment();

    // Drive through the intersection. 
    startMoving(SPEED, SPEED);
    delay(OS_DELAY);
                   
    // Make a turn according to the instruction stored in
    // path[i].
    turn(path[i]);
  }
    
  // Follow the last segment up to the finish.
  followSegment();
 
  // The finish line has been reached.
  // Return and wait for another button push to
  // restart the maze.         
  readSensorVals();
  stopMoving();
  
  return; 
} 
