#include <Aria.h>

#include "avoid.h"

using namespace std;
// Implementation

// Constructor
avoid::avoid() : ArAction("Avoid")
{
  speed = 50.0f; // Set the robots speed to 100 mm/s. 200 is top speed
  min_distance = 3000.0f; // Set the minimum distance to an obstacle to 450 mm;
  radius = 340.0f; // Radius of the robot
}

// Body of action
ArActionDesired * avoid::fire(ArActionDesired d)
{
  desiredState.reset(); // reset the desired state (must be done)

  // get sonar readings
  leftDist = myRobot->checkRangeDevicesCurrentPolar(0,75,&leftAngle);
  rightDist = myRobot->checkRangeDevicesCurrentPolar(-75,0,&rightAngle);

  // This gives distance to the centre of the robot
  // we must now find the actual distance

  //leftDist = leftDist - radius; 
  //rightDist = rightDist - radius;

 //ArLog::log(ArLog::Normal, "left %.2f %.2f\tright %.2f %.2f", leftDist, leftAngle, rightDist, rightAngle);

  if(leftDist < min_distance)
  {
     if(rightDist < min_distance)
     {
       ArLog::log(ArLog::Normal, "Too close - stopping... ");
       speed = 0;
       myRobot->stop(); // stop the robot
       myRobot->stopRunning(true); // disconnect from the robot
       ArLog::log(ArLog::Normal,"stopped and disconnect\n");
     }
     else
     {
       speed = 0;
       turn( (rand() % 60 + 1) * -1 );
     } 
  }
  else
  {
    if(rightDist < min_distance)
     {
       speed = 0;
       turn((rand() % 60 + 1));
     }
     else
     {
        speed = 100;
     }
  }

  desiredState.setVel(speed); // set the speed of the robot in the desired state
  return &desiredState; // give the desired state to the robot for actioning
}


// Turn the robot
void avoid::turn(double turnAngle)
{
  myRobot->setDeltaHeading(turnAngle);
}
