#ifndef AVOID_H
#define AVOID_H

// Signatures

class avoid : public ArAction // Class action inherits from ArAction
{
 public:
   avoid(); // Constructor
   virtual ~avoid() {}  // Destructor
   virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
   ArActionDesired desiredState; // Holds state of the robot that we wish to action
 protected:
   int speed; // Speed of the robot in mm/s
   double min_distance; // The closest we will let the robot get to an obstacle
   double radius; // radius of the robot
   double leftDist; // Distance to the closest obstacle on the front left of the robot
   double leftAngle; // Angle to the closest obstacle on the front left of the robot
   double rightDist; // Distance to the closest obstacle on the front right of the robot
   double rightAngle; // Angle to the closest obstacle on the front right of the robot
   void turn(double turnAngle); // Turn the robot
};

#endif