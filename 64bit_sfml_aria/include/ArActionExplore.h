#pragma once

#include "OccupancyMap.h"
#include "SonarModel.h"

class ArActionExplore : public ArAction
{
public:
	ArActionExplore();
	virtual ~ArActionExplore();

	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
	ArActionDesired desiredState; // Holds state of the robot that we wish to action

	void Init(OccupancyMap *pOther, sf::CircleShape *pCircle, sf::RectangleShape *pLine, sf::CircleShape *pSonar);

	void SetRadius(double newRadius) { radius = newRadius; }

protected:
	SonarModel *pSonarModel; // Sonar model for probability calculations.

	//int maxSpeed;
	bool rotated;
	int currentSpeed; // Speed of the robot in mm/s
	double radius; // radius of the robot
	//double lowestDistance; // Distance to the closest obstacle

	//
	const ArRangeDevice **pSensor; // sensor that returned the sonar result
	//ArSonarDevice *pSensor;

	//
	double frontDist; // Distance to the closest obstacle in front of the robot
	double frontAngle; // Angle to the closest obstacle in front of the robot

	double otherDist; // Distance to the closest obstacle in front of the robot
	double otherAngle; // Angle to the closest obstacle in front of the robot

};

