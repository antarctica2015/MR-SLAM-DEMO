#include "Aria.h"
#include "ArActionExplore.h"

ArActionExplore::ArActionExplore() : ArAction("Explore")
{
	radius = 340.0f * 0.1; //mm -> m

	frontDist = 0.0f;
	frontAngle = 0.0f;

	otherDist = 0.0f;
	otherAngle = 0.0f;

	rotated = false;
}

ArActionExplore::~ArActionExplore()
{
}

// Body of action
ArActionDesired * ArActionExplore::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)

    //Rotate the robot
	//myRobot->setDeltaHeading(10);

	// setting to -180, 180 causes an aria bug with returned angle......
    frontDist = myRobot->checkRangeDevicesCurrentPolar(-15, 15, &frontAngle); // 
	//frontDist = myRobot->checkRangeDevicesCurrentPolar(-180, 0, &frontAngle); // 
	//frontDist = myRobot->checkRangeDevicesCurrentPolar(-80, 80, &frontAngle); // 
	//otherDist = myRobot->checkRangeDevicesCurrentPolar(90, -90, &otherAngle); // 

	//frontDist = myRobot->checkRangeDevicesCumulativePolar(-20, 20, &frontAngle);

	//rearDist = myRobot->checkRangeDevicesCurrentPolar(-45, 45, &rearAngle, &pSensor); // 90 degree arc behind the robot
	//frontDist = myRobot->checkRangeDevicesCurrentPolar(-10.0, 10.0, &frontAngle);

	//**pSensor;
	//ArSonarDevice sd = &pSensor;
	//which sensor detected the object
	//ArLog::log(ArLog::Normal, "Device Name: %s", sd->GetName());
	//use name to lookup sensor offset information?

	// This gives distance to the centre of the robot - so now find the actual distance
	//frontDist = frontDist - radius;

	//ArLog::log(ArLog::Normal, "front sonar:: range: %.2f angle: %.2f", frontDist, frontAngle);
	//printf("RAW X: %f Y: %f TH: %f r: %f alpha: %f\n", myRobot->getX(), myRobot->getY(), myRobot->getTh(), frontDist, frontAngle);

	// We now know the Local Polar Coordinates of the closest Object with respect to the robot.
	// And with the Robots global odometric pose we can update the Sonar Model.
	if (frontDist < 5000.0f)
	{
		pSonarModel->UpdateSonar(myRobot->getX(), myRobot->getY(), myRobot->getTh(), frontDist, frontAngle);
		//pSonarModel->UpdateSonar(myRobot->getX(), myRobot->getY(), myRobot->getTh(), otherDist, otherAngle);
	}

	
	//if (rotated == false)
	//{
		//myRobot->setDeltaHeading(10);
		//rotated = true;
	//}
	
	//
	//myRobot->setDeltaHeading(-10);

	return &desiredState; // give the desired state to the robot for actioning
}

//
void ArActionExplore::Init(OccupancyMap *pOther, sf::CircleShape *pCircle, sf::RectangleShape *pLine, sf::CircleShape *pSonar)
{
	//pMarker = pCircle;
	pSonarModel = new SonarModel(pOther, pCircle,pLine, pSonar);
}