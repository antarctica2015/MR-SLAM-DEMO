
#include "SonarModel.h"
#include <algorithm>


// Functionality for a sonar model
SonarModel::SonarModel(OccupancyMap *pOther, sf::CircleShape *pShape, sf::RectangleShape *pOtherLine, sf::CircleShape *pOtherSonar)
{
  pMap = pOther;
  pRobot = pShape;
  pLine = pOtherLine;
  pSonar = pOtherSonar;

  pBounds[0] = NULL;
  pBounds[1] = NULL;
  pBounds[2] = NULL;
  pBounds[3] = NULL;

  //
  gridPoint.alpha = 0.0f;
  gridPoint.r = 0.0f;

  //some defaults
  maxRange = 480.0f; //
  beta = 15; // (degrees)
  tolerance = 10.0f;
  maxOccupied = 0.98f;
  minProb = 0.02f;
  maxProb = 0.98f;
  sonarDistance = 0.0f;
  sonarTheta = 0.0f;
}

SonarModel::~SonarModel()
{
                  
}

//Calcuate a rough bounding box for the sonar model
int SonarModel::CalculateBoundingBox(sf::Vector2f rPos, sf::Vector2f sPos, double *pBox)
{
	double x1, x2, x3, x4;
	double y1, y2, y3, y4;
	double r, r2;

	double dx = (sPos.x - rPos.x); // sonar result offset X from robot
	double dy = (sPos.y - rPos.y); // sonar result offset Y from robot
	
	//printf("BB sonar offsets dX: %f dY: %f\n", dx, dy);

	x1 = robotPos.x; // +dx; //sonar device pos
	y1 = robotPos.y; // +dy;

	r = sonarDistance;
	r2 = (r / cos(DegreesToRadians(beta)));

	//printf("BB Sonar X: %f Y: %f Th: %f\n", x1, y1, sonarTh);
	
	// -90 + 10 = -80 in sfml rotation
	double adjustedTh = robotTh + sonarTh;

	//("Adjusted Sonar Th: %f\n", adjustedTh);

	//BKJ:
	double th1 = (adjustedTh - beta);
	double th2 = (adjustedTh + beta);

	double aL = std::min(th1, th2);
	double aU = std::max(th1, th2);

	//double aL = sonarTh - beta;
	//double aU = sonarTh + beta;

	//printf("BB Sonar aL: %f aU: %f \n", aL, aU);
	int quad = 0;

	x2 = r * cos(DegreesToRadians(aL)) + x1;
	y2 = r * sin(DegreesToRadians(aL)) + y1;

	x4 = r * cos(DegreesToRadians(aU)) + x1;
	y4 = r * sin(DegreesToRadians(aU)) + y1;

	x3 = r2 * cos(DegreesToRadians(adjustedTh)) + x1;
	y3 = r2 * sin(DegreesToRadians(adjustedTh)) + y1;

	//
	if ((dx >= 0) && (dy >= 0))
	{
		//Quadrant I
		quad = 1;
	}
	else if ((dx < 0) && (dy > 0))
	{
		// Quadant II
		quad = 2;
	}
	else if ((dx < 0) && (dy < 0))
	{
		// Quadant III
		quad = 3;
	}
	else if ((dx > 0) && (dy < 0))
	{
		// Quadant IV
		quad = 4;
	}
    
	//printf("x1: %f y1: %f\n", x1, y1);
	//printf("x2: %f y2: %f\n", x2, y2);
	//printf("x3: %f y3: %f\n", x3, y3);
	//printf("x4: %f y4: %f\n", x4, y4);

	auto ilX = { x1, x2, x3, x4 };
	auto ilY = { y1, y2, y3, y4 };

	double xL = std::min(ilX);
	double yL = std::min(ilY);

	double xU = std::max(ilX);
	double yU = std::max(ilY);

	//printf("xL: %f yL: %f xU: %f yU: %f\n", xL, yL, xU, yU);

	// -90 + 10 = -80 in sfml rotation
	//robotTh + sensorTh

	//need to rotate each point to match robots Th??
	//sf::Vector2f SonarModel::TransformToWorldSpace(double robotX, double robotY, double robotTh, double r, double alpha)

	//sf::Vector2f tempL = TransformToWorldSpace(rPos.x, rPos.y, robotTh,  );

	pBox[0] = xL;
	pBox[1] = yL;
	pBox[2] = xU;
	pBox[3] = yU;

	//pBox[0] = xL;
	//pBox[1] = yL;
	//pBox[2] = xU;
	//pBox[3] = yU;

	return quad;
}

void SonarModel::UpdateSonar(double rX, double rY, double rTh, double r, double alpha)
{
	//pose is the odometry pose in simulator global space.
	//r = range to object in mm
	//alpha is angle in degrees; (-+90)

	if ( r == 5000.0f)
	{
		alpha = 0.0f;
	}

	double range = r * 0.1; // range to detected object

	//printf("raw robot values are: %f %f %f\n", rX, rY, rTh);
	robotPos = sf::Vector2f((rX * 0.1), (rY * 0.1)); // Y-coord is flipped in SMFL

	if (robotPos.y != 0)
	{
		robotPos.y *= -1;
	}
	
	robotTh = ( (int)rTh == 0 ) ? rTh : rTh * -1;  // smfl rotation is opposite to simulators
	alpha = ( (int)alpha == 0 ) ? alpha : alpha * -1; // smfl rotation is opposite to simulators
		
	//printf("Robot : X: %f Y: %f Th: %f || Sonar Result: range: %f alpha: %f\n", robotPos.x, robotPos.y, robotTh, range, alpha);

	sonarX = robotPos.x;  // should have offset applied
	sonarY = robotPos.y;  // should have offset applied
	sonarTh = alpha ; // smfl rotation is opposite to simulators

	//perform the worldspace transforms to get sonar result position in global space.
	sonarPos = TransformToWorldSpace(robotPos.x, robotPos.y, robotTh, range, sonarTh);
	
	//printf("Sonar Pose X: %f Y: %f Th: %f\n", sonarPos.x, sonarPos.y, sonarTh);

	//update circle & sonar markers
	pRobot->setPosition(robotPos);
	pLine->setPosition(robotPos.x, robotPos.y);
	pLine->setRotation(robotTh);
	pSonar->setPosition(sonarPos);

	//update location of sonar result.
	sonarDistance = range; // * 0.01;

	//update the sonars bounding box around point
	int quadrant = CalculateBoundingBox(robotPos, sonarPos, pBounds);
	//printf("Sonar bounding box: x1:%f y1:%f x2:%f y2:%f\n", pBounds[0], pBounds[1], pBounds[2], pBounds[3]);
	//printf("Quadrant: %i \n", quadrant);
	
	//get a list of points to update
	pointsToCheck = pMap->GetGridPointsInBounds(pBounds, robotPos, robotTh, sonarPos, sonarTh, maxRange, radius); // a cone to check between robot and sonar points.

	//for each grid point check what region it is and update its probability
	for (std::list<CELL>::iterator it = pointsToCheck.begin(); it != pointsToCheck.end(); it++)
	{
		//
		gridPoint.r = it->distance;
		gridPoint.alpha = it->angle; //alpha should be the offset from sonarth and less than beta value
	
	//	printf("GridPoint: r: %f alpha: %f\n",it->x, it->y, gridPoint.r, gridPoint.alpha);

		//regionI bounds
		double lower = sonarDistance - tolerance;
		double upper = sonarDistance + tolerance;

		//now for each point under consideration check which region and update probability
		if (lower < gridPoint.r && gridPoint.r < upper)
		{
			//Region I (object)
			UpdateRegionI(it->i, it->j);
			//printf("Updating region I\n");
		}
		else if (gridPoint.r < lower)
		{
			//Region II (empty)
			UpdateRegionII(it->i, it->j);
			//printf("Updating region II\n");
		}

		//Region III - do nothing
	}
}

// updates
void SonarModel::UpdateRegionI(int i, int j)
{
	//calculate new sensor probabilty and update accordingly
	double probSonarOccupied = (( ((maxRange - gridPoint.r)/maxRange) + ((beta - gridPoint.alpha)/beta) ) / 2 ) * maxOccupied;
	//probSonarOccupied = probSonarOccupied > maxOccupied ? maxOccupied : probSonarOccupied;

	if (probSonarOccupied < minProb)
	{
		probSonarOccupied = minProb;
	}
	
	if (probSonarOccupied > maxOccupied)
	{
		probSonarOccupied = maxOccupied;
	}

	double probSonarEmpty = 1.0 - probSonarOccupied;

	if (probSonarEmpty < minProb)
	{
		probSonarEmpty = minProb;
	}

	//now should be clamped to range [0.02, maxOccupied]
	UpdateBayes(1, i, j, probSonarOccupied, probSonarEmpty, pMap->obstacleColour);
}

void SonarModel::UpdateRegionII(int i, int j)
{
	//calculate new sensor probabilty and update accordingly
	double probSonarEmpty = ( ((maxRange - gridPoint.r) / maxRange ) + ( (beta - gridPoint.alpha) / beta) ) / 2;
	double probSonarOccupied = 1.0 - probSonarEmpty;

	if (probSonarEmpty < minProb)
	{
		probSonarEmpty = minProb;
	}

	if (probSonarOccupied < minProb)
	{
		probSonarOccupied = minProb;
	}

	//clamped to range [0.02, 0.98]
	UpdateBayes(2, i, j, probSonarOccupied, probSonarEmpty, pMap->emptyColour);
	//UpdateBayes(2, i, j, probSonarOccupied, probSonarEmpty, pMap->obstacleColour);
}

void SonarModel::UpdateBayes(int region, int i, int j, double sonarOccupied, double sonarEmpty, sf::Color colour)
{
	double probGridOccupied = 0.0;
	double probGridEmpty = 0.0;

	//now can calculate updated grid probability taking prior into account.
	if (region == 1)
	{
		pMap->grid[i][j].region = 1;
		probGridOccupied = (sonarOccupied * pMap->grid[i][j].occupied) / ( (sonarOccupied * pMap->grid[i][j].occupied) + (sonarEmpty * pMap->grid[i][j].empty) );
		probGridEmpty = 1.0 - probGridOccupied;
	}
	else if (region == 2)
	{
		pMap->grid[i][j].region = 2;
		probGridEmpty = (sonarEmpty * pMap->grid[i][j].empty) / ( (sonarEmpty * pMap->grid[i][j].empty) + (sonarOccupied * pMap->grid[i][j].occupied) );
		probGridOccupied = 1.0 - probGridEmpty;
	}
	
	if ( isnan(probGridOccupied) || isnan(probGridEmpty) )
	{
		printf("NaN found");
	}

	/***********************************/

	if (probGridOccupied < minProb)
	{
		probGridOccupied = minProb;
	}
	else if (probGridOccupied > maxProb)
	{
		probGridOccupied = maxProb;
	}

	if (probGridEmpty < minProb)
	{
		probGridEmpty = minProb;
	}
	else if (probGridEmpty > maxProb)
	{
		probGridEmpty = maxProb;
	}

	/***********************************/

	// now actually update the grid probability
	pMap->UpdateGridPosition(region, i, j, probGridOccupied, probGridEmpty, colour);
}
      
double SonarModel::DegreesToRadians(double angle)
{
       return (M_PI / 180.0) * angle;
}

double SonarModel::RadiansToDegrees(double angle)
{
       return (180.0  / M_PI) * angle;
}

sf::Vector2f SonarModel::GetSonarOffset()
{
	//return the offset for the given device
	return sf::Vector2f(0.0,0.0);
}

//convert polar to cartesian coords
sf::Vector2f SonarModel::PolarToCartesianCoords(double r, double alpha)
{
	double a  = DegreesToRadians(alpha);

	double x = r * cos(a);
	double y = r * sin(a);

	return sf::Vector2f(x,y);
}

sf::Vector2f SonarModel::PolarToCartesianCoords(POLAR polarCoords)
{
	double a = DegreesToRadians(polarCoords.alpha);

	double x = (polarCoords.r) * cos(a);
	double y = (polarCoords.r) * sin(a);

	return sf::Vector2f(x,y);
}

//
POLAR SonarModel::CartesianToPolarCoords(double x, double y)
{
	POLAR tmp;

	tmp.r = sqrt((x*x)+ (y*y));
	tmp.alpha = RadiansToDegrees(atan2(y, x));
	
	return tmp;
}

//
POLAR SonarModel::CartesianToPolarCoords(sf::Vector2f pos)
{
	POLAR tmp;

	tmp.r = sqrt((pos.x * pos.x) + (pos.y * pos.y));
	tmp.alpha = RadiansToDegrees(atan2(pos.y, pos.x));
	
	return tmp;
}


/*
algorithm (robot local to SFML global) :
-Find the position of each sonar reading in the robots co-ordinate system
- Rotate all calculated positions by the robots heading
- Translate all rotated co-ordinates by the robots position
*/
//convert Sonar position Aria Global Coordinates to SFML Global
sf::Vector2f SonarModel::TransformToWorldSpace(double robotX, double robotY, double robotTh, double r, double alpha)
{
	/*
	double robotX,   // robot global X
	double robotY,   // robot global Y
	double robotTh,  // robot angle
	double r,        // distance from robot to sonar result
	double alpha     // angle from robot to sonar result
	*/

	//convert sonar r, alpha to cartesian coordinates.  (offsets from robot?)
	//sf::Vector2f resultCoords = DoSonarQuadTranslate( PolarToCartesianCoords(r, alpha) );

	// now do the rest.
	double globalX = robotX;
	double globalY = robotY;
	double globalTh = DegreesToRadians(robotTh);

	//convert alpha from degrees to radians
	double a = DegreesToRadians(alpha);

	//step 1 - find x and y (cartesian coords of sensor result offset from robot.)
	double x = cos(a) *  r;	//(r + radius);
	double y = sin(a) *  r;	//(r + radius);

	//step 2 - rotate CCW
	double x_r = (x * cos(globalTh)) + (y * -sin(globalTh));
	double y_r = (x * sin(globalTh)) + (y * cos(globalTh));

	//CW
	//double x_r = (x * cos(globalTh)) + (y * sin(globalTh));
	//double y_r = (x * -sin(globalTh)) + (y * cos(globalTh));

	//step 3 - translate
	double x_t = x_r + globalX;
	double y_t = y_r + globalY;

	sf::Vector2f globalPoint = sf::Vector2f(x_t, y_t);

	// this should be the sensor result in global cartesian coordinates.
	return globalPoint;
}



