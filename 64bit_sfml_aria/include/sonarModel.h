#ifndef SONARMODEL_H
#define SONARMODEL_H

#pragma once
#include <SFML\Graphics.hpp>
#include <SFML\Window.hpp>
#include <SFML\System.hpp>

#include "Aria.h"
#include "math.h"
#include "OccupancyMap.h"

class SonarModel
{
public:
	SonarModel(OccupancyMap *pMap, sf::CircleShape *pRobot, sf::RectangleShape *pLine, sf::CircleShape *pSonar);
	~SonarModel();

	//update methods
	void UpdateSonar(double robotX, double robotY, double robotTh, double r, double alpha);

protected:
	
	  std::list<CELL> pointsToCheck;
	  const double radius = 340.0f * 0.1; // 340mm // 0.34f; 

	  sf::Vector2f sonarOrigin; //

	  double pBounds[4];

	  double maxRange;  // The max range of the sonar sensor
	  double beta;
	  double tolerance;  // Error tolerance (epsilon)
	  double sonarDistance; // sonar distance reading (r)
	  double sonarTheta; //  (alpha)
	  double maxOccupied; // maximum certainty of obstacle (0.0 - 1.0)
	  double maxProb;
	  double minProb;

	  POLAR gridPoint; // Polar coords of grid point under consideration.

	  sf::CircleShape *pRobot;
	  sf::RectangleShape *pLine;
	  sf::CircleShape *pSonar;
	  OccupancyMap *pMap; // pointer to the main grid 

	  ArPose robotPose;  //robot simulator pose // //ArPose sonarPose; //sonar global pose in sfml
	  sf::Vector2f sonarPos; //sonar global position in sfml
	  sf::Vector2f robotPos; //robot global position in sfml

	  double sonarX, sonarY, sonarTh; // Sonar Pose in global (robot) co-ord space
	  double robotX, robotY, robotTh; // Sonar Pose in global (robot) co-ord space

	  sf::Vector2f sonarOffset = sf::Vector2f(0.0f, 0.0f);
	  sf::FloatRect box;

	  //methods
	  void UpdateRegionI(int x, int y);
	  void UpdateRegionII(int x, int y);
	  void UpdateBayes(int region, int x, int y, double sonarOccupied, double sonarEmpty, sf::Color colour);
 
      double DegreesToRadians(double angle); // Angle convertors
      double RadiansToDegrees(double angle);   

	  int CalculateBoundingBox(sf::Vector2f rPos, sf::Vector2f sPos, double* pBounds);
	  sf::Vector2f GetSonarOffset();

	  sf::Vector2f PolarToCartesianCoords(double r, double alpha);
	  sf::Vector2f PolarToCartesianCoords(POLAR polarCoords);

	  POLAR CartesianToPolarCoords(double x, double y);
	  POLAR CartesianToPolarCoords(sf::Vector2f vec);
	  
	  sf::Vector2f TransformToWorldSpace(double sonarX, double sonarY, double sonarTh, double r, double alpha);
};

#endif
