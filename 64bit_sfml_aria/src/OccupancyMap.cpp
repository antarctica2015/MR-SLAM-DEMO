#include "Aria.h"
#include "OccupancyMap.h"

OccupancyMap::OccupancyMap()
{	
	//
	//sonarBox.setFillColor(sf::Color::Transparent);
	sonarBox.setFillColor(sf::Color::White);
	sonarBox.setOutlineColor(sf::Color::Yellow);
	sonarBox.setOutlineThickness(-2.0f);

	//
	for (unsigned int i = 0; i < mapWidth; ++i)
	{	
		for (unsigned int j = 0; j < mapHeight; ++j)
		{

			grid[i][j].region = 3;

			grid[i][j].i = i;
			grid[i][j].j = j;

			//set the inital probabilities for the grid`
			grid[i][j].empty = 0.5f;
			grid[i][j].occupied = 0.5f;

			grid[i][j].distance = 0.0f;
			grid[i][j].angle = 0.0f;

			// Set the default colors and also draw a nice grid pattern over the top.
			grid[i][j].rect.setSize(sf::Vector2f(tileSize.x, tileSize.y));
			grid[i][j].rect.setOutlineColor(sf::Color::Black);
			grid[i][j].rect.setFillColor(unknownColour);
			grid[i][j].rect.setOutlineThickness(-0.5f);

			//
			grid[i][j].x = (i * tileSize.x);
			grid[i][j].y = (j * tileSize.y);

			//
			grid[i][j].rect.setOrigin(sf::Vector2f(5.0f, 5.0f));
			grid[i][j].rect.setPosition(sf::Vector2f( (i * tileSize.x) +5, (j * tileSize.y) +5) );
			grid[i][j].rect.move(sf::Vector2f(-500.0f, -500.0f));
		}
	}
}

//
OccupancyMap::~OccupancyMap()
{
	//
}

// converts a x,y position in space to grid coordinates
sf::Vector2u OccupancyMap::MapToGridCoords(double x, double y)
{
	//bounds checks
	x = (x >= 500) ? 500 : x;
	y = (y >= 500) ? 500 : y;

	x = (x <= -500) ? -500 : x;
	y = (y <= -500) ? -500 : y;

	double tmpX = floor((x) / tileSize.x) + 50;
	double tmpY = floor((y) / tileSize.y) + 50;

	tmpX = (tmpX >= 99) ? 99 : tmpX;
	tmpY = (tmpY >= 99) ? 99 : tmpY;

	tmpX = (tmpX <= 0) ? 0 : tmpX;
	tmpY = (tmpY <= 0) ? 0 : tmpY;

	return sf::Vector2u(tmpX, tmpY);
}

// converts a 2d vector to grid coordinates
sf::Vector2u OccupancyMap::MapToGridCoords(sf::Vector2f vec)
{
	return sf::Vector2u(floor(vec.x / tileSize.x), floor(vec.y / tileSize.y));
}

// converts a 2d vector to grid coordinates
sf::Vector2u OccupancyMap::MapToGridCoords(sf::Vector2i vec)
{
	return sf::Vector2u(floor(vec.x / tileSize.x), floor(vec.y / tileSize.y)); 
}

sf::Vector2f OccupancyMap::GetProbabilitiesAtPoint(sf::Vector2u vec)
{
	return sf::Vector2f(grid[vec.x][vec.y].occupied, grid[vec.x][vec.y].empty);
}

std::list<CELL> OccupancyMap::GetGridPointsInBounds(double *pBounds, sf::Vector2f rPos, double rTh, sf::Vector2f sPos, double sTh, double maxDistance, double robotRadius)
{
	std::list<CELL> temp;
	temp.clear();

	for (unsigned int i = 0; i < mapWidth; ++i)
	{
		for (unsigned int j = 0; j < mapHeight; ++j)
		{
			sf::Vector2f pos = grid[i][j].rect.getPosition();			

			// if inside bounding box
			if ((pBounds[0] <= pos.x && pos.x <= pBounds[2]) && (pBounds[1] <= pos.y && pos.y <= pBounds[3]))
			{
				//grid[i][j].rect.setFillColor(sf::Color::Blue);
				//printf(" %.2f < %.2f < %.2f  &&  %.2f < %.2f < %.2f \n", pBounds[0], pos.x, pBounds[2], pBounds[1], pos.y, pBounds[3]);

				double distance = sqrt((pow((rPos.x - pos.x), 2)) + (pow((rPos.y - pos.y), 2)));

				double dx = (pos.x - rPos.x); // grid point offset X from robot
				double dy = (pos.y - rPos.y); // grid point offset Y from robot
				double th = (180.0 / M_PI) * (atan2(dy,dx)); // in degrees
				
				//double relativeTh = (rTh + sTh) - th;
				//double relativeTh2 = (rTh + sTh) + th;

				double th1 = ((rTh + sTh) - 15);  // 15 is beta (beamwidth)
				double th2 = ((rTh + sTh) + 15);  // 15 is beta (beamwidth)

				// handle +- 180 boundaries.
				th1 = (th1 >= 180) ? (-180 + (th1 - 180)) : th1;
				th2 = (th2 >= 180) ? (-180 + (th2 - 180)) : th2;

				th1 = (th1 <= -180) ? (180 + (th1 + 180)) : th1;
				th2 = (th2 <= -180) ? (180 + (th2 + 180)) : th2;

				double aL = min(th1,th2);
				double aU = max(th1,th2);

				//printf("aL: %.2f < th: %.2f < aU: %.2f || [%i][%i] || grid x: %.2f y: %.2f ||dx: %.2f dy: %.2f rTh %.2f || sTh %.2f || adjustedTh %.2f||adjustedsTh %.2f \n", aL, th, aU, i, j, pos.x, pos.y, dx, dy, rTh, sTh, adjustedTh, adjustedsTh);

				// if inside cone
				if ( ((ceil(aL) <= th) && ( th <= floor(aU))) && (distance <= maxDistance) && (distance >= robotRadius) )
				//if (((ceil(aL) <= relativeTh) && (relativeTh <= floor(aU))) && (distance <= maxDistance) && (distance >= robotRadius))
				{
					//printf("aL: %.2f < th: %.2f < aU: %.2f || [%i][%i] || grid x: %.2f y: %.2f || rTh %.2f || sTh %.2f \n", aL, th, aU, i, j, pos.x, pos.y, rTh, sTh);
					//printf("aL: %.2f < relativeTh: %.2f < aU: %.2f || [%i][%i] || grid x: %.2f y: %.2f || rTh %.2f || sTh %.2f \n", floor(aL), relativeTh, ceil(aU), i, j, pos.x, pos.y, rTh, sTh);

					double relativeTh = (rTh + sTh) - th; // should be < beta

					grid[i][j].distance = distance;
					grid[i][j].angle = relativeTh;

					CELL point = CELL();

					point.x = pos.x;
					point.y = pos.y;
					point.i = i;
					point.j = j;
					point.distance = distance;
					point.angle = relativeTh;

					//grid[i][j].rect.setFillColor(sf::Color::Yellow);
					temp.push_back(point);
				}
			}
		}
	}

	return temp;
}


void OccupancyMap::UpdateGridPosition(int region, int i, int j, double probGridOccupied, double probGridEmpty, sf::Color colour)
{
	grid[i][j].occupied = probGridOccupied;
	grid[i][j].empty = probGridEmpty;
	
	sf::Color current = grid[i][j].rect.getFillColor();
	int tmp = 0;

	//if (region == 1)
	//{
		tmp = floor((255 * probGridOccupied));

		if (colour.r != current.r && colour.g != current.g && colour.b != current.b)
		{
			current = colour;
		}
	//}
	
	/*
	else if (region == 2)
	{
		tmp = floor((255 * probGridEmpty));

		if (colour.r != current.r && colour.g != current.g && colour.b != current.b)
		{
			current = colour;
		}
	}
	*/
	
	// map to [0,255]
	if (tmp >= 255)
	{
		tmp = 255;
	}
	else if (tmp <= 0)
	{

		tmp = 0;
	}

	current.a = (UINT8)tmp;

	grid[i][j].rect.setFillColor(current);
}
