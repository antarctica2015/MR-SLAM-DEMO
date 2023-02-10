#pragma once
#include <SFML\Graphics.hpp>
#include <SFML\Window.hpp>
#include <SFML\System.hpp>


// Polar coords of grid point being checked
struct POLAR
{
	double r; // Radial coordinate
	double alpha; // Angular coordinate
};

struct CELL
{
	//int id;
	double x;
	double y;
	int i;
	int j;
	double occupied;
	double empty;
	double distance; // distance from robot to this grid point
	double angle; // angle from robot to this grid point
	sf::RectangleShape rect;
	int region;
};

class OccupancyMap : public sf::Drawable, public sf::Transformable
{
public:
	OccupancyMap();
	~OccupancyMap();

	//colors to use - start 50% alpha (confidence)
	sf::Color unknownColour = sf::Color(127.5, 127.5, 127.5, 255); //grey
	sf::Color emptyColour = sf::Color(255, 255, 255, 255); //white
	sf::Color obstacleColour = sf::Color(0, 0, 0, 255); //black
	sf::Color clearColour = sf::Color::Transparent; //

	CELL grid[100][100];

	//returns i,j for input coordinates
	sf::Vector2u MapToGridCoords(double x, double y);
	sf::Vector2u MapToGridCoords(sf::Vector2f);
	sf::Vector2u MapToGridCoords(sf::Vector2i);

	sf::Vector2f GetProbabilitiesAtPoint(sf::Vector2u);

	std::list<CELL> GetGridPointsInBounds(double *pBounds, sf::Vector2f point1, double rTh, sf::Vector2f point2, double sTh, double maxDistance, double robotRadius);

	void UpdateGridPosition(int region, int i, int j, double probGridOccupied, double probGridEmpty, sf::Color colour);

private:
	const sf::Vector2u tileSize = sf::Vector2u(10, 10);

	const unsigned int mapWidth = 100U;
	const unsigned int mapHeight = 100U;

	const sf::Vector2i mapOrigin = sf::Vector2i(0, 0); // what is the grid space origin square.

	sf::RectangleShape sonarBox;

	// draw the tilemap to screen
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
	{
		// apply the transform
		states.transform *= getTransform();

		for (unsigned int i = 0; i < mapWidth; ++i)
		{
			for (unsigned int j = 0; j < mapHeight; ++j)
			{
				target.draw(grid[i][j].rect);
			}
		}
	}
};



