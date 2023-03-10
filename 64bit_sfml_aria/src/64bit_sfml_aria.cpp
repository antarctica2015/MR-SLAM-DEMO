#include "Aria.h"
#include "ArActionExplore.h"
#include <SFML\Graphics.hpp>
#include <SFML\Window.hpp>
#include <SFML\System.hpp>

#include "OccupancyMap.h"

#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

const int window_width = 800;
const int window_height = 800;

sf::Color sfml_windowBackgroundColor = sf::Color::White;
//sf::Color sfml_windowBackgroundColor = sf::Color(127.5, 127.5, 127.5, 255);
//sf::Color sfml_windowBackgroundColor = sf::Color(0, 0, 127.5);

//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------

// TODO: move all aria stuff into its own Robot class to tidy up?
int main(int argc, char **argv)
{
	//
	OccupancyMap *pMap = new OccupancyMap();

	//______________________________________________________________________________________________________________________
	//______________________________________________________________________________________________________________________ ROBOT
	Aria::init();																		//Initialise ARIA library
	ArArgumentParser argParser(&argc, argv);											//Initialise argument parser
	argParser.loadDefaultArguments();													//Load standard arguments
	ArRobot robot;																	//Initialise Robot object
	ArRobotConnector robotConnector(&argParser, &robot);								//Initialise Robot Connector

	if (!robotConnector.connectRobot())												//Check if robot is running
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())					//Log options
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	ArKeyHandler keyHandler;															//Escape Key Handler
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	puts("Press Escape to exit.");

	//
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	std::cout << robot.getRobotRadius() << std::endl;

	robot.runAsync(true);
	robot.enableMotors();																//Enable Motors
	robot.setAbsoluteMaxLatVel(200);

	// Subsumption Actions
	ArActionStallRecover recover;
	ArActionConstantVelocity move;
	//ArActionAvoidFront frontavoid;
	ArActionAvoidFront avoid;
	ArActionBumpers bumpers;

	//sf::Text *pDebugText;
	
	//unsigned int MaxRange = sonar.getMaxRange();
	sf::CircleShape *pMarker = new sf::CircleShape();
	pMarker->setRadius(34.0f); //pMarker->setRadius(10.0f);
	pMarker->setOrigin(34.0, 34.0);
	pMarker->setFillColor(sf::Color::Red);

	sf::RectangleShape *pLine = new sf::RectangleShape();
	pLine->setSize(sf::Vector2f(pMarker->getRadius(), 2.0f));
	
	//
	sf::CircleShape *pSonarResult = new sf::CircleShape();
	pSonarResult->setRadius(5.0f);
	pSonarResult->setOrigin(5.0, 5.0);
	pSonarResult->setFillColor(sf::Color::Green);
	
	ArActionExplore explore;  //update the OccupancyMap
	explore.Init(pMap, pMarker, pLine, pSonarResult);

	//
    robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	robot.addAction(&move, 60);
	//robot.addAction(&frontavoid, 90);
	robot.addAction(&avoid, 95);
	robot.addAction(&explore, 90);
	
	//______________________________________________________________________________________________________________________
	//______________________________________________________________________________________________________________________ MAPPING

	/*
	// 
	view.setCenter(sf::Vector2f(350.f, 300.f));
    view.setSize(sf::Vector2f(200.f, 200.f));
	view.zoom(0.5f);
	
	// activate it
	window.setView(view);

	// define a centered viewport, with half the size of the window
	view.setViewport(sf::FloatRect(0.25f, 0.25, 0.5f, 0.5f));
	*/

	/*--------------------------------------------------------------------------------------------------------------*/

	/*
	sf::RectangleShape rect;
	rect.setFillColor(sf::Color::Green);
	rect.setPosition(250, 250);
	rect.setSize(sf::Vector2f(150,50));
	rect.rotate(-20);
	*/

	//make a nice debug line
	/*
	sf::Vertex line[] =
	{
		sf::Vertex(sf::Vector2f(pMarker->getPosition().x, pMarker->getPosition().y)),
		sf::Vertex(sf::Vector2f(pSonarResult->getPosition().x, pSonarResult->getPosition().y))
	};
	*/

	/*--------------------------------------------------------------------------------------------------------------*/

	sf::RenderWindow window(sf::VideoMode(window_width, window_height), "sfmlViewer");
	window.setVerticalSyncEnabled(true); // call it once, after creating the window
										 // window.setFramerateLimit(10);


	sf::View view(sf::Vector2f(0.f, 0.f), sf::Vector2f(800.f, 800.f));
	view.zoom(1.0f);
	window.setFramerateLimit(30);

	/*--------------------------------------------------------------------------------------------------------------*/
	// Declare and load a font
	sf::Font font;
	font.loadFromFile("./resources/sansation.ttf");

	sf::Text posText;
	posText.setFont(font);
	posText.setCharacterSize(24); // in pixels, not points!
	posText.setPosition(-395.f, -400.f);
	posText.setColor(sf::Color::White);

	//
	std::stringstream ss;
	std::stringstream str;
	/*--------------------------------------------------------------------------------------------------------------*/
	//window.mapPixelToCoords(pos, standard).x << ", " << window.mapPixelToCoords(pos, standard).y << ">";
	//pos = sf::Mouse::getPosition(window); // Get the new position
    //sf::Vector2i pos = sf::Mouse::getPosition();
    /*--------------------------------------------------------------------------------------------------------------*/

	sf::Color pointColour; 
	sf::Vector2i pos = sf::Mouse::getPosition();
	sf::Vector2f probs = sf::Vector2f(0,0);
	sf::Vector2u  tmp = sf::Vector2u(0,0);
	double dist = 0.0f;
	double alpha = 0.0f;

	while (window.isOpen()) 
	{
		sf::Event event;
		while (window.pollEvent(event)) 
		{
			bool moved = false;

			// catch the resize events
			if (event.type == sf::Event::Resized)
			{
				// update the view to the new size of the window
				sf::FloatRect visibleArea(0.f, 0.f, event.size.width, event.size.height);
				window.setView(sf::View(visibleArea));
			}
			else if ((event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Left) || (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::A))
			{
				// Move left
				view.move(-10.0f, 0.0f);
				moved = true;
			}
			else if ((event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Right) || (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::D))
			{
				// Move right
				view.move(10.0f, 0.0f);
				moved = true;
			}
			else if ((event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Up) || (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::W))
			{
				// Move up
				view.move(0.0f, -10.0f);
				moved = true;
			}
			else if ((event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Down) || (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::S))
			{
				// Move down
				view.move(0.0f, 10.0f);
				moved = true;
			}
			//else if (event.type == sf::Event::MouseButtonPressed || event.type == sf::Event::MouseButtonReleased)
			//{
				//sf::Vector2i pos = sf::Mouse::getPosition();
				//ss.str("");
				//ss << "Selected : (" << window.mapPixelToCoords(pos, view).x << ", " << window.mapPixelToCoords(pos, view).y << ")";
				//sf::Vector2u  tmp = pMap->MapToGridCoords(pos);
				//sf::Vector2f probs = pMap->GetProbabilitiesAtPoint(tmp);
			//}


			if (event.type == sf::Event::MouseButtonPressed || event.type == sf::Event::MouseButtonReleased)
			{
				pos = sf::Mouse::getPosition(window);
				tmp = pMap->MapToGridCoords(window.mapPixelToCoords(pos, view).x, window.mapPixelToCoords(pos, view).y);
				probs = pMap->GetProbabilitiesAtPoint(tmp);

				pointColour = pMap->grid[tmp.x][tmp.y].rect.getFillColor();

				//pMap->grid[tmp.x][tmp.y].rect.setFillColor(sf::Color::Yellow);
				dist = pMap->grid[tmp.x][tmp.y].distance;
				alpha = pMap->grid[tmp.x][tmp.y].angle;
			}
		}

		ss.str("");
		//ss << "<" << pos.x << ", " << pos.y << ">\t<" << window.mapPixelToCoords(pos, view).x << ", " << window.mapPixelToCoords(pos, view).y << "> :: [" << tmp.x << "][" << tmp.y << "] \t (O: " << probs.x << ", E: " << probs.y << ", D:" << dist << ", A: " << alpha << ")";

		ss << "[" << tmp.x << "][" << tmp.y << "] (O:" << probs.x << ", E:" << probs.y << ", D:" << dist << ", A:" << alpha << ")";
		//posText.setString(ss.str());

		str.str("");
		str << " R:" << (int)pointColour.r << " G:" << (int)pointColour.g <<  " B:" << (int)pointColour.b << " A:" << (int)pointColour.a;
		posText.setString(ss.str() + str.str());

		//
		float tmpX = pSonarResult->getPosition().x;
		float tmpY = pSonarResult->getPosition().y;
			
		/*******************************************************************/

		//Update and render Occupancy grid here....
		window.clear(sfml_windowBackgroundColor); 

		//
		window.setView(view);
		
		//
		window.draw(*pMap);
		window.draw(*pMarker);
		window.draw(*pLine);
		window.draw(*pSonarResult);
		//window.draw(line, 2, sf::Lines);

		window.draw(posText);

		window.display();

		/*******************************************************************/
	}

	//______________________________________________________________________________________________________________________
	//______________________________________________________________________________________________________________________ ARIA EXIT

	robot.waitForRunExit();																//Wait for robot task loop to end before exiting the program				
	Aria::exit(0);																		//Exit Aria
}





