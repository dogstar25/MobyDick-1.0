#pragma once
#include "Waypoint.h"
#include <string>
#include <vector>

#include <SDL2/SDL.h>
#include <Box2D/Box2D.h>
#include <map>


struct LevelObject
{
	std::string gameObjectId;
	int angleAdjustment;
};

class Level
{
public:

	std::string m_id; //probably same as the textureId since the map is represented by a texture
	int m_width, m_height; // in tile count
	float m_tileWidth, m_tileHeight;

	std::vector< std::vector <LevelObject> > levelObjects;

	void addWaypoint(Waypoint waypoint);
	void addLevelObject(int xIndex, int yIndex, LevelObject levelObject);
	void setLevelObjectArraySize(int width, int height);

	void load(std::string levelId);

	//Accessor Functions
	std::string description() {
		return m_description;
	}
	std::vector<Waypoint> waypoints() {
		return m_waypoints;
	}

private:
	std::string m_description;
	std::string m_blueprint;

	std::vector<Waypoint> m_waypoints;
	std::map<std::string, LevelObject*> m_locationObjects;

	LevelObject* _determineTile(int, int, SDL_Surface*);
	void _loadDefinition(std::string levelId);
	void _buildLevelObjects();

};



