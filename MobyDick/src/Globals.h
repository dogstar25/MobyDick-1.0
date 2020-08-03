#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>
#include <map>

#include <SDL2/SDL.h>

/*
Overloaded operators used throught the game
*/
static bool operator ==(SDL_Color a, SDL_Color b)
{
	return (a.r == b.r) && (a.g == b.g) && (a.b == b.b);
}

static bool operator !=(SDL_Color a, SDL_Color b)
{
	return (a.r != b.r) || (a.g != b.g) || (a.b != b.b);
}

static const size_t MAX_GAMEOBJECT_LAYERS = 3;
static const size_t CHILD_POSITIONS = 9;

//Mouse State
static const size_t MOUSE_NONE = 0;
static const size_t MOUSE_HOVER = 1;
static const size_t MOUSE_HOLD = 2;
static const size_t MOUSE_CLICKED = 3;

//Mouse Modes
static const size_t MOUSE_MODE_NAVIGATE = 0;
static const size_t MOUSE_MODE_CONTROLLER = 1;

//Collision Groups
static const size_t COLLISION_GENERIC = 1;
static const size_t COLLISION_PLAYER = 2;
static const size_t COLLISION_WALL = 4;
static const size_t COLLISION_PLAYER_BULLET = 8;
static const size_t COLLISION_PARTICLE1 = 16;
static const size_t COLLISION_PARTICLE2 = 32;
static const size_t COLLISION_PARTICLE3 = 64;
static const size_t COLLISION_ENEMY_FRAME = 128;
static const size_t COLLISION_ENEMY_ARMOR = 256;
static const size_t COLLISION_ENEMY_ARMOR_PIECE = 512;

//Components
static const size_t ACTION_COMPONENT = 1;
static const size_t ANIMATION_COMPONENT = 2;
static const size_t ATTACHMENTS_COMPONENT = 3;
static const size_t CHILDREN_COMPONENT = 4;
static const size_t COMPOSITE_COMPONENT = 5;
static const size_t TRANSFORM_COMPONENT = 6;
static const size_t PARTICLE_COMPONENT = 7;
static const size_t PHYSICS_COMPONENT = 8;
static const size_t RENDER_COMPONENT = 9;
static const size_t TEXT_COMPONENT = 10;
static const size_t VITALITY_COMPONENT = 11;
static const size_t WEAPON_COMPONENT = 12;
static const size_t PLAYERCONTROL_COMPONENT = 13;
static const size_t INVENTORY_COMPONENT = 14;

//Animation States
static const size_t ANIMATION_IDLE = 0;
static const size_t ANIMATION_RUN = 1;
static const size_t ANIMATION_ACTIVE = 3;
static const size_t ANIMATION_ACTION = 4;

//Player Control
static const size_t CONTROL_MOVEMENT = 1;
static const size_t CONTROL_USE = 2;

//Actions
static const size_t ACTION_NONE = 0;
static const size_t ACTION_MOVE = 1;
static const size_t ACTION_ROTATE = 2;
static const size_t ACTION_USE = 3;

//GameSpace Types
static const size_t GAMESPACE_INTRO = 0;
static const size_t GAMESPACE_PLAY = 1;
static const size_t GAMESPACE_GAMEOVER = 2;
static const size_t GAMESPACE_MENU = 3;

//Scene Action Codes
static const size_t SCENE_ACTION_QUIT = 0;
static const size_t SCENE_ACTION_ADD = 1;
static const size_t SCENE_ACTION_REPLACE = 2;
static const size_t SCENE_ACTION_EXIT = 3;

//Scene Tags
static const size_t SCENETAG_MENU = 1;

//Game Layers
static const size_t LAYER_BACKGROUND = 0;
static const size_t LAYER_MAIN = 1;
static const size_t LAYER_TEXT = 2;


namespace util
{

	const int generateRandomNumber(int min, int max);
	const float generateRandomNumber(float min, float max);
	const SDL_Color generateRandomColor();
	const float radiansToDegrees(float angleInRadians);
	const float degreesToRadians(float angleInDegrees);
	const std::string floatToString(float x, int decDigits);

};

/*
Game State
*/
enum class GameState{ 
	QUIT = 0,
	PLAY = 1,
	PAUSE = 2,
	EXIT = 3
};

enum class SceneState {
	RUN = 0,
	PAUSE = 1,
	EXIT = 2
};



#endif