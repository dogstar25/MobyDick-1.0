#pragma once
#include <string>

using namespace std;

class GameObject;
class PlayerObject;
class Game;

class Weapon
{
public:
	Weapon();
	~Weapon();

	void fire();
	void fireEmitter();
	void init(string, PlayerObject*, float,float);
	


	PlayerObject* weaponWieldingObject;
	string bulletGameObjectId;
	float xOffset, yOffset; //what offset from the weapon holding obejct to originate bullet from
	float stength;

};

