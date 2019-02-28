#include "game.h"
#include "PlayerObject.h"
#include "LevelManager.h"
#include "TextureManager.h"
#include "GameObjectManager.h"
#include "GameObject.h"
#include "Util.h"
#include "Camera.h"
#include "Weapon.h"


Clock Game::clock;
Util Game::util;
TextureManager Game::textureManager;
GameObjectManager Game::gameObjectManager;
LevelManager Game::levelManager;
Config Game::config;
Camera Game::camera;
SDL_Rect Game::worldBounds;
b2World* Game::physicsWorld;
vector<GameObject*> Game::gameObjects;

using namespace chrono_literals;

bool Game::init()
{

	//Get all of the configuration values
	getConfig();

	//Initialize world
	if (SDL_Init(SDL_INIT_EVERYTHING) == 0) 
	{
		printf("SDL_Init success\n");

		//Create the game window
		pWindow = SDL_CreateWindow(this->gameTitle.c_str(),
			this->windowXpos,
			this->windowYPos,
			this->camera.frame.w, 
			this->camera.frame.h, 
			SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);

		//Initialize the textture manager
		this->textureManager.init(pWindow);

		// Construct a world object, which will hold and simulate the rigid bodies.
		this->physicsWorld = new b2World(this->gravity);
		//Add a collision contact listener
		this->physicsWorld->SetContactListener(&this->gameObjectContactListner);

		//Debug Mode
		if (this->debugDrawMode == true)
		{
			this->debugDraw.SetFlags(DebugDraw::e_shapeBit);
			this->physicsWorld->SetDebugDraw(&this->debugDraw);
		}

		//Initilaze the Game Object Manager
		//This will hold all possible game objects that the game/level supports
		this->gameObjectManager.init();

		//Create the main player object
		PlayerObject* playerObject = 
			(PlayerObject*)Game::gameObjectManager.buildGameObject("GINA_64", 5, 5);
		this->player = (PlayerObject*)playerObject;
		this->player->direction = 0;
		this->player->strafe = 0;
		playerObject->currentAnimationState = "IDLE";
		// Add a weapon that will have bullet origin that is located half way
		// in the X position and halfway in the Y position from this objects origin
		playerObject->addWeapon("BULLET1", .50, .50);
		
		//Set the mouse mode
		SDL_ShowCursor(false);
		SDL_SetRelativeMouseMode(SDL_TRUE);

		//Load level 1
		Game::levelManager.loadLevel("TX_LEVEL1");
		this->buildLevel("TX_LEVEL1");

		this->initWorldBounds();

		//Init Camera
		this->camera.init(&this->worldBounds);

		//set camera to center on player object
		this->camera.setPosition((this->player->physicsBody->GetPosition().x *  Game::config.scaleFactor) -
									(camera.frame.w / 2),
								 (this->player->physicsBody->GetPosition().y *  Game::config.scaleFactor) -
									(camera.frame.h / 2));

		//Initialize the clock object
		clock.init();

		bRunning = true;

	}


	//Add a few objects to the world
	GameObject *gameObject=nullptr;
	gameObject = Game::gameObjectManager.buildGameObject("BOWMAN", 1, 1);
	this->gameObjects.push_back(gameObject);

	gameObject = Game::gameObjectManager.buildGameObject("SWORDLADY", 12, 12);
	gameObject->currentAnimationState = "IDLE";
	this->gameObjects.push_back(gameObject);

	gameObject = Game::gameObjectManager.buildGameObject("BULLET1", 10, 10);
	gameObject->currentAnimationState = "ACTIVE";
	this->gameObjects.push_back(gameObject);

	return true;
}

void Game::update() {

	//Specifiaclly handle input and stuff for the one player gameObject
	this->player->updatePlayer();

	//Update the camera frame to point to the new player position
	this->camera.setPosition((this->player->physicsBody->GetPosition().x *  Game::config.scaleFactor) -
		(camera.frame.w / 2),
		(this->player->physicsBody->GetPosition().y *  Game::config.scaleFactor) -
		(camera.frame.h / 2));

	//Update all of the other non player related update chores for each game object
	this->awakeCount=0;
	for (auto & gameObject : gameObjects)
	{

		gameObject->update();
	}
	
	//cout << "awake count " << this->awakeCount << "\n";

	this->physicsWorld->Step(this->timeStep, this->velocityIterations, this->positionIterations);

}

void Game::render() {

	//Clear teh graphics display
	Game::textureManager.clear();

	//render the player
	Game::textureManager.render(this->player);
	
	//Render all of the game objects
	for (auto & gameObject : gameObjects) {
		Game::textureManager.render(gameObject);
	}

	//DebugDraw
	if (this->debugDrawMode == true)
	{
		this->physicsWorld->DrawDebugData();
	}

	//Push all drawn things to the graphics display
	Game::textureManager.present();

}

void Game::clean() {
	printf("cleaning game\n");
	//SDL_SetWindowFullscreen(pWindow, SDL_WINDOW_RESIZABLE);
	SDL_DestroyWindow(this->pWindow);
	SDL_Quit();
}

bool Game::getConfig()
{
	//Read file and stream it to a JSON object
	Json::Value root;
	ifstream ifs("assets/gameConfig.json");
	ifs >> root;

	//Get and store config values
	this->gameTitle = root["gameTitle"].asString();
	this->gravity.Set(root["physics"]["gravity"]["x"].asInt(), root["physics"]["gravity"]["y"].asFloat());

	this->timeStep = root["physics"]["timeStep"].asFloat();
	this->velocityIterations = root["physics"]["velocityIterations"].asInt();
	this->positionIterations = root["physics"]["positionIterations"].asInt();
	this->debugDrawMode = root["physics"]["debugDrawMode"].asBool(); 

	this->config.scaleFactor = root["physics"]["box2dScale"].asFloat();
	this->config.mouseSensitivity = root["mouseSensitivity"].asFloat();

	this->camera.frame.w = root["camera"]["width"].asInt();
	this->camera.frame.h = root["camera"]["height"].asInt();

	return true;
}

void Game::handleEvents() {
	SDL_Event event;
	if (SDL_PollEvent(&event)) {

		switch (event.type) {
		case SDL_QUIT:
			bRunning = false;
			break;

		case SDL_KEYDOWN:
		case SDL_KEYUP:
		case SDL_MOUSEMOTION:
			if ((char)event.key.keysym.sym == SDLK_ESCAPE && event.type == SDL_KEYDOWN)
			{
				event.type = SDL_QUIT;
				SDL_PushEvent(&event);
			}
			else
			{
				this->player->handlePlayerMovementEvent(&event);
			}
			break;
		case SDL_MOUSEBUTTONDOWN:
			//this->testBlocks(&event, this->physicsWorld);
			this->player->weapon->fire();
			std::cout << "FPS is " << Game::clock.fps << "\n";
			std::cout << "bodycount is " << Game::gameObjectManager.box2dBodyCount << "\n";
			std::cout << "gameobject count is " << Game::gameObjects.size() << "\n";
			break;
		default:
			break;
		}
	}
}

void Game::buildLevel(string levelId)
{
	this->currentLevel = levelId;
	Level* level = Game::levelManager.levels[levelId];
	LevelObject* levelObject;
	GameObject* gameObject;

	for (int y = 0; y < level->height; y++)
	{
		for (int x = 0; x < level->width; x++)
		{

			if (level->levelObjects[x][y].gameObjectId.empty() == false)
			{
				levelObject = &level->levelObjects[x][y];
				gameObject = Game::gameObjectManager.buildGameObject(levelObject->gameObjectId,	
								x, y, levelObject->angleAdjustment);

				this->gameObjects.push_back(gameObject);

				//Use the first level object found to determine and store the tile width and height for the map
				if (level->tileHeight == 0 and level->tileWidth == 0)
				{
					level->tileWidth = gameObject->definition->xSize * Game::config.scaleFactor;
					level->tileHeight = gameObject->definition->ySize * Game::config.scaleFactor;
				}
			}

		}
	}
}

void Game::initWorldBounds()
{
	int width, height;

	width = this->levelManager.levels[this->currentLevel]->width *
		this->levelManager.levels[this->currentLevel]->tileWidth;

	height = this->levelManager.levels[this->currentLevel]->height *
		this->levelManager.levels[this->currentLevel]->tileHeight;

	this->worldBounds.x = 0;
	this->worldBounds.y = 0;
	this->worldBounds.w = width;
	this->worldBounds.h = height;

}

void Game::testBlocks(SDL_Event* event, b2World* physicsWorld)
{

	//std::cout << "Object created " << " \n";
	//std::cout << "X " << event->button.x << " \n";
	//std::cout << "Y " << event->button.y << " \n";

	GameObject* gameObject;
	gameObject = new GameObject();
	gameObject->definition = new GameObjectDefinition();

	//build id
	int count = this->gameObjects.size();
	string id = "block" + to_string(count);
	gameObject->definition->id = id;

	gameObject->definition->description = "block";
	//gameObject->xSize = Game::util.generateRandomNumber(1,30) * .1;
	//gameObject->ySize = Game::util.generateRandomNumber(1, 30) * .1;
	gameObject->definition->xSize = .15;
	gameObject->definition->ySize = .15;

	gameObject->definition->initPosX = event->button.x / Game::config.scaleFactor;
	gameObject->definition->initPosY = event->button.y / Game::config.scaleFactor;

	//gameObject->isPrimitiveShape = true;
	//gameObject->primativeColor = Game::util.generateRandomColor();

	string textureId = "TX_TILE1";
	gameObject->staticTexture = Game::textureManager.getTexture(textureId)->texture;

	gameObject->definition->isPhysicsObject = true;
	//gameObject->definition->physicsType = "B2_STATIC";
	gameObject->definition->physicsType = "B2_DYNAMIC";
	gameObject->definition->friction = .5;
	gameObject->definition->density = 10.0;
	gameObject->definition->linearDamping = 0;
	gameObject->definition->angularDamping = 0;
	gameObject->physicsBody = Game::gameObjectManager.buildB2Body(gameObject->definition);

	this->gameObjects.push_back(gameObject);

}

Game::~Game()
{

	printf("cleaning game\n");

	//Delete SDL stuff
	SDL_DestroyWindow(this->pWindow);
	SDL_Quit();


	//Delete box2d world - should delete all bodies and fixtures within
	delete this->physicsWorld;

}

Game::Game()
{

	

}


