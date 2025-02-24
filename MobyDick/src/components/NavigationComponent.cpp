#include "NavigationComponent.h"
#include "../EnumMap.h"
#include "../RayCastCallBack.h"
#include "../game.h"
#include <thread>
#include <glm/glm.hpp>
#include <queue>

extern std::unique_ptr<Game> game;

NavigationComponent::~NavigationComponent()
{

	if (m_pathfindingThread.joinable()) {
		m_pathfindingThread.join();
	}
}

NavigationComponent::NavigationComponent(Json::Value componentJSON)
{
	m_componentType = ComponentTypes::NAVIGATION_COMPONENT;

	if (componentJSON.isMember("passageFitSizeCategory")) {
		m_passageFitSizeCategory = game->enumMap()->toEnum(componentJSON["passageFitSizeCategory"].asString());
	}

	m_pathRefreshTimer = { 0.5, true };

	//Based on the size of the object, calculate the destination met tolerance
	auto tileSize = game->worldTileSize().x;
	m_navigationDestinationTolerance = float((m_passageFitSizeCategory + 1) * tileSize / 2);
	m_navigationStuckTolerance = float((m_passageFitSizeCategory + 1) * tileSize / 16);

}

void NavigationComponent::navigateStop()
{

	b2Vec2 trajectory{ 0,0 };
	float angularVelocity{ 0. };

	auto actionComponent = parent()->getComponent<ActionComponent>(ComponentTypes::ACTION_COMPONENT);

	if (actionComponent->getAction(ACTION_MOVE)) {
		const auto& moveAction = actionComponent->getAction(ACTION_MOVE);
		moveAction->perform(parent(), trajectory);
	}

	//if (actionComponent->getAction(ACTION_ROTATE)) {
	//	const auto& moveAction = actionComponent->getAction(ACTION_ROTATE);
	//	moveAction->perform(parent(), angularVelocity);
	//}
}

NavigationStatus NavigationComponent::navigateTo(float pixelX, float pixelY, int fuzzyFactor)
{
	bool destinationChanged{};

	//We need to check if the last thread was successful in finding a path before we start a new thread
	{
		// Check if a new path is available
		std::lock_guard<std::mutex> lock(m_pathMutex);
		if (m_unableToFindPath == true && m_pathInProgress == false) {
			m_unableToFindPath = false;
			return NavigationStatus::NO_PATH_FOUND;
		}

	}

	//Is the nagigation destination changing
	if (m_targetPixelDestination.x != pixelX || m_targetPixelDestination.y != pixelY) {
		destinationChanged = true;
	}

	m_targetPixelDestination = { pixelX, pixelY };
	m_targetTileDestination = util::pixelToTileLocation(pixelX, pixelY);

	bool pathFound{};

	//If we have been stuck in the same spot for a few seconds then
	//return stuck status and let the brain decide what to do
	if (_isStuck()) {
		return NavigationStatus::STUCK;
	}

	// If the navigatioMap changed based on conditional walls and such
	// OR
	// this objects destination changed, then
	if (parent()->parentScene()->navigationMapChanged() == true || 
		destinationChanged == true ||
		m_solutionPath.empty() == true) {

		m_solutionPath.clear();
		m_currentNavStep = 0;
		
		if (!m_pathInProgress) {
			_startPathfinding(fuzzyFactor);
		}

	}

	//We need to check if the last thread was successful in finding a path before we start a new thread
	{
		// Check if a new path is available
		std::lock_guard<std::mutex> lock(m_pathMutex);
		if (m_newPathAvailable) {
			m_solutionPath = std::move(m_newSolutionPath);
			m_newPathAvailable = false;
			m_currentNavStep = 0;
		}
	}

	//The solution path will be blank the first time calculating a path so return and do nothing
	if (m_solutionPath.empty()) {
		return NavigationStatus::IN_PROGRESS;
	}

	//Have we reached the targetDestination
	if (util::calculateDistance(parent()->getCenterPosition(), m_targetPixelDestination) < m_navigationDestinationTolerance) {
	
		m_solutionPath.clear();
		return NavigationStatus::DESTINATION_REACHED;
	}

	//Have we reached the next interim navigation step
	const auto objectSize = parent()->getSize();
	SDL_Point interimNavStepTileLocation = m_solutionPath.at(m_currentNavStep);
	SDL_FPoint interimNavStepPixelLocation = util::tileToPixelLocation(
		(float)interimNavStepTileLocation.x, (float)interimNavStepTileLocation.y);

	//Distance between our master oobject and the destination
	float targetDistance = util::calculateDistance(parent()->getCenterPosition(), interimNavStepPixelLocation);

	if (targetDistance < m_navigationDestinationTolerance) {

		if ((m_currentNavStep+1) < m_solutionPath.size()) {
			m_currentNavStep++;
		}

	}

	//Execute Moves to get to targetDestination
	_moveTo(m_solutionPath.at(m_currentNavStep));

	return NavigationStatus::IN_PROGRESS;
}

void NavigationComponent::_startPathfinding(int fuzzyFactor) {
	
	if (m_pathInProgress) return; // Avoid starting multiple threads

	m_pathInProgress = true;
	
	{
		// Ensure `m_unableToFindPath` is updated safely
		std::lock_guard<std::mutex> lock(m_pathMutex);
		m_unableToFindPath = false;
	}

	// Launch a new thread for pathfinding
	m_pathfindingThread = std::thread([this, fuzzyFactor]() {
		std::vector<SDL_Point> solutionPath;

		// Perform pathfinding (expensive operation)
		if (_buildPathToDestination(fuzzyFactor, solutionPath)) {
			// Lock mutex to safely update the new solution path
			std::lock_guard<std::mutex> lock(m_pathMutex);
			m_newSolutionPath = std::move(solutionPath);
			m_newPathAvailable = true;
		}
		else {
			// If no path was found, update the flag safely
			std::lock_guard<std::mutex> lock(m_pathMutex);
			m_unableToFindPath = true;
		}

		m_pathInProgress = false;
		});

	// Detach the thread to allow it to run independently
	m_pathfindingThread.detach();
}

void NavigationComponent::update()
{

	int todd = 1;

}

void NavigationComponent::postInit()
{

}

bool NavigationComponent::_buildPathToDestination(int fuzzyFactor, std::vector<SDL_Point>& solutionPath) {

	using TileKey = std::tuple<int, int>; // Define a type alias for the tuple key
	std::map<TileKey, std::shared_ptr<AStarNode>> toSearch;
	std::map<TileKey, std::shared_ptr<AStarNode>> processed;

	// Get current position
	SDL_FPoint currentPosition = parent()->getCenterPosition();
	SDL_Point tileLocation = util::pixelToTileLocation(currentPosition.x, currentPosition.y);
	TileKey startingKey = std::make_tuple(tileLocation.x, tileLocation.y);

	// Create the starting node
	std::shared_ptr<AStarNode> startingNode = std::make_shared<AStarNode>();
	startingNode->position = tileLocation;
	toSearch[startingKey] = startingNode;

	// Reset grid display (for debugging/visualization)
	parent()->parentScene()->resetGridDisplay();

	// While there are nodes left to search
	while (!toSearch.empty()) {
		// Find the node with the lowest fCost
		auto currentItr = toSearch.begin();
		auto currentNode = currentItr->second;

		for (auto it = toSearch.begin(); it != toSearch.end(); ++it) {

			auto nodeCandidate = it->second;
			if (nodeCandidate->fCost < currentNode->fCost ||
				(nodeCandidate->fCost == currentNode->fCost && nodeCandidate->hCost < currentNode->hCost)) {
				currentNode = nodeCandidate;
				currentItr = it;
			}

		}

		// Add the current node to processed and remove it from toSearch
		TileKey currentKey = std::make_tuple(currentNode->position.x, currentNode->position.y);
		processed[currentKey] = currentNode;
		toSearch.erase(currentItr);

		// Check if the current node is the destination
		if (_foundDestinationNode(m_targetTileDestination, currentNode, fuzzyFactor)) {
			// Build the solution path by traversing back
			AStarNode* pathNode = currentNode.get();
			solutionPath.insert(solutionPath.begin(), pathNode->position);

			//if (pathNode->position.x == startingNode->position.x &&
			//	pathNode->position.y == startingNode->position.y) {

			//	break;
			//}

			while (pathNode->position.x != tileLocation.x || pathNode->position.y != tileLocation.y) {

				TileKey connectionKey = std::make_tuple(pathNode->connectionKeyX, pathNode->connectionKeyY);
				pathNode = processed[connectionKey].get();
				solutionPath.insert(solutionPath.begin(), pathNode->position);

				//test
			//parent()->parentScene()->updateGridDisplay(pathNode->position.x, pathNode->position.y, TURN_ON, Colors::GREEN);
				/////
			}

			

			return true; // Path found
		}

		// Get neighbors and evaluate them
		std::vector<std::shared_ptr<AStarNode>> neighbors;
		_buildNeighbors(*currentNode, neighbors);

		for (auto& neighbor : neighbors) {
			TileKey neighborKey = std::make_tuple(neighbor->position.x, neighbor->position.y);

			// Skip if the neighbor has already been processed
			if (processed.find(neighborKey) != processed.end()) {
				continue;
			}

			// Calculate costs for the neighbor
			_calculateCosts(startingNode.get(), neighbor.get());

			// Add neighbor to toSearch or update its connection if this path is better
			auto it = toSearch.find(neighborKey);
			if (it == toSearch.end() || neighbor->gCost < it->second->gCost) {

				neighbor->connectionKeyX = currentNode->position.x;
				neighbor->connectionKeyY = currentNode->position.y;
				toSearch[neighborKey] = neighbor;
			}
		}
	}


	if (m_targetTileDestination.x==33 && m_targetTileDestination.y == 27) {

		int todd = 1;

	}

	return false; // No path found
}


bool NavigationComponent::_foundDestinationNode(SDL_Point destinationTile, std::shared_ptr<AStarNode> currentNode, int fuzzyFactor)
{

	//Exact location match
	if (destinationTile.x == currentNode->position.x &&
		destinationTile.y == currentNode->position.y) {
		return true;
	}

	if (fuzzyFactor > 0) {

		auto matchFound = false;

		for (auto adj = 1; adj <= fuzzyFactor; adj++) {
			//top left
			if (destinationTile.x -adj  == currentNode->position.x &&
				destinationTile.y -adj == currentNode->position.y) {
				matchFound = true;
				break;
			}
			//top
			if (destinationTile.x == currentNode->position.x &&
				destinationTile.y - adj == currentNode->position.y) {
				matchFound = true;
				break;
			}
			//top right
			if (destinationTile.x + adj == currentNode->position.x &&
				destinationTile.y + adj == currentNode->position.y) {
				matchFound = true;
				break;
			}
			//left
			if (destinationTile.x - adj == currentNode->position.x &&
				destinationTile.y  == currentNode->position.y) {
				matchFound = true;
				break;
			}
			//right
			if (destinationTile.x + adj == currentNode->position.x &&
				destinationTile.y == currentNode->position.y) {
				matchFound = true;
				break;
			}
			//bottom left
			if (destinationTile.x - adj == currentNode->position.x &&
				destinationTile.y - adj == currentNode->position.y) {
				matchFound = true;
				break;
			}
			//bottom right
			if (destinationTile.x + adj == currentNode->position.x &&
				destinationTile.y + adj == currentNode->position.y) {
				matchFound = true;
				break;
			}

		}

		return matchFound;
	}

	return false;
}

bool NavigationComponent::_isStuck()
{
	bool isStuck{};

	//Get current location
	float distanceTraveled = util::calculateDistance(parent()->getCenterPosition(), m_previousLocation);
	if (distanceTraveled < m_navigationStuckTolerance) {

		if (m_stuckTimer.isSet() == false) {
			m_stuckTimer = { 2 };
			//std::cout << "Reset Timer" << std::endl;
		}
		else {
			if (m_stuckTimer.hasMetTargetDuration()) {
				m_stuckTimer = { 0 };
				//Fcoutstd::cout << "Stuck!" << std::endl;
				isStuck = true;
			}
		}
	}
	else {
		m_stuckTimer = { 0 };
	}

	m_previousLocation = parent()->getCenterPosition();

	return isStuck;


}


void NavigationComponent::_calculateCosts(AStarNode* startingNode, AStarNode* node)
{

	//Calculate GCost - distance from start
	node->gCost = util::calculateDistance(startingNode->position, node->position);

	//Calculate the HCost - distance from the destination
	node->hCost = util::calculateDistance(node->position, m_targetTileDestination);

	//Calculate the FCost which is the total of the other 2 costs
	node->fCost = node->hCost + node->gCost;

}

void NavigationComponent::_buildNeighbors(AStarNode& currentNode, std::vector<std::shared_ptr<AStarNode>>& neighbors)
{
	int x{};
	int y{};

	//TopLeft
	x = currentNode.position.x - 1;
	y = currentNode.position.y - 1;
	_addNeighbor(x, y, neighbors);

	//Top
	x = currentNode.position.x;
	y = currentNode.position.y - 1;
	_addNeighbor(x, y, neighbors);

	//TopRight
	x = currentNode.position.x + 1;
	y = currentNode.position.y - 1;
	_addNeighbor(x, y, neighbors);

	//Left
	x = currentNode.position.x - 1;
	y = currentNode.position.y;
	_addNeighbor(x, y, neighbors);

	//Right
	x = currentNode.position.x + 1;
	y = currentNode.position.y;
	_addNeighbor(x, y, neighbors);

	//BottomLeft
	x = currentNode.position.x - 1;
	y = currentNode.position.y + 1;
	_addNeighbor(x, y, neighbors);

	//Bottom
	x = currentNode.position.x;
	y = currentNode.position.y + 1;
	_addNeighbor(x, y, neighbors);

	//BottomRight
	x = currentNode.position.x + 1;
	y = currentNode.position.y + 1;
	_addNeighbor(x, y, neighbors);

}

void NavigationComponent::_addNeighbor(int x, int y, std::vector<std::shared_ptr<AStarNode>>& neighbors) 
{

	if (_isValidNode(x, y)) {
        // Create a new AStarNode
        std::shared_ptr<AStarNode> starNode = std::make_shared<AStarNode>();
        starNode->position = { x, y }; // Set position directly
        
		// Set the connection keys to the parent node's position
		starNode->connectionKeyX = x;
		starNode->connectionKeyY = y;

        // Add to neighbors
        neighbors.push_back(starNode);
    }
}

bool NavigationComponent::_isValidNode(const int x, const int y) 
{

	const auto& navMap = parent()->parentScene()->navigationMap();
	int xMax = navMap.size();
	int yMax = navMap[0].size();
	bool passable{false};

	//Check map boundaries - small navigating object
	if (x >= 0 && x < xMax && y > 0 && y < yMax) {

		//Check if a passable object
		if (navMap[x][y].passable == false) {

			passable = false;
		}
		else {
			passable = true;
		}

		// If this tile was passable and this is a medium size navigating object
		// then also check each tile around this one
		if (m_applyPassageFit ==true && m_passageFitSizeCategory > NavigationSizeCategory::SMALL && passable == true ) {

			//first check medium
			if (m_passageFitSizeCategory == NavigationSizeCategory::MEDIUM || m_passageFitSizeCategory == NavigationSizeCategory::LARGE) {
				passable = _applyNavObjectSizeCheck(x, y, m_passageFitSizeCategory);
			}
			//If we're still passable and we're a large object then check for large
			if (m_passageFitSizeCategory == NavigationSizeCategory::LARGE && passable) {
				passable = _applyNavObjectSizeCheck(x, y, m_passageFitSizeCategory);
			}

		}
	}

	return passable;

}


bool NavigationComponent::_applyNavObjectSizeCheck(int x, int y, int objectCategory)
{
	const auto& navMap = parent()->parentScene()->navigationMap();
	int xMax = navMap.size();
	int yMax = navMap[0].size();
	bool passable{ true };

	SDL_Point sizeAdjPoint{};

	//Top Left
	sizeAdjPoint = { x - objectCategory, y - objectCategory };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Top
	sizeAdjPoint = { x, y - objectCategory };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Top Right
	sizeAdjPoint = { x + objectCategory, y - objectCategory };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Left
	sizeAdjPoint = { x - objectCategory, y };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Right
	sizeAdjPoint = { x + objectCategory, y };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Bottom Left
	sizeAdjPoint = { x - objectCategory, y + objectCategory };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Bottom 
	sizeAdjPoint = { x, y + objectCategory };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	//Bottom Right
	sizeAdjPoint = { x + objectCategory, y + objectCategory };
	if (sizeAdjPoint.x >= 0 && sizeAdjPoint.x < xMax && sizeAdjPoint.y >= 0 && sizeAdjPoint.y < yMax) {
		if (navMap[sizeAdjPoint.x][sizeAdjPoint.y].passable == false) {
			passable = false;
		}
	}

	return passable;

}

void NavigationComponent::_moveTo(SDL_Point destinationTile)
{

	//Need the objects size to calc the right pixel position
	const auto objectSize = parent()->getSize();

	SDL_FPoint destinationPixelLoc = util::tileToPixelLocation((float)destinationTile.x, (float)destinationTile.y);

	b2Vec2 trajectory{};
	trajectory.x = destinationPixelLoc.x - parent()->getCenterPosition().x;
	trajectory.y = destinationPixelLoc.y - parent()->getCenterPosition().y;

	/// debug line
	//float x = parent()->getCenterPosition().x;
	//float y = parent()->getCenterPosition().y;

	//glm::vec2 startPoint = { x, y};
	//glm::vec2 endPoint = { destinationPixelLoc.x, destinationPixelLoc.y };
	//glm::uvec4 color = { 255,255,255,255 };
	//game->renderer()->addLine(startPoint, endPoint, color);
	///

	trajectory.Normalize();

	const auto& actionComponent = parent()->getComponent<ActionComponent>(ComponentTypes::ACTION_COMPONENT);
	const auto& moveAction = actionComponent->getAction(ACTION_MOVE);

	moveAction->perform(parent(), trajectory);

	_applyAvoidanceMovement(trajectory);

	moveAction->perform(parent(), trajectory);



	//Set the angle to point towards the next nav point using the trajectory we calculated above
	//_rotateTowards(trajectory, parent());


}


void NavigationComponent::_applyAvoidanceMovement(b2Vec2& trajectory)
{

	const auto& physics = parent()->getComponent<PhysicsComponent>(ComponentTypes::PHYSICS_COMPONENT);

	//Check all sensor detected objects and see if any are close enough to prompt a movement adjustment
	for (const auto& seenObject : parent()->getSeenObjects()) {

		//If I see a barrier or a mobile object and it within X pixels, then apply an adjustment to the trajectory so that we dont
		//run into each other head on and get stuck
		if (seenObject.gameObject.expired() == false &&
			seenObject.gameObject.lock().get()->hasTrait(TraitTag::mobile)) {

			//Distance
			float distance = util::calculateDistance(parent()->getCenterPosition(), seenObject.gameObject.lock()->getCenterPosition());

			//Avoid for other mobile objects
			if (distance < 64) {

				//Compare the other objects id to determine who has the right of way
				std::string otherId = seenObject.gameObject.lock()->id();
				if (otherId > parent()->id()) {

					//Easy 90 degree applied to trajectory
					b2Vec2 avoidTrajectory90 = { trajectory.y, -trajectory.x };

					//Apply 45 degrees to  trajectory
					b2Vec2 avoidTrajectory45 = { (trajectory.x + trajectory.y) * sqrtf(2), (trajectory.y - trajectory.x) * sqrtf(2) };

					trajectory = avoidTrajectory90;
					break;

				}

			}

		}


	}

}



