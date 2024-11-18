#pragma once
#include "Component.h"
#include "../GameObject.h"

#include <json/json.h>
#include <list>
#include <memory>
#include <mutex>


//inline constexpr int NAV_DISTANCE_TOLERANCE = 42;
inline constexpr int NAV_STUCK_TOLERANCE = 16;


struct AStarNode
{
	std::optional<std::shared_ptr<AStarNode>> connection{};
	SDL_Point position{};
	float gCost{};
	float hCost{};
	float fCost{};
	int connectionKeyX = -1;
	int connectionKeyY = -1;
};

//struct CompareNodes {
//	bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
//		if (a->fCost == b->fCost) {
//			return a->hCost > b->hCost; // Break ties with hCost
//		}
//		return a->fCost > b->fCost; // Lower fCost has higher priority
//	}
//};

//namespace std {
//	template <>
//	struct hash<std::tuple<int, int>> {
//		size_t operator()(const std::tuple<int, int>& key) const noexcept {
//			// Combine the hash of both elements
//			size_t h1 = std::hash<int>()(std::get<0>(key));
//			size_t h2 = std::hash<int>()(std::get<1>(key));
//			return h1 ^ (h2 << 1); // XOR and shift to combine hashes
//		}
//	};
//}

class NavigationComponent : public Component
{

public:
	NavigationComponent() = default;
	NavigationComponent(Json::Value componentJSON);
	~NavigationComponent();

	void update() override;
	void postInit() override;
	NavigationStatus navigateTo(float pixelX, float pixelY, int fuzzyFactor=0);
	void navigateStop();
	void disablePassageFitCheck() { m_applyPassageFit = false; }
	void enablePassageFitCheck() { m_applyPassageFit = true; }
	void setDestination(SDL_FPoint destination) { m_targetPixelDestination = destination; }
	SDL_FPoint getDestination() { return m_targetPixelDestination; }
	virtual void reset() {};

private:

	std::vector<SDL_Point>m_solutionPath{};

	bool _buildPathToDestination(int fuzzyFactor, std::vector<SDL_Point>& solutionPath);
	bool _buildPathToDestination_old(int fuzzyFactor, std::vector<SDL_Point>& solutionPath);

	void _buildNeighbors(AStarNode& currentNode, std::vector<std::shared_ptr<AStarNode>>& neighbors);
	bool _isValidNode(const int x, const int y);
	void _addNeighbor(int x, int y, std::vector<std::shared_ptr<AStarNode>>& neighbors);
	bool _listContainsNode(AStarNode* node, std::unordered_map<std::tuple<int, int>, std::shared_ptr<AStarNode>>& list);
	void _calculateCosts(AStarNode* startingNode, AStarNode* node);
	void _moveTo(SDL_Point destinationTile);
	void _applyAvoidanceMovement(b2Vec2& trajectory);
	void _applyAvoidanceMovement2(b2Vec2& trajectory);
	void _checkLocation(const int x, const int y);
	bool _applyNavObjectSizeCheck(int x, int y, int objectCategory);
	bool _isStuck();
	bool _foundDestinationNode(SDL_Point destinationTile, std::shared_ptr<AStarNode> currentNode, int fuzzyFactor=0);
	void _startPathfinding(int fuzzyFactor);

	Timer m_pathRefreshTimer{ .5, true };
	
	SDL_FPoint m_targetPixelDestination{};
	SDL_Point m_targetTileDestination{};
	int m_currentNavStep{};

	Timer m_stuckTimer{2, true};
	SDL_FPoint m_previousLocation{};
	int m_passageFitSizeCategory{};
	bool m_applyPassageFit{ true };
	float m_navigationDestinationTolerance{};
	float m_navigationStuckTolerance{};

	std::mutex m_pathMutex;
	std::thread m_pathfindingThread;
	bool m_pathInProgress{ false };
	bool m_newPathAvailable{ false };
	bool m_unableToFindPath{ false };
	std::vector<SDL_Point> m_newSolutionPath;





};

