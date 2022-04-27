#include "SurvivorInteractionAction.h"
#include "../components/SurvivorBrainComponent.h"
#include <iostream>
#include "game.h"
#include "../GameConstants.h"

extern std::unique_ptr<Game> game;

void SurvivorInteractionAction::perform(GameObject* interactingObject, GameObject* interactionObject, SDL_Scancode keyScanCode)
{
	const auto& brainComponent = interactionObject->getComponent<SurvivorBrainComponent>(ComponentTypes::SURVIVOR_BRAIN_COMPONENT);

	brainComponent->followMe(interactingObject);

	std::cout << "do brain action " << keyScanCode << std::endl;

}