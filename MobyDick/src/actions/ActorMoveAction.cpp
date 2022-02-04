#include "ActorMoveAction.h"


#include "../GameObject.h"


ActorMoveAction::ActorMoveAction() :
	MoveAction(0, 0)
{

}

void ActorMoveAction::perform(GameObject* gameObject)
{
	const auto& physicsComponent = gameObject->getComponent<PhysicsComponent>(ComponentTypes::PHYSICS_COMPONENT);
	const auto& animationComponent = gameObject->getComponent<AnimationComponent>(ComponentTypes::ANIMATION_COMPONENT);
	const auto& vitalityComponent = gameObject->getComponent<VitalityComponent>(ComponentTypes::VITALITY_COMPONENT);

	physicsComponent->applyMovement(vitalityComponent->speed(), m_direction, m_strafe);


	if (animationComponent)
	{

		if (m_direction != 0 || m_strafe != 0)
		{
			animationComponent->animate(ANIMATION_RUN, ANIMATE_ONE_TIME);
		}

	}

}