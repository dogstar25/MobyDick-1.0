#include "DefaultRotateAction.h"



#include "../GameObject.h"



DefaultRotateAction::DefaultRotateAction() :
	RotateAction(0)
{

}

DefaultRotateAction::DefaultRotateAction(float angularVelocity) :
	RotateAction(angularVelocity)
{

}

DefaultRotateAction::~DefaultRotateAction()
{


}

void DefaultRotateAction::perform(GameObject* gameObject)
{
	const auto& physicsComponent = gameObject->getComponent<PhysicsComponent>(ComponentTypes::PHYSICS_COMPONENT);

	physicsComponent->applyRotation(m_angularVelocity);

}