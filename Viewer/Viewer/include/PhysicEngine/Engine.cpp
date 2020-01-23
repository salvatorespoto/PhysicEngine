#include "PhysicEngine/collision/collision.h"
#include "PhysicEngine/Engine.h"


namespace PhysicEngine
{

	void RigidBodyEngine::Tick()
	{
		DetectCollisions();
		//ProcessCollisions();
		Update();
	}


	void RigidBodyEngine::SetStartingTime(float t)
	{
		currentTime = t;
	}


	void RigidBodyEngine::SetTimeStep(float dt)
	{
		timeStep = dt;
	}


	void RigidBodyEngine::AddRigidBody(RigidBody* rb)
	{
		Objects.push_back(rb);
	}


	const std::vector<RigidBody*>& RigidBodyEngine::GetRigidBodies()
	{
		return Objects;
	}


	const std::vector<Contact>& RigidBodyEngine::GetContacts()
	{
		return Contacts;
	}


	void RigidBodyEngine::DetectCollisions()
	{
		// Reset collisions
		Contacts.clear();
		for (RigidBody* rb : Objects)
		{
			rb->IsColliding = false;
		}

		// Detect collisions and compute contact sets
		for (RigidBody* rbA : Objects)
		{
			for (RigidBody* rbB : Objects)
			{
				if (rbA == rbB) break;
				std::vector<Contact> cs = ComputeRigidBodyContact(currentTime, timeStep, *rbA, *rbB);
				if (!cs.empty())
				{
					rbA->IsColliding = true;
					rbB->IsColliding = true;
					for (Contact c : cs) Contacts.push_back(c);
				}
			}
		}
	}


	void RigidBodyEngine::ProcessCollisions()
	{
		if (Contacts.size() > 0)
		{
			DoCollisionResponse(currentTime, timeStep, Contacts, Objects);
		}
	}


	void RigidBodyEngine::Update()
	{
		for (RigidBody* b : Objects)
		{
			b->UpdateState(currentTime, timeStep);
		}
		currentTime += timeStep;
	}
}