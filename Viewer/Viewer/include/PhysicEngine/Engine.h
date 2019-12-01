#ifndef ENGINE_H
#define ENGINE_H

#include <vector>

#include "DataTypes.h"
#include "Functions.h"
#include "RigidBody.h"

/** The phisic engine */
namespace PhysicEngine
{
	class Engine
	{

		private: 

		/**
		 * List of Rigid Body objects in the simulation
		 */
		std::vector<RigidBody*> Objects;

		/**
		 * Current contacts among objects
		 */
		std::vector<Contact> Contacts;

		/**
		 * Current simulation time
		 */
		float currentTime;

		/**
		 * Delta time between two engine updates 
		 */
		float timeStep;


	public:

		void Tick()
		{
			DetectCollisions();
			Update();
		}

		void SetStartingTime(float t) 
		{
			currentTime = t;
		}
		
		void SetDeltaTime(float dt) 
		{
			timeStep = dt;
		}


		void AddRigidBody(RigidBody* c)
		{
			Objects.push_back(c);
		}


		void DetectCollisions()
		{
			Contacts.clear();
			for (RigidBody* rb : Objects) rb->IsColliding = false;

			for	(RigidBody* rbA : Objects)
			{
				for (RigidBody* rbB : Objects)
				{
					if (rbA == rbB) break;

					if (TestRigidBodyIntersect(*rbA, *rbB, Contacts))
					{
						rbA->IsColliding = true;
						rbB->IsColliding = true;
					};
				}
			}
		}


		void Update() 
		{
			for (RigidBody* b : Objects)
			{
				b->UpdateState(currentTime, timeStep);
			}
			
			currentTime += timeStep;
		}

		/**
		 * Return all contacts among objects
		 */
		const std::vector<Contact>& GetContacts()
		{
			return Contacts;
		}
	};
};
#endif
