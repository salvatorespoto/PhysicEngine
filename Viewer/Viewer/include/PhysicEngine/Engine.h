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
			for (RigidBody* c : Objects) c->IsColliding = false;

			for	(RigidBody* c0 : Objects)
			{
				for (RigidBody* c1 : Objects)
				{
					if (c0 == c1) break;

					if (TestPolyHedronIntersect(*c0, *c1, Contacts)) 
					{
						c0->IsColliding = true;
						c1->IsColliding = true;
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
