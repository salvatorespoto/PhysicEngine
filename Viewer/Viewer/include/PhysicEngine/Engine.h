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
	public:
		
		std::vector<RigidBody*> Objects;
		std::vector<Contact> Contacts;

		void Tick()
		{
			DetectCollisions();
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
	};
};
#endif
