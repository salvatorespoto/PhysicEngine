#ifndef ENGINE_H
#define ENGINE_H

#include <vector>

#include "DataTypes.h"
#include "Functions.h"


/** The phisic engine */
namespace PhysicEngine
{
	class Engine
	{
	public:
		
		std::vector<ConvexPolyhedron*> Objects;
		std::vector<Contact> Contacts;

		void Tick()
		{
			DetectCollisions();
		}

		void AddConvexPolyhedron(ConvexPolyhedron* c)
		{
			Objects.push_back(c);
		}

		void DetectCollisions()
		{
			Contacts.clear();
			for (ConvexPolyhedron* c : Objects) c->IsColliding = false;

			for	(ConvexPolyhedron* c0 : Objects) 
			{
				for (ConvexPolyhedron* c1 : Objects)
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
