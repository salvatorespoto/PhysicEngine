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
	private:
		std::vector<ConvexPolyhedron*> Objects;

	public:
		
		void Tick()
		{
			TestCollisions();
		}

		void AddConvexPolyhedron(ConvexPolyhedron* c)
		{
			Objects.push_back(c);
		}

		void TestCollisions() 
		{
			for(ConvexPolyhedron* c : Objects) 
				c->IsColliding = false;
			
			for	(ConvexPolyhedron* c0 : Objects) 
			{
				for (ConvexPolyhedron* c1 : Objects)
				{
					if (c0 == c1) break;
					if (TestPolyHedronIntersect(*c0, *c1)) 
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
