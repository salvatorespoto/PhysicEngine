#pragma once

#include "PhysicEngine/geometry/Object3D.h"
#include "PhysicEngine/geometry/Vertex.h"
#include "PhysicEngine/geometry/Edge.h"
#include "PhysicEngine/geometry/Face.h"
#include "PhysicEngine/physics/RigidBody.h"


namespace PhysicEngine 
{

	/**
	 * The projection of a Rigid Body on an given axis
	 *
	 * This class stores the projection of a Rigid Body over an axis.
	 * It stores the info about the minimum and maximum extremes.
	 */
	class AxisProjection
	{
		
		public:

			class Extreme
			{
				public:

					enum class Type { UNASSIGNED = 0, VERTEX, EDGE, FACE };	/**< Extreme type definition */
					Extreme::Type type = Extreme::Type::UNASSIGNED;			/**< Extreme type */
					float value = 0.0f;										/**< Projection value */
					Vertex v;
					Edge e;
					Face f;
					std::vector<Vertex> vertices;							/**< The list of vertices of the extreme */

					Extreme() {}
			};
	
			RigidBody* rigidBody;		/**< Reference to the projected rigid body */
			Extreme Min;	/**< Minimimum extreme projection info */
			Extreme Max;	/**< Maximum extreme projection info */

			/**
			 * Constructor
			 */
			AxisProjection() : rigidBody(NULL) {}
			
			/**
			 * Constructor
			 *
			 * @param vertex The extreme is a vertex
			 * @parame p The extreme value
			 */
			AxisProjection(RigidBody* rb) : rigidBody(rb) {}
			
			/**
			 * Constructor
			 *
			 * @param vertex The extreme is a vertex
			 * @parame p The extreme value
			 */
			AxisProjection(RigidBody* rb, Extreme min, Extreme max) : rigidBody(rb), Min(min), Max(max) {}

			/**
			 * Get the parent RigidBody
			 *
			 * @return The RigidBody of this projection
			 */
			const RigidBody* GetRigidBody() 
			{
				return rigidBody;
			}
	};


	/**
	 * Compute the interval of a convex polyhedron projection onto a specifi axis
	 *
	 * @param c0 the convex polyhedron
	 * @param d the direction onto compute the projection
	 * @param
	 * @param
	 */
	AxisProjection ComputeRigidBodyProjectionOnAxis(RigidBody& rb, glm::vec3 axis);

}