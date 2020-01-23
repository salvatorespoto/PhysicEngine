#pragma once

#include <glm/vec4.hpp>

#include "PhysicEngine/geometry/Object3D.h"


namespace PhysicEngine
{

	class RigidBody;

	/**
	 * A 3D vertex
	 */
	class Vertex : public Object3D
	{

	public:

		/** 
		 * Default constructor
		 */
		Vertex() {}

		/**
		 * Construct of Object3D
		 *
		 * Object3D is an abstract class. Every geometric 3D object used in the engine must derive from it.
		 *
		 * @param

		 */
		Vertex(RigidBody *rb, glm::vec4 c) : parent(rb), coordinates(c) {};

		/** 
		 * Destructor
		 */
		~Vertex() {}

		/**
		 * Comparison operator
		 *
		 * @param v The vertex that will be compared with this
		 * @return True if the two vertices have the same coordinates
		 */
		bool operator==(const Vertex& v) const
		{
			return v.V() == coordinates;
		};

		/**
		 * Return the vertex location in object coordinates
		 *
		 * @return The vertex location in the object reference frame
		 */
		const glm::vec4 V() const
		{
			return coordinates;
		};

		/**
		 * Return the vertex location in world coordinates
		 *
		 * Return the vertex coordinates trasformed by the parent RigidBody model matrix
		 *
		 * @return The tranformed vertex coordinates
		 */
		const glm::vec4 MV() const;


	private:

		glm::vec4 coordinates;	/**< Vertex coordinates in the RigidBody local reference frame */
		const RigidBody* parent;		/**< The parent RigidBody */
	};

}