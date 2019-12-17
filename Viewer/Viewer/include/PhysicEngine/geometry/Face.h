#pragma once

#include <vector>
#include <glm/vec3.hpp>


namespace PhysicEngine 
{

	class RigidBody;
	class Edge;

	
	/** 
	 * A face of a Rigid Body
	 */
	class Face 
	{

	public:
		
		/** 
		 * Costruct a new face.
		 *
		 * @param parent A pointer to the parent RigidBody
		 */
		Face(const RigidBody* parent, std::vector<int> verticesIds);

		/**
		 * Return the number of vertices in the face
		 *
		 * @return The number of vertices in the face
		 */
		size_t size() const;

		/**
		 * Return ith index of the face
		 *
		 * @return The face normal
		 */
		int I(int i) const;

		/**
		 * Return ith vertex of the face.
		 * Vertices are assumed to be in COUTERCLOCKWISE order
		 *
		 * @return The ith vertex
		 */
		glm::vec3 V(int i) const;

		/**
		 * Return ith vertex transformed with the current parent model matrix.
		 * Vertices are assumed to be in COUTERCLOCKWISE order
		 *
		 * @return The ith vertex
		 */
		glm::vec3 MV(int i) const;

		/**
		 * Return ith edge.
		 * Edges are assumed to be in COUTERCLOCKWISE order
		 *
		 * @return The ith edge
		 */
		Edge E(int i) const;

		/** 
		 * Return the normal to the face
		 * 
		 * @return The face normal
		 */
		glm::vec3 N() const;

		/**
		 * Return the trasformed face normal
		 *
		 * @return The face normal multipliead by the current parent model matrix
		 */
		glm::vec3 MN() const;
	
		size_t Size;
		glm::vec3 Normal;
		std::vector<int> VIds;
		std::vector<Edge> Edges;
		const RigidBody* Parent;
		
	};

}