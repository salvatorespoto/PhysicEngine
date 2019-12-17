#pragma once

#include <glm/vec3.hpp>


namespace PhysicEngine
{

	class Face;
	class RigidBody;
	
	/**
	 * A 3D object edge
	 *
	 * This class stores the an edge indexes in a RigidBody and must holds a reference to
	 * the parent RigidBody. The edges are assumed to be in COUNTERCLOCKWISE order, so the
	 * Edge direction is Edge.P[1] - Edge.P[0]
	 */
	class Edge
	{

	public:

		/**
		 * Construct a new Edge
		 *
		 * @param parent The RigidBody object that owns this edge
		 * @param i0 the first index of the edge in the parent RigidBody vertex list
		 * @param i1 the second index of the edge in the parent RigidBody vertex list
		 */
		Edge(const Face *parent, int i0, int i1);

		/**
		 * Return the edge indexes
		 *
		 * @return The edge indexes. Edges are assumed to be in COUNTERCLOCKWISE order.
		 */
		int I(int i) const
		{
			return ids[i];
		};

		/**
		 * Return the edge end points
		 *
		 * @return The edge end point. Edges are assumed to be in COUNTERCLOCKWISE order, so it return Edge.P[1] - Edge.P[0]
		 */
		glm::vec3 V(int i) const;

		/**
		 * Get the edge direction
		 *
		 * @return The edge direction. Edges are assumed to be in COUNTERCLOCKWISE order, so it return Edge.P[1] - Edge.P[0]
		 */
		glm::vec3 D() const;

		/**
		 * Get the current trasformed endpoint
		 *
		 * Get the current endpoint position in world coordinates. I.e. the Parent ù
		 * Model matrix * the edge endpoint in object coordinate.
		 *
		 * @return The edge trasformed direction.
		 */
		glm::vec3 MV(int i) const;

		/**
		 * Get the current trasformed edge direction
		 *
		 * Get the current edge direction in world coordinates. I.e. the Parent ù
		 * Model matrix * the edge endpoint in object coordinate.
		 *
		 * @return The edge trasformed direction.
		 */
		glm::vec3 MD() const;

		inline bool operator==(const Edge& e) const
		{
			return ids[0] == e.ids[0] && ids[1] == e.ids[1];
		};


	private:

		const RigidBody *RigidBodyParent;	/**< The RigidBody objects that owns this edge */
		const Face* FaceParent;				/**< The Face objects that owns this edge */
		int ids[2];							/**< The two edge indexes in the parent RigidBody vertex list */
		glm::vec3 endpoints[2];				/**< End points coordinates relative to the parent Center of Mass */
		glm::vec3 direction;				/**< Edge direction */
	};

}



namespace std {

	/** 
	 * Define the hash function for the Edge type.
	 */
	template<>
	struct hash<PhysicEngine::Edge> {

		inline size_t operator()(const PhysicEngine::Edge& e) const {
			return std::hash<int>()(e.I(0)) ^ std::hash<int>()(e.I(1));
		}
	};

}