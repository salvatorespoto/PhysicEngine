#pragma once

#include <glm/vec3.hpp>


namespace PhysicEngine
{

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
		Edge(const RigidBody *parent, int i0, int i1);

		/**
		 * Return the edge indexes
		 *
		 * @return The edge indexes. Edges are assumed to be in COUNTERCLOCKWISE order.
		 */
		int I(int i) const
		{
			return index[i];
		};

		/**
		 * Return the edge end points
		 *
		 * @return The edge direction. Edges are assumed to be in COUNTERCLOCKWISE order, so it return Edge.P[1] - Edge.P[0]
		 */
		glm::vec3 P(int i) const;

		/**
		 * Get the edge direction
		 *
		 * @return The edge direction. Edges are assumed to be in COUNTERCLOCKWISE order, so it return Edge.P[1] - Edge.P[0]
		 */
		glm::vec3 V() const;

		inline bool operator==(const Edge& e) const
		{
			return index[0] == e.index[0] && index[1] == e.index[1];
		};


	private:

		const RigidBody *Parent;	/**< The RigidBody objects that owns this edge */
		int index[2] = { 0, 0 };	/**< The two edge indexes in the parent RigidBody vertex list */
	};

}



namespace std {

	/** 
	 * Define the hash function for the Edge object to use std containers
	 */
	template<>
	struct hash<PhysicEngine::Edge> {

		inline size_t operator()(const PhysicEngine::Edge& e) const {
			return std::hash<int>()(e.I(0)) ^ std::hash<int>()(e.I(1));
		}
	};

}