#pragma once

#include <set>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>


namespace PhysicEngine 
{
	class RigidBody;

	class PolyhedronBSPTree
	{

	public:

		struct Arc						/**< An arc in the spherical polyheron dual */
		{
			int i;
			int j;
		};

		struct SphericalPoly			/**< A spherical face in in the spherical polyheron dual */
		{
			std::set<int> v;
			int vId;
		};

		/** 
		 * A node in the Polyhedron BSP tree
		 */
		class PolyhedronBSPNode {
		
		public:
			
			PolyhedronBSPNode() {};

			Arc A;
			SphericalPoly S;

			PolyhedronBSPNode* left;	/**< Left child */
			PolyhedronBSPNode* right;	/**< Right child */
			bool isLeaf;				/**< A tree leaf */
		};

		/**
		 * Build a BSP tree for a convex rigidbod object object
		 *
		 * @param rb The RigidBody
		 */
		PolyhedronBSPTree(const RigidBody& rb);

		/**
		 * Builds the BSP tree recursively
		 */
		PolyhedronBSPNode* BuildTree(std::vector<Arc> arcs, std::vector<SphericalPoly> polygons);

	private:
		const RigidBody& polyhedron;
	};

}