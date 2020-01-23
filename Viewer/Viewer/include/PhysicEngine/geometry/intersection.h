#pragma once

#include "PhysicEngine/geometry/projection.h"
#include "PhysicEngine/physics/RigidBody.h"


namespace PhysicEngine 
{

	/**
	 * A contact between two RigidBody
	 */
	class Contact
	{

	public:

		enum class Type { VERTEX_FACE = 0, EDGE_EDGE };			/**< Possible contacts types. If the type is VERTEX_FACE, there is the assumption
																	that the vertex is from RigidBody A and the face is from the RigidBody B. */

		Type type = Type::VERTEX_FACE;							/**< The type of this contact */
		RigidBody& rb0;									/**< First RigidBody involved in contact */
		RigidBody& rb1;									/**< Second RigidBody involved in contact */
		glm::vec3 point = glm::vec3(0);							/**< The contact point between the two RigidBody */
		glm::vec3 normal = glm::vec3(0);						/**< If it is a VERTEX_FACE contact, holds the normal to the face */
		glm::vec3 edgeA[2] = { glm::vec3(0), glm::vec3(0) };	/**< If it is an EDGE_EDGE contact the edge from the RigidBody A, stores the two edge vertices */
		glm::vec3 edgeB[2] = { glm::vec3(0), glm::vec3(0) };	/**< If it is an EDGE_EDGE contact the edge from the RigidBody A, stores the two edge vertices */

		/**
		 * Constructor
		 */
		Contact(Contact::Type type, RigidBody& rb0, RigidBody& rb1, glm::vec3 point, glm::vec3 normal)
			: type(type), rb0(rb0), rb1(rb1), point(point), normal(normal) {};

		/**
		 * Constructor
		 */
		Contact(Contact::Type type, RigidBody& rb0, RigidBody& rb1, glm::vec3 point, glm::vec3 normal, glm::vec3 edgeA[2], glm::vec3 edgeB[2])
			: type(type), rb0(rb0), rb1(rb1), point(point), normal(normal), edgeA{ edgeA[0], edgeA[1] }, edgeB{ edgeB[0], edgeB[1] } {}

		/**
		 * Overload the == operator
		 */
		bool operator ==(Contact c)
		{
			return point == c.point;
		}

		/**
		 * Overload the != operator
		 */
		bool operator !=(Contact c)
		{
			return point != c.point;
		};
	};

	/**
	 * Compute the intersection set of two polyhedra that are in contact
	 */
	std::vector<Contact> ComputeRigidBodyIntersection(
		AxisProjection& projectionA, 
		AxisProjection& projectionB, 
		int side, 
		float tFirst
	);

	/**
	 * Compute the intersection between two 3D segments
	 */
	bool ComputeCrossSegmentSegmentIntersection(
		const glm::vec3& p0,
		const glm::vec3& p1,
		const glm::vec3& q0,
		const glm::vec3& q1,
		glm::vec3& outIntersection
	);
	
	/**
	 * Compute the intersection between a segment and a face
	 */
	std::vector<glm::vec3> ComputeEdgeFaceIntersection(
		const glm::vec3 p0,
		const glm::vec3 p1,
		std::vector<std::vector<glm::vec3>> edges
	);

	/**
	 * Compute the intersection between two coplanar 3D faces 
	 */
	std::vector<glm::vec3> ComputeCoplanarFaceFaceIntersection(
		std::vector<std::vector<glm::vec3>> edgesA,
		std::vector<std::vector<glm::vec3>> edgesB
	);

	/**
	 * Test intersection between a ray and an oriented bounding box
	 *
	 * @param The ray origin
	 * @param The ray direction
	 * @param The oriented bounding box
	 * @param The oriented bounding box model trasfromation matrix
	 */
	bool TestRayBoundingBoxIntersection(
		glm::vec3 origin, 
		glm::vec3 direction, 
		BoundingBox boundingBox, 
		glm::mat4 modelMatrix, 
		float& outIntersectionDistance
	);
}