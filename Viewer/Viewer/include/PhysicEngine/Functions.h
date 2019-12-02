#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_SWIZZLE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/epsilon.hpp>

#include "DataTypes.h"
#include "RigidBody.h"
#include "Collision.h"


namespace PhysicEngine
{

	const float EPSILON_ZERO = 0.00001f;

	/**
	 * Compares two floating values
	 *
	 * @param f0 
	 * @param f1
	 */
	bool Equal(float f0, float f1);

	bool GreaterThan(float f0, float f1);

	bool LessThan(float f0, float f1);

	/**
	 * Test intersection between a ray and an oriented bounding box 
	 *
	 * @param The ray origin
	 * @param The ray direction
	 * @param The oriented bounding box
	 * @param The oriented bounding box model trasfromation matrix
	 */
	bool TestRayBoundingBoxIntersection(glm::vec3 origin, glm::vec3 direction, BoundingBox boundingBox, glm::mat4 modelMatrix, float& outIntersectionDistance);

	/**
	 * Test intersection between a two convex polyhedron
	 *
	 * @param p0 the first polyhedron
	 * @param p1 the second polyhedron
	 */
	bool TestRigidBodyIntersect(const PhysicEngine::RigidBody& p0, const PhysicEngine::RigidBody& p1, std::vector<Contact>& outContacts);

	/** 
	 * Compute the interval of a convex polyhedron projection onto a specifi axis
	 * This function could be used to test if two polyhedron intersect along an axis in the following way:
	 *		ComputePolyhedronProjectionOnAxis(polyhedron0, axis, min0, max0);
	 *		ComputePolyhedronProjectionOnAxis(polyhedron0, axis, min1, max1);
	 *		if (max0 < min1 || max1 < min0) return false; // the two polyhedron do not intersect on the axis
	 *
	 * @param c0 the convex polyhedron
	 * @param d the direction onto compute the projection
	 * @param outMin the output minimum value projection
	 * @param outMax the output minimum value projection
	 */
	void ComputeRigidBodyProjectionOnAxis(const PhysicEngine::RigidBody& c0, const glm::vec3& axis, double& outMin, double& outMax);


	/**
	 * Compute the interval of a convex polyhedron projection onto a specifi axis
	 *
	 * @param c0 the convex polyhedron
	 * @param d the direction onto compute the projection
	 * @param 
	 * @param 
	 */
	void ComputeRigidBodyProjectionOnAxis(const PhysicEngine::RigidBody& c0, const glm::vec3& axis, PhysicEngine::ProjectionInfo& outProjectionInfo);


	/**
	 * Compute the interval of a convex polyhedron projection onto a specifi axis
	 *
	 * @param c0 the convex polyhedron
	 * @param d the direction onto compute the projection
	 * @param
	 * @param
	 */
	bool NoIntersect(double tMax, const float& speed, ProjectionInfo& projectionInfo0, ProjectionInfo& projectionInfo1,
		ProjectionInfo& pCurr0, ProjectionInfo& pCurr1, int& side, double& tFirst, double& tLast);


	/**
	 * Compute the intersection set of two polyhedra that are in contact
	 *
	 * @param c0 the first convex polyhedron
	 * @param c0 the first convex polyhedron* @param
	 */
	void GetIntersection(const RigidBody& c0, const RigidBody& c1,
		ProjectionInfo& pInfo0, ProjectionInfo& pInfo1, int side, double tFirst, std::vector<Contact>& outContacts);
	

	/**
	 * 
	 */
	std::vector<Contact> GetEdgeEdgeIntersection(RigidBody rb0, RigidBody rb1, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& q0, const glm::vec3& q1);


	/**
	 *
	 */
	std::vector<Contact> GetEdgeFacesIntersection(RigidBody rb0, RigidBody rb1, const glm::vec3 p0, const glm::vec3 p1, std::vector<Object3D> edges);


	/**
	 *
	 */
	std::vector<Contact> GetCoplanarFaceFaceIntersection(RigidBody rb0, RigidBody rb1, std::vector<Object3D> f0, std::vector<Object3D> f1);
}

#endif
