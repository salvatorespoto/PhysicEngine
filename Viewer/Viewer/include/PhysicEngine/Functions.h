#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_SWIZZLE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include "DataTypes.h"

namespace PhysicEngine
{

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
	bool TestPolyHedronIntersect(const PhysicEngine::ConvexPolyhedron& p0, const PhysicEngine::ConvexPolyhedron& p1);

	/** 
	 * Compute the interval of a convex polyhedron projection onto a specifi axis
	 *
	 * @param c0 the convex polyhedron
	 * @param d the direction onto compute the projection
	 * @param outMin the output minimum value projection
	 * @param outMax the output minimum value projection
	 */
	void ComputePolyhedronProjectionOnAxis(const PhysicEngine::ConvexPolyhedron& c0, const glm::vec3& axis, double& outMin, double& outMax);

}
#endif
