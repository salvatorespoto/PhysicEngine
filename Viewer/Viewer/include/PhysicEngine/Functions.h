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
	bool TestRayBoundingBoxIntersection(glm::vec3 origin, glm::vec3 direction, BoundingBox boundingBox, glm::mat4 modelMatrix);
}
#endif
