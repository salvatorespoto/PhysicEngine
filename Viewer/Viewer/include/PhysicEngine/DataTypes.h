#pragma once

#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>


namespace PhysicEngine
{

	/** A 3d bounding box. Faces are quads */
	typedef struct BoundingBox {
		glm::vec3 center;			// Untrasformed center
		glm::vec3 extension;		// Half a side 
		glm::mat4 M;				// Transformation matrix
		glm::vec3 vertices[8];		// Trasformed vertices of the bounding box 
		unsigned int edges[24];		// Index list of the edges of the bounding box
	} BoundingBox;

}


