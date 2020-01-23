#include <glm/vec4.hpp>

#include "PhysicEngine/geometry/Vertex.h"
#include "PhysicEngine/physics/RigidBody.h"

namespace PhysicEngine 
{

	const glm::vec4 Vertex::MV() const
	{
		return parent->ModelMatrix * coordinates;
	}

}