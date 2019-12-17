#pragma once

#include <glm/vec3.hpp>

#include "Edge.h"
#include "Face.h"
#include "../RigidBody.h"



namespace PhysicEngine 
{

	Edge::Edge(const Face* f, int i0, int i1)
		: FaceParent(f), ids{ i0, i1 }
	{
		RigidBodyParent = f->Parent;
		endpoints[0] = RigidBodyParent->Vertices[ids[0]];
		endpoints[1] = RigidBodyParent->Vertices[ids[1]];
		direction = glm::normalize(endpoints[1] - endpoints[0]);
	}
	
	glm::vec3 Edge::V(int i) const
	{
		return endpoints[i];
	}

	glm::vec3 Edge::D() const
	{
		return direction;
	}

	glm::vec3 Edge::MV(int i) const
	{
		return glm::mat3(RigidBodyParent->ModelMatrix) * RigidBodyParent->Vertices[ids[i]];
	}

	glm::vec3 Edge::MD() const
	{
		return glm::mat3(RigidBodyParent->ModelMatrix) * direction;
	}

}