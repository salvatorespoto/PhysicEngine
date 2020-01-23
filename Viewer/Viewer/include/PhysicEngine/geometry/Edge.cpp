#pragma once

#define GLM_FORCE_SWIZZLE 
#include <glm/glm.hpp>
#include <glm/vec3.hpp>

#include "PhysicEngine/geometry/Edge.h"
#include "PhysicEngine/geometry/Face.h"
#include "PhysicEngine/physics/RigidBody.h"



namespace PhysicEngine 
{

	Edge::Edge() {}

	Edge::Edge(const Face* f, int i0, int i1)
		: FaceParent(f), ids{ i0, i1 }
	{
		RigidBodyParent = f->Parent;
		endpoints[0] = RigidBodyParent->Vertices[ids[0]].V();
		endpoints[1] = RigidBodyParent->Vertices[ids[1]].V();
		direction = glm::normalize(endpoints[1] - endpoints[0]);
	}

	void Edge::SetParentRigidBody(RigidBody* rb) 
	{
		RigidBodyParent = rb;
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
		return RigidBodyParent->Vertices[ids[i]].MV().xyz();
	}

	glm::vec3 Edge::MD() const
	{
		return glm::mat3(RigidBodyParent->ModelMatrix) * direction;
	}

}