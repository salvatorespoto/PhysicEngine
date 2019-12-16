#pragma once

#include <glm/vec3.hpp>

#include "Edge.h"
#include "RigidBody.h"



namespace PhysicEngine 
{

	Edge::Edge(const RigidBody *parent, int i0, int i1) : Parent(parent), index{ i0, i1 } {}
	

	glm::vec3 Edge::P(int i) const
	{
		return Parent->Vertices[index[i]];
	};


	glm::vec3 Edge::V() const
	{
		return Parent->Vertices[index[1]] - Parent->Vertices[index[0]];
	};

}