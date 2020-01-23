#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include "PhysicEngine/geometry/Face.h"
#include "PhysicEngine/physics/RigidBody.h"


namespace PhysicEngine 
{

	Face::Face() {}

	Face::Face(const RigidBody* parent, std::vector<int> verticesIds)
		: Parent(parent), Size(verticesIds.size()), VIds(verticesIds)
	{

		// Build edges, also edges will be in COUTERCLOCKWISE order
		for (size_t i = 0; i < VIds.size(); i++)
		{
			Edges.push_back(Edge(this, VIds[i], VIds[(i + 1) % VIds.size()]));
		}

		// Compute normal assuming COUNTERCLOCKWISE vertices order
		Normal = glm::normalize(glm::cross(Edges[0].D(), Edges[1].D()));
	}

	void Face::SetRigidBody(RigidBody* rb)
	{
		Parent = rb;
	}

	size_t Face::size() const
	{
		return VIds.size();
	}

	int Face::I(int i) const
	{
		return VIds[i];
	}
		
	glm::vec3 Face::V(int i) const
	{
		return Parent->Vertices[VIds[i]].V();
	}
		
	glm::vec3 Face::MV(int i) const
	{
		return Parent->Vertices[VIds[i]].MV();
	}

	Edge Face::E(int i) const
	{
		return Edges[i];
	}

	glm::vec3 Face::N() const
	{
		return Normal;
	}

	glm::vec3 Face::MN() const
	{
		return glm::mat3(Parent->ModelMatrix) * Normal;
	}

}