#include <limits>

#define GLM_SWIZZLE
#include <glm/glm.hpp>

#include "PhysicEngine/geometry/projection.h"
#include "PhysicEngine/physics/RigidBody.h"
#include "PhysicEngine/math/comparison.h"


namespace PhysicEngine 
{

	AxisProjection ComputeRigidBodyProjectionOnAxis(RigidBody& rb, const glm::vec3 axis)
	{
		AxisProjection projection(&rb);
		glm::vec3 d = glm::normalize(axis);

		projection.Min.value = std::numeric_limits<float>::max();
		projection.Max.value = std::numeric_limits<float>::min();

		// Compute the maximum and minimum extremals in the axis direction
		for (Vertex v : rb.Vertices) 
		{
			float p = glm::dot(v.MV().xyz(), axis);
			
			if(EpsEqual(p, projection.Max.value))
			{
				if(projection.Max.type == AxisProjection::Extreme::Type::VERTEX)
				{
					projection.Max.type = AxisProjection::Extreme::Type::EDGE;
					projection.Max.vertices.push_back(v);
				}

				if (projection.Max.type == AxisProjection::Extreme::Type::EDGE)
				{
					projection.Max.type = AxisProjection::Extreme::Type::FACE;
					projection.Max.vertices.push_back(v);
					projection.Max.f = rb.GetFace(projection.Max.vertices);
				}
			}

			if (p > projection.Max.value)
			{
				projection.Max.value = p;
				projection.Max.type == AxisProjection::Extreme::Type::VERTEX;
				projection.Max.vertices.push_back(v);
			}
				
			if (p < projection.Min.value)
			{
				projection.Min.value = p;
				projection.Min.type == AxisProjection::Extreme::Type::VERTEX;
				projection.Min.vertices.push_back(v);
			}
		}

		if (projection.Max.type == AxisProjection::Extreme::Type::EDGE) projection.Max.e = rb.GetEdge(projection.Max.vertices);
		if (projection.Max.type == AxisProjection::Extreme::Type::FACE) projection.Max.f = rb.GetFace(projection.Max.vertices);
		if (projection.Min.type == AxisProjection::Extreme::Type::EDGE) projection.Min.e = rb.GetEdge(projection.Min.vertices);
		if (projection.Min.type == AxisProjection::Extreme::Type::FACE) projection.Min.f = rb.GetFace(projection.Min.vertices);

		return projection;
	}

}