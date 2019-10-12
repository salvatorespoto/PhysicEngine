#include "Functions.h"


namespace PhysicEngine
{

	bool TestRayBoundingBoxIntersection(glm::vec3 origin, glm::vec3 direction, BoundingBox boundingBox, glm::mat4 modelMatrix, float& outIntersectionDistance)
	{
		// Bounding box transform matrix
		glm::mat4 M = modelMatrix * boundingBox.M;

		// Bounding box center in woorld coordinates
		glm::vec3 bBoxCenter = (M * glm::vec4(boundingBox.center, 1.0f)).xyz();

		glm::vec3 d = bBoxCenter - origin;

		// Test intersection with the 2 planes perpendicular to the OBB's X axis
		float tMin = 0.0f;
		float tMax = 100000.0f;

		glm::vec3 bBoxXAxis = M[0].xyz();
		float e = glm::dot(bBoxXAxis, d);
		float f = glm::dot(direction, bBoxXAxis);

		if (fabs(f) > 0.001f)
		{
			// t1, t2: distance intersection for plane perendicular to bounding box X axis
			float t1 = (e - boundingBox.extension.x) / f;
			float t2 = (e + boundingBox.extension.x) / f;

			// Swap t1 and t2 to make t1 the nearest intersection
			if (t1 > t2) { float w = t1; t1 = t2; t2 = w; }

			// tMax is the nearest "far" intersection (amongst the X,Y and Z planes pairs)
			if (t2 < tMax) tMax = t2;
			// tMin is the farthest "near" intersection (amongst the X,Y and Z planes pairs)
			if (t1 > tMin) tMin = t1;

			// If "far" is closer than "near", then there is NO intersection.
			if (tMax < tMin) return false;
		}
		else
		{	// The ray is almost parallel to the planes and they don't "intersection"
			if (-e - boundingBox.extension.x > 0.0f || -e + boundingBox.extension.x < 0.0f) return false;
		}


		// Test intersection with the 2 planes perpendicular to the OBB's Y axis
		glm::vec3 bBoxYAxis = M[1].xyz();
		e = glm::dot(bBoxYAxis, d);
		f = glm::dot(direction, bBoxYAxis);

		if (fabs(f) > 0.001f) {

			float t1 = (e - boundingBox.extension.y) / f;
			float t2 = (e + boundingBox.extension.y) / f;

			if (t1 > t2) { float w = t1; t1 = t2; t2 = w; }
			if (t2 < tMax) tMax = t2;
			if (t1 > tMin) tMin = t1;
			if (tMin > tMax) return false;
		}
		else {
			if (-e - boundingBox.extension.y > 0.0f || -e + boundingBox.extension.y < 0.0f) return false;
		}


		// Test intersection with the 2 planes perpendicular to the OBB's Z axis
		glm::vec3 bBoxZAxis = M[2].xyz();
		e = glm::dot(bBoxZAxis, d);
		f = glm::dot(direction, bBoxZAxis);

		if (fabs(f) > 0.001f)
		{

			float t1 = (e - boundingBox.extension.z) / f;
			float t2 = (e + boundingBox.extension.z) / f;

			if (t1 > t2) { float w = t1; t1 = t2; t2 = w; }

			if (t2 < tMax) tMax = t2;
			if (t1 > tMin) tMin = t1;
			if (tMin > tMax) return false;

		}
		else
		{
			if (-e - boundingBox.extension.z > 0.0f || -e + boundingBox.extension.z < 0.0f) return false;
		}

		outIntersectionDistance = tMin;
		return true;
	}


	bool TestPolyHedronIntersect(const PhysicEngine::ConvexPolyhedron& c0, const PhysicEngine::ConvexPolyhedron& c1)
	{
		// This algorithm is based on the separate axis theorem.
		// For three dimensional convex polyhedra the axes to be test are the ones parallel to face normals and the ones 
		// parallels to the cross product betwenn two objects face normals.

		double min0, max0, min1, max1;
		glm::mat3 M0 = glm::mat3(*c0.ModelMatrix);
		glm::mat3 M1 = glm::mat3(*c1.ModelMatrix);

		// Test direction parallel to faces normal of c0
		for (const PhysicEngine::Face f : c0.Faces)
		{
			ComputePolyhedronProjectionOnAxis(c0, M0 * f.n, min0, max0);
			ComputePolyhedronProjectionOnAxis(c1, M0 * f.n, min1, max1);
			if (max0 < min1 || max1 < min0) return false;
		}

		// Test direction parallel to faces normal of c1
		for (const PhysicEngine::Face f : c1.Faces)
		{
			ComputePolyhedronProjectionOnAxis(c0, M1 * f.n, min0, max0);
			ComputePolyhedronProjectionOnAxis(c1, M1 * f.n, min1, max1);
			if (max0 < min1 || max1 < min0) return false;
		}

		// Test direction paralles to the cross product of c0 and c1 face normals
		for (const PhysicEngine::Face f0 : c0.Faces) 
		{
			for (const PhysicEngine::Face f1 : c1.Faces) 
			{
				glm::vec3&& axis = glm::cross(M0 * f0.n, M1 * f1.n);
				ComputePolyhedronProjectionOnAxis(c0, axis, min0, max0);
				ComputePolyhedronProjectionOnAxis(c1, axis, min1, max1);
				if (max0 < min1 || max1 < min0) return false;
			}
		}

		return true;
	}


	void ComputePolyhedronProjectionOnAxis(const PhysicEngine::ConvexPolyhedron& c0, const glm::vec3& axis, double& outMin, double& outMax)
	{
		glm::vec3&& d = glm::normalize(axis);

		glm::vec3 Mv = (*c0.ModelMatrix * glm::vec4(c0.Vertices[0], 1.0f)).xyz();
		outMin = outMax = glm::dot(Mv, d);
		for (const glm::vec3& v : c0.Vertices)
		{
			Mv = (*c0.ModelMatrix * glm::vec4(v , 1.0f)).xyz();
			double p = glm::dot(Mv, d);
			outMin = (p < outMin) ? p : outMin;
			outMax = (p > outMax) ? p : outMax;
		}
	}

}