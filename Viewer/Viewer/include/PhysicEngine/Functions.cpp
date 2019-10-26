#include "Functions.h"


namespace PhysicEngine
{

	bool Equal(float f0, float f1) 
	{
		return fabs(f1 - f0) < EPSILON_ZERO ? true : false;
	}

	bool GreaterThan(float f0, float f1)
	{
		return (f0 - f1) > EPSILON_ZERO ? true : false;
	}

	bool LessThan(float f0, float f1)
	{
		return (f0 - f1) < -EPSILON_ZERO ? true : false;
	}




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
		{	// The ray is almost parallel to the planes and they don't "intersect"
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

		glm::vec3&& v = c1.Velocity - c0.Velocity;	// Relative speed between c0 and c1
		double tFirst = 0.0f;
		double tLast = 99999999.0f;
		int side = 0;
		int tMax = 10;

		PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1, pCurr0, pCurr1;
		// Test direction parallel to faces normal of c0
		for (const PhysicEngine::Face f : c0.Faces)
		{
			PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1;
			ComputePolyhedronProjectionOnAxis(c0, M0 * f.n, projectionInfo0);
			ComputePolyhedronProjectionOnAxis(c1, M0 * f.n, projectionInfo1);
			float speed = glm::dot(f.n, v);
			if(NoIntersect(tMax, speed, projectionInfo0, projectionInfo1, pCurr0, pCurr1, side, tFirst, tLast))
				return false;
		}

		// Test direction parallel to faces normal of c1
		for (const PhysicEngine::Face f : c1.Faces)
		{
			PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1;
			ComputePolyhedronProjectionOnAxis(c0, M1 * f.n, projectionInfo0);
			ComputePolyhedronProjectionOnAxis(c1, M1 * f.n, projectionInfo1);
			float speed = glm::dot(f.n, v);
			if (NoIntersect(tMax, speed, projectionInfo0, projectionInfo1, pCurr0, pCurr1, side, tFirst, tLast)) 
				return false;	
		}

		// Test direction paralles to the cross product of c0 and c1 face normals
		for (const PhysicEngine::Face f0 : c0.Faces) 
		{
			for (const PhysicEngine::Face f1 : c1.Faces) 
			{
				PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1;
				glm::vec3&& axis = glm::cross(M0 * f0.n, M1 * f1.n);
				if(axis == glm::vec3(0, 0, 0)) break;
				ComputePolyhedronProjectionOnAxis(c0, axis, projectionInfo0);
				ComputePolyhedronProjectionOnAxis(c1, axis, projectionInfo1);
				float speed = glm::dot(axis, v);
				if (NoIntersect(tMax, speed, projectionInfo0, projectionInfo1, pCurr0, pCurr1, side, tFirst, tLast))
					return false;	
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

	void ComputePolyhedronProjectionOnAxis(const PhysicEngine::ConvexPolyhedron& c0, const glm::vec3& axis, PhysicEngine::ProjectionInfo& outProjectionInfo)
	{
		glm::vec3&& d = glm::normalize(axis);

		outProjectionInfo.min = FLT_MAX;
		outProjectionInfo.max = -FLT_MAX;

		for (const Face& f : c0.Faces)
		{
			// Transform the three vertices according to the model matrix
			glm::vec3 Mv0 = (*c0.ModelMatrix * glm::vec4(c0.Vertices[f.vId[0]], 1.0f)).xyz();
			glm::vec3 Mv1 = (*c0.ModelMatrix * glm::vec4(c0.Vertices[f.vId[1]], 1.0f)).xyz();
			glm::vec3 Mv2 = (*c0.ModelMatrix * glm::vec4(c0.Vertices[f.vId[2]], 1.0f)).xyz();

			// Project the face vertices onto the direction d
			double t0, t1, t2;
			t0 = glm::dot(Mv0, d);
			t1 = glm::dot(Mv1, d);
			t2 = glm::dot(Mv2, d);

		
			// UPDATE MAXIMUM 

			// If the face is perpendicular the the direction d, the whole face is a candidate to be a contact set
			if (Equal(t0, t1) && Equal(t1, t2))
			{
				if (GreaterThan(t0, outProjectionInfo.max))
				{
					outProjectionInfo.max = t0;
					outProjectionInfo.sMax.type = ContactSet::FACE;
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.faces.push_back(f);
				}
				else if (Equal(t0, outProjectionInfo.max))
				{
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.type = ContactSet::FACE;
					outProjectionInfo.sMax.faces.push_back(f);
				}

				if (LessThan(t0, outProjectionInfo.min))
				{
					outProjectionInfo.min = t0;
					outProjectionInfo.sMin.type = ContactSet::FACE;
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.faces.push_back(f);
				}
				else if (Equal(t0, outProjectionInfo.min))
				{
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.type = ContactSet::FACE;
					outProjectionInfo.sMin.faces.push_back(f);
				}
			}

			// An edge is perpendicular to the direction d and is the candidate to be a contact set
			else if (Equal(t0, t1))
			{
				if (GreaterThan(t0, outProjectionInfo.max))
				{
					outProjectionInfo.max = t0;
					outProjectionInfo.sMax.type = ContactSet::EDGE;
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.edges.push_back(glm::ivec2(f.vId[0], f.vId[1]));
				}
			
				if (LessThan(t0, outProjectionInfo.min))
				{
					outProjectionInfo.min = t0;
					outProjectionInfo.sMin.type = ContactSet::EDGE;
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.edges.push_back(glm::ivec2(f.vId[0], f.vId[1]));
				}
			}

			// An edge is perpendicular to the direction d and is the candidate to be a contact set
			else if (Equal(t1, t2))
			{
				if (GreaterThan(t1, outProjectionInfo.max))
				{
					outProjectionInfo.max = t1;
					outProjectionInfo.sMax.type = ContactSet::EDGE;
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.edges.push_back(glm::ivec2(f.vId[1], f.vId[2]));
				}

				if (LessThan(t1, outProjectionInfo.min))
				{
					outProjectionInfo.min = t1;
					outProjectionInfo.sMin.type = ContactSet::EDGE;
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.edges.push_back(glm::ivec2(f.vId[1], f.vId[2]));
				}
			}

			// An edge is perpendicular to the direction d and is the candidate to be a contact set
			else if (Equal(t2, t0))
			{
				if (GreaterThan(t2, outProjectionInfo.max))
				{
					outProjectionInfo.max = t2;
					outProjectionInfo.sMax.type = ContactSet::EDGE;
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.edges.push_back(glm::ivec2(f.vId[2], f.vId[0]));
				}
				
				if (LessThan(t2, outProjectionInfo.min))
				{
					outProjectionInfo.min = t2;
					outProjectionInfo.sMin.type = ContactSet::EDGE;
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.edges.push_back(glm::ivec2(f.vId[2], f.vId[0]));
				}
			}

			// Otherwise only a vertex could be a candidate for the contact set
			if (!(Equal(t0, t1) && Equal(t1, t2)))
			{
				if (GreaterThan(t0, outProjectionInfo.max))
				{
					outProjectionInfo.max = t0;
					outProjectionInfo.sMax.type = ContactSet::VERTEX;
					outProjectionInfo.sMax.vertex = f.vId[0];
				}

				if (GreaterThan(t1, outProjectionInfo.max))
				{
					outProjectionInfo.max = t1;
					outProjectionInfo.sMax.type = ContactSet::VERTEX;
					outProjectionInfo.sMax.vertex = f.vId[1];
				}

				if (GreaterThan(t2, outProjectionInfo.max))
				{
					outProjectionInfo.max = t1;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertex = f.vId[2];
				}
				
				if (LessThan(t0, outProjectionInfo.min))
				{
					outProjectionInfo.min = t0;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertex = f.vId[0];
				}

				if (LessThan(t1, outProjectionInfo.min))
				{
					outProjectionInfo.min = t1;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertex = f.vId[1];
				}

				if (LessThan(t2, outProjectionInfo.min))
				{
					outProjectionInfo.min = t1;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertex = f.vId[2];
				}

			}

		}
	}


	bool NoIntersect(double tMax, const float& speed, ProjectionInfo& projectionInfo0, ProjectionInfo& projectionInfo1,
		ProjectionInfo& pCurr0, ProjectionInfo& pCurr1, int& side, double& tFirst, double& tLast)
	{
		tFirst = 0;
		tLast = 99999999.0f;

		// c1 is on the "left" of c0 on the direction d		
		if (projectionInfo1.max < projectionInfo0.min)
		{
			if (speed <= 0)  return true;	// intervals are moving apart -> no intersection

			double t = (projectionInfo0.min - projectionInfo1.max) / speed; // time of the first contact
			if (t > tFirst) 
			{ 
				tFirst = t;
				side = -1;
				pCurr0 = projectionInfo0;
				pCurr1 = projectionInfo1;
			}
			if (t > tMax) return true;	// no intersection until tMax

			t = (projectionInfo0.max - projectionInfo1.min) / speed;	// time of the last contact, when c1 surpass c0
			if (t < tLast) tLast = t;
			if (tFirst > tLast) return true;	// No intersection
		}

		// c1 is on the "right" of c0 on the direction d
		else if (projectionInfo0.max < projectionInfo1.min)
		{
			if (speed >= 0)  return true;	// intervals are moving apart -> no intersection

			double t = (projectionInfo0.max - projectionInfo1.min) / speed; // time of the first contact
			if (t > tFirst) 
			{
				tFirst = t;
				side = +1;
				pCurr0 = projectionInfo0;
				pCurr1 = projectionInfo1;
			}
			
			if (t > tMax) return true;	// no intersection until tMax

			t = (projectionInfo0.min - projectionInfo1.max) / speed;	// time of the last contact, when c1 surpass c0
			if (t < tLast) tLast = t;
			if (tFirst > tLast) return true;	// No intersection
		}

		// Intervals are already overlapping 
		else
		{
			if (speed > 0)
			{
				double t = (projectionInfo0.max - projectionInfo1.min) / speed; // time of the first contact
				if (t < tLast) tLast = t;
				if (tFirst > tLast) return true;	// No intersection
			}
			if (speed < 0)
			{
				double t = (projectionInfo0.min - projectionInfo1.max) / speed;	// time of the last contact, when c1 surpass c0
				if (t < tLast) tLast = t;
				if (tFirst > tLast) return true;	// No intersection
			}
		}

		return false;
	}
}