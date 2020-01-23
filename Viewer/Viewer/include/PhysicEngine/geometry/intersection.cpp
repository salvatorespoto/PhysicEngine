#include<vector>

#define GLM_FORCE_SWIZZLE 
#include <glm/glm.hpp>

#include "PhysicEngine/math/comparison.h"
#include "PhysicEngine/geometry/intersection.h"
#include "PhysicEngine/geometry/Vertex.h"
#include "PhysicEngine/geometry/Edge.h"
#include "PhysicEngine/geometry/Face.h"


namespace PhysicEngine 
{

	std::vector<Contact> ComputeRigidBodyIntersection(AxisProjection& projectionA, AxisProjection& projectionB, int side, float tFirst)
	{
		std::vector<Contact> contactSet;
		RigidBody& A = *projectionA.rigidBody;
		RigidBody& B = *projectionB.rigidBody;

		// Maximum extreme of RigidBody A meet minimum extreme of RigidBody B
		if (side == 1)
		{
			// [VERTEX A - FACE B] contact
			if (projectionA.Max.type == AxisProjection::Extreme::Type::VERTEX &&
				projectionB.Min.type == AxisProjection::Extreme::Type::FACE)
			{
				Vertex v = projectionA.Max.v;
				Face f = projectionB.Min.f;
				
				contactSet.push_back(
					Contact
					{
						Contact::Type::VERTEX_FACE,
						A,
						B,
						v.V().xyz() + (A.LinearVelocity * tFirst), // The predicted position of the vertex
						f.MN() // The face normal
					}
				);
			}
			
			// [VERTEX B - FACE A] contact
			else if (projectionB.Min.type == AxisProjection::Extreme::Type::VERTEX &&
					 projectionA.Max.type == AxisProjection::Extreme::Type::FACE)
			{
				Vertex v = projectionB.Min.v;
				Face f = projectionA.Max.f;

				contactSet.push_back(
					Contact
					{
						Contact::Type::VERTEX_FACE,
						B,
						A,
						v.V().xyz() + (B.LinearVelocity * tFirst), // The predicted position of the vertex
						f.MN() // The face normal
					}
				);
			}

			else if (projectionA.Max.type == AxisProjection::Extreme::Type::EDGE)
			{
				Edge eA = projectionA.Max.e;

				// [EDGE A - EDGE B] contact
				if (projectionB.Min.type == AxisProjection::Extreme::Type::EDGE)
				{
					Edge eB = projectionB.Min.e;

					glm::vec3 intersection; 
					if (ComputeCrossSegmentSegmentIntersection(
						eA.MV(0) + (A.LinearVelocity * tFirst),
						eA.MV(1) + (A.LinearVelocity * tFirst),
						eB.MV(0) + (B.LinearVelocity * tFirst),
						eB.MV(1) + (B.LinearVelocity * tFirst),
						intersection))
					{
						// Select the normal as the one pointing outside the second object
						glm::vec3 n = glm::normalize(glm::cross(eA.MD(), eA.MD()));
						float d = glm::dot(n, glm::normalize(intersection - B.Position));
						n = (d > 0) ? n : -1.0f * n;

						contactSet.push_back( 
							Contact 
							{
								Contact::Type::EDGE_EDGE,
								A,
								B,
								intersection, // Intersection point
								n // Normal
							});
					}					
				}

				// [EDGE A - FACE  B] contact
				else if (projectionB.Min.type == AxisProjection::Extreme::Type::FACE)
				{
					Face f = projectionB.Min.f;
					std::vector<std::vector<glm::vec3>> edgesB(f.Edges.size());
					for (int i = 0; i< f.Edges.size(); i++)
					{
						edgesB[i].resize(2);
						edgesB[i][0] = f.Edges[i].MV(0) + (B.LinearVelocity * (float)tFirst);
						edgesB[i][1] = f.Edges[i].MV(1) + (B.LinearVelocity * (float)tFirst);
					}

					std::vector<glm::vec3> intersections = ComputeEdgeFaceIntersection(
						eA.MV(0) + (A.LinearVelocity * tFirst),
						eA.MV(1) + (A.LinearVelocity * tFirst),
						edgesB);

					for (glm::vec3 p : intersections) 
					{
						contactSet.push_back(
							Contact
							{
								Contact::Type::EDGE_EDGE,
								A,
								B,
								p, // Intersection point
								f.MN() // Normal
							});
					}
				}
			}

			else if (projectionA.Max.type == AxisProjection::Extreme::Type::FACE)
			{
				Face f = projectionA.Max.f;
				std::vector<std::vector<glm::vec3>> edgesA(f.Edges.size());
				for (int i = 0; i < f.Edges.size(); i++)
				{
					edgesA[i].resize(2);
					edgesA[i][0] = f.Edges[i].MV(0) + (A.LinearVelocity * (float)tFirst);
					edgesA[i][1] = f.Edges[i].MV(1) + (A.LinearVelocity * (float)tFirst);
				}

				// [FACE A - EDGE B] contact
				if ((projectionB.Min.type == AxisProjection::Extreme::Type::EDGE))
				{
					Edge eB = projectionB.Min.e;
					std::vector<glm::vec3> intersections = ComputeEdgeFaceIntersection(
						eB.MV(0) + (B.LinearVelocity * tFirst),
						eB.MV(1) + (B.LinearVelocity * tFirst),
						edgesA);

					for (glm::vec3 p : intersections)
					{
						contactSet.push_back(
							Contact {
								Contact::Type::EDGE_EDGE,
								A,
								B,
								p, // Intersection point
								f.MN() // Normal
							});
					}
				}

				// [FACE A - FACE B] contact
				else if ((projectionB.Min.type == AxisProjection::Extreme::Type::FACE))
				{
					Face f = projectionB.Min.f;
					std::vector<std::vector<glm::vec3>> edgesB(f.Edges.size());
					for (int i = 0; i < f.Edges.size(); i++)
					{
						edgesB[i].resize(2);
						edgesB[i][0] = f.Edges[i].MV(0) + (B.LinearVelocity * (float)tFirst);
						edgesB[i][1] = f.Edges[i].MV(1) + (B.LinearVelocity * (float)tFirst);
					}

					std::vector<glm::vec3> intersections = ComputeCoplanarFaceFaceIntersection(edgesA, edgesB);

					for (glm::vec3 p : intersections)
					{
						contactSet.push_back(
							Contact{
								Contact::Type::VERTEX_FACE,
								B,
								A,
								p, // Intersection point
								f.MN() // Normal
							});
					}
				}
			}
		}

		// Maximum extreme of RigidBody B meet minimum extreme of RigidBody A
		if (side == -1)
		{
			// [VERTEX B - FACE A] contact
			if (projectionB.Max.type == AxisProjection::Extreme::Type::VERTEX &&
				projectionA.Min.type == AxisProjection::Extreme::Type::FACE)
			{
				Vertex v = projectionB.Max.v;
				Face f = projectionA.Min.f;

				contactSet.push_back(
					Contact
					{
						Contact::Type::VERTEX_FACE,
						B,
						A,
						v.V().xyz() + (B.LinearVelocity * tFirst), // The predicted position of the vertex
						f.MN() // The face normal
					}
				);
			}

			// [VERTEX A - FACE B] contact
			else if (projectionA.Min.type == AxisProjection::Extreme::Type::VERTEX &&
				projectionB.Max.type == AxisProjection::Extreme::Type::FACE)
			{
				Vertex v = projectionA.Min.v;
				Face f = projectionB.Max.f;

				contactSet.push_back(
					Contact
					{
						Contact::Type::VERTEX_FACE,
						A,
						B,
						v.V().xyz() + (A.LinearVelocity * tFirst), // The predicted position of the vertex
						f.MN() // The face normal
					}
				);
			}

			else if (projectionB.Max.type == AxisProjection::Extreme::Type::EDGE)
			{
				Edge eB = projectionB.Max.e;

				// [EDGE B - EDGE A] contact
				if (projectionA.Min.type == AxisProjection::Extreme::Type::EDGE)
				{
					Edge eA = projectionA.Min.e;

					glm::vec3 intersection;
					if (ComputeCrossSegmentSegmentIntersection(
						eB.MV(0) + (B.LinearVelocity * tFirst),
						eB.MV(1) + (B.LinearVelocity * tFirst),
						eA.MV(0) + (A.LinearVelocity * tFirst),
						eA.MV(1) + (A.LinearVelocity * tFirst),
						intersection))
					{
						// Select the normal as the one pointing outside the second object
						glm::vec3 n = glm::normalize(glm::cross(eB.MD(), eB.MD()));
						float d = glm::dot(n, glm::normalize(intersection - A.Position));
						n = (d > 0) ? n : -1.0f * n;

						contactSet.push_back(
							Contact
							{
								Contact::Type::EDGE_EDGE,
								B,
								A,
								intersection, // Intersection point
								n // Normal
							});
					}
				}

				// [EDGE A - FACE  B] contact
				else if (projectionA.Min.type == AxisProjection::Extreme::Type::FACE)
				{
					Face f = projectionA.Min.f;
					std::vector<std::vector<glm::vec3>> edgesA(f.Edges.size());
					for (int i = 0; i < f.Edges.size(); i++)
					{
						edgesA[i].resize(2);
						edgesA[i][0] = f.Edges[i].MV(0) + (A.LinearVelocity * (float)tFirst);
						edgesA[i][1] = f.Edges[i].MV(1) + (A.LinearVelocity * (float)tFirst);
					}

					std::vector<glm::vec3> intersections = ComputeEdgeFaceIntersection(
						eB.MV(0) + (A.LinearVelocity * tFirst),
						eB.MV(1) + (A.LinearVelocity * tFirst),
						edgesA);

					for (glm::vec3 p : intersections)
					{
						contactSet.push_back(
							Contact
							{
								Contact::Type::EDGE_EDGE,
								B,
								A,
								p, // Intersection point
								f.MN() // Normal
							});
					}
				}
			}

			else if (projectionB.Max.type == AxisProjection::Extreme::Type::FACE)
			{
				Face f = projectionB.Max.f;
				std::vector<std::vector<glm::vec3>> edgesB(f.Edges.size());
				for (int i = 0; i < f.Edges.size(); i++)
				{
					edgesB[i].resize(2);
					edgesB[i][0] = f.Edges[i].MV(0) + (B.LinearVelocity * (float)tFirst);
					edgesB[i][1] = f.Edges[i].MV(1) + (B.LinearVelocity * (float)tFirst);
				}

				// [FACE B - EDGE A] contact
				if ((projectionA.Min.type == AxisProjection::Extreme::Type::EDGE))
				{
					Edge eA = projectionA.Min.e;
					std::vector<glm::vec3> intersections = ComputeEdgeFaceIntersection(
						eA.MV(0) + (B.LinearVelocity * tFirst),
						eA.MV(1) + (B.LinearVelocity * tFirst),
						edgesB);

					for (glm::vec3 p : intersections)
					{
						contactSet.push_back(
							Contact{
								Contact::Type::EDGE_EDGE,
								B,
								A,
								p, // Intersection point
								f.MN() // Normal
							});
					}
				}

				// [FACE B - FACE A] contact
				else if ((projectionA.Min.type == AxisProjection::Extreme::Type::FACE))
				{
					Face f = projectionA.Min.f;
					std::vector<std::vector<glm::vec3>> edgesA(f.Edges.size());
					for (int i = 0; i < f.Edges.size(); i++)
					{
						edgesA[i].resize(2);
						edgesA[i][0] = f.Edges[i].MV(0) + (A.LinearVelocity * (float)tFirst);
						edgesA[i][1] = f.Edges[i].MV(1) + (A.LinearVelocity * (float)tFirst);
					}

					std::vector<glm::vec3> intersections = ComputeCoplanarFaceFaceIntersection(edgesB, edgesA);

					for (glm::vec3 p : intersections)
					{
						contactSet.push_back(
							Contact{
								Contact::Type::VERTEX_FACE,
								A,
								B,
								p, // Intersection point
								f.MN() // Normal
							});
					}
				}
			}
		}

		return contactSet;
	}


	bool ComputeCrossSegmentSegmentIntersection(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& q0, const glm::vec3& q1, glm::vec3& outIntersection)
	{
		glm::vec3 p = p1 - p0;
		glm::vec3 q = q1 - q0;
		glm::vec3 r = q0 - p0;

		glm::vec3 e = glm::cross(p, r);

		// The two edges lay on the same line, in the engine we discard this type of intersection
		if (EpsZero(e)) return false;
		
		glm::vec3 t = glm::cross(q, r);
		glm::vec3 s = glm::cross(q, p);

		// Check if the two lines are parallel, but do not lay on the same line
		if (EpsZero(s))
		{
			// The two segments do not overlap
			return false;
		}

		// Compute the distance between the two lines
		if (EpsEqual(fabs(glm::dot(r, glm::normalize(s))), 0))
		{
			// Distance is 0, the two lines are incident, compute the intersection point
			// that is p0 (+-) (cross(p,r)/cross(p,q)) * q
			//outIntersection.type = Object3D::VERTEX;
			glm::vec3 i;
			if (EpsZero(t))
			{
				// p0 is already the intersection point
				i = p0;
			}
			else
			{
				float sign = (EpsEqual(glm::normalize(t), glm::normalize(s))) ? 1.0f : -1.0f;
				i = p0 + sign * ((glm::length(t) / glm::length(s)) * p);
			}

			// Check if the intersection is contained in both the edges
			glm::vec3 pD = glm::normalize(p);
			glm::vec3 qD = glm::normalize(q);

			float pI = glm::dot(i, glm::normalize(pD));
			float qI = glm::dot(i, glm::normalize(qD));

			float lp0 = glm::dot(p0, glm::normalize(pD));
			float lp1 = glm::dot(p1, glm::normalize(pD));
			float lq0 = glm::dot(q0, glm::normalize(qD));
			float lq1 = glm::dot(q1, glm::normalize(qD));

			if (lp0 > lp1) std::swap(lp0, lp1);
			if (lq0 > lq1) std::swap(lq0, lq1);

			if ((lp0 <= pI && pI <= lp1) && (lq0 <= qI && qI <= lq1))
			{
				//outIntersection.type = Object3D::VERTEX;
				glm::vec3 edgeA[2] = { p0, p1 };
				glm::vec3 edgeB[2] = { q0, q1 };

				outIntersection = i;
				return true;
			}
		}

		// No cross segment-segment intersection		
		return false;
	}


	std::vector<glm::vec3> ComputeEdgeFaceIntersection(const glm::vec3 p0, const glm::vec3 p1, std::vector<std::vector<glm::vec3>> edges)
	{
		std::vector<glm::vec3> intersections;
		glm::vec3 insidePoint;

		// Compute the face normal
		glm::vec3 faceNormal;
		if (edges[0][0] == edges[1][1])
			faceNormal = glm::normalize(glm::cross(edges[1][1] - edges[1][0], edges[0][1] - edges[0][0]));
		else
			faceNormal = glm::normalize(glm::cross(edges[0][1] - edges[0][0], edges[1][1] - edges[1][0]));

		for (std::vector<glm::vec3> e : edges)
		{
			// I already know that the intersection could be one point, or none
			glm::vec3 intersection;
			if (ComputeCrossSegmentSegmentIntersection(p0, p1, e[0], e[1], intersection))
			{
				intersections.push_back(intersection);
				glm::vec3 edgeNormal = glm::normalize(glm::cross(e[1] - e[0], faceNormal));
				insidePoint = (glm::dot(p1 - p0, edgeNormal) < 0) ? p1 : p0;
			}
		}

		// The whole edge is inside or outside the face
		if (intersections.empty())
		{
			bool inside = true;

			// Check if the segment is outside the polygon
			for (std::vector<glm::vec3> e : edges)
			{
				glm::vec3 edgeNormal = glm::normalize(glm::cross(e[1] - e[0], faceNormal));

				// if one point is in front of one the face -> the point is outside -> the whole segment is outside
				inside = ((glm::dot(p1 - (e[1] + e[0]) / 2.0f, edgeNormal) < 0) ? true : false) && inside; 
			}
			
			if (inside) {
				intersections.push_back(p0);
				intersections.push_back(p1);
			}
		}

		// Only one intersection -> one point	is in the face
		if (intersections.size() == 1)
		{
			intersections.push_back(insidePoint);
		}

		return intersections;
	}


	std::vector<glm::vec3> ComputeCoplanarFaceFaceIntersection(std::vector<std::vector<glm::vec3>> edgesA, std::vector<std::vector<glm::vec3>> edgesB)
	{
		std::vector<glm::vec3> intersections;

		for (std::vector<glm::vec3> eA : edgesA)
		{
			for (glm::vec3 i : ComputeEdgeFaceIntersection(eA[0], eA[1], edgesB))
			{
				if (std::find(intersections.begin(), intersections.end(), i) == intersections.end()) 
					intersections.push_back(i);
			}
		}

		// Check what points of face 1 are inside face0
		bool insideE00 = true;
		bool insideE01 = true;

		// Compute the face normal
		glm::vec3 face0Normal;
		if (edgesA[0][0] == edgesA[1][1])
			face0Normal = glm::normalize(glm::cross(edgesA[1][1] - edgesA[1][0], edgesA[0][1] - edgesA[0][0]));
		else
			face0Normal = glm::normalize(glm::cross(edgesA[0][1] - edgesA[0][0], edgesA[1][1] - edgesA[1][0]));

		for (std::vector<glm::vec3> eB : edgesB)
		{
			insideE00 = true;
			insideE01 = true;
			for (std::vector<glm::vec3> eA : edgesA)
			{
				glm::vec3 edgeNormal = glm::cross(eA[1] - eA[0], face0Normal);
				insideE00 = ((glm::dot(eB[0] - ((eA[0] + eA[1]) / 2.0f), edgeNormal) <= 0) ? true : false) && insideE00;
				insideE01 = ((glm::dot(eB[1] - ((eA[0] + eA[1]) / 2.0f), edgeNormal) <= 0) ? true : false) && insideE01;
			}

			if (insideE00) {
				if (std::find(intersections.begin(), intersections.end(), eB[0]) == intersections.end()) intersections.push_back(eB[0]);
			}

			if (insideE01) {
				if (std::find(intersections.begin(), intersections.end(), eB[0]) == intersections.end()) intersections.push_back(eB[1]);
			}
		}

		return intersections;
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

}