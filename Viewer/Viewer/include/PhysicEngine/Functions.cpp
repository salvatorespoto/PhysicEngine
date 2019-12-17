#include <map>

#include "Functions.h"
#include "math/comparison.h"

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


	std::vector<Contact> TestRigidBodyIntersect(const float currentTime, const float timeStep, PhysicEngine::RigidBody& c0, PhysicEngine::RigidBody& c1)
	{
		// This algorithm is based on the separate axis theorem.
		// For three dimensional convex polyhedra the axes to be test are the ones parallel to face normals and the ones 
		// parallels to the cross product betwenn two objects face normals.

		std::vector<Contact> outContacts;

		glm::mat3 M0 = glm::mat3(c0.ModelMatrix);
		glm::mat3 M1 = glm::mat3(c1.ModelMatrix);

		glm::vec3 v = c1.LinearVelocity - c0.LinearVelocity;	// Relative speed between c0 and c1
		float tFirst = 0.0f;
		float tLast = 99999999.0f;
		int side = 0;
		float tMax = 5;// timeStep;

		PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1, pCurr0, pCurr1;
		// Test direction parallel to faces normal of c0
		for (const PhysicEngine::Face f : c0.Faces)
		{
			PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1;
			ComputeRigidBodyProjectionOnAxis(c0, M0 * f.N(), projectionInfo0);
			ComputeRigidBodyProjectionOnAxis(c1, M0 * f.N(), projectionInfo1);
			float speed = glm::dot(v, glm::normalize(M0 * f.N()));
			if(NoIntersect(tMax, speed, projectionInfo0, projectionInfo1, pCurr0, pCurr1, side, tFirst, tLast))
				return outContacts;
		}

		// Test direction parallel to faces normal of c1
		for (const PhysicEngine::Face f : c1.Faces)
		{
			PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1;
			ComputeRigidBodyProjectionOnAxis(c0, M1 * f.N(), projectionInfo0);
			ComputeRigidBodyProjectionOnAxis(c1, M1 * f.N(), projectionInfo1);
			float speed = glm::dot(v, glm::normalize(M1 * f.N()));
			if (NoIntersect(tMax, speed, projectionInfo0, projectionInfo1, pCurr0, pCurr1, side, tFirst, tLast)) 
				return outContacts;
		}

		
		// Test direction paralles to the cross product of two edges
		for(const glm::vec3 e0 : c0.EdgesDirs)
		{			
			for (const glm::vec3 e1 : c1.EdgesDirs)
			{
				PhysicEngine::ProjectionInfo projectionInfo0, projectionInfo1;
				glm::vec3&& axis = glm::normalize(glm::cross(e0, e1));
				if (axis == glm::vec3(0, 0, 0)) break;
				ComputeRigidBodyProjectionOnAxis(c0, axis, projectionInfo0);
				ComputeRigidBodyProjectionOnAxis(c1, axis, projectionInfo1);
				float speed = glm::dot(v, glm::normalize(axis));
				if (NoIntersect(tMax, speed, projectionInfo0, projectionInfo1, pCurr0, pCurr1, side, tFirst, tLast))
					return outContacts;
			}
		}
		
		// The two polyhedron intersect: compute the intersection set
		Object3D contactSet;
		GetIntersection(c0, c1, pCurr0, pCurr1, side, tFirst, outContacts);

		return outContacts;
	}


	void ComputeRigidBodyProjectionOnAxis(const PhysicEngine::RigidBody& c0, const glm::vec3& axis, float& outMin, float& outMax)
	{
		glm::vec3&& d = glm::normalize(axis);

		glm::vec3 Mv = (c0.ModelMatrix * glm::vec4(c0.Vertices[0], 1.0f)).xyz();
		outMin = outMax = glm::dot(Mv, d);
		for (const glm::vec3& v : c0.Vertices)
		{
			Mv = (c0.ModelMatrix * glm::vec4(v , 1.0f)).xyz();
			float p = glm::dot(Mv, d);
			outMin = (p < outMin) ? p : outMin;
			outMax = (p > outMax) ? p : outMax;
		}
	}

	void ComputeRigidBodyProjectionOnAxis(const PhysicEngine::RigidBody& c0, const glm::vec3& axis, PhysicEngine::ProjectionInfo& outProjectionInfo)
	{
		glm::vec3&& d = glm::normalize(axis);

		outProjectionInfo.min = FLT_MAX;
		outProjectionInfo.max = -FLT_MAX;

		for (const Face& f : c0.Faces)
		{
			// Transform the three vertices according to the model matrix
			glm::vec3 Mv0 = (c0.ModelMatrix * glm::vec4(c0.Vertices[f.I(0)], 1.0f)).xyz();
			glm::vec3 Mv1 = (c0.ModelMatrix * glm::vec4(c0.Vertices[f.I(1)], 1.0f)).xyz();
			glm::vec3 Mv2 = (c0.ModelMatrix * glm::vec4(c0.Vertices[f.I(2)], 1.0f)).xyz();

			// Project the face vertices onto the direction d
			float t0, t1, t2;
			t0 = glm::dot(Mv0, d);
			t1 = glm::dot(Mv1, d);
			t2 = glm::dot(Mv2, d);

		
			// UPDATE MAXIMUM 

			// If the face is perpendicular the the direction d, the whole face is a candidate to be a contact set
			if (EpsEqual(t0, t1) && EpsEqual(t1, t2))
			{
				if (EpsGreaterThan(t0, outProjectionInfo.max))
				{
					outProjectionInfo.max = t0;
					outProjectionInfo.sMax.type = ContactSet::FACE;
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.faces.push_back(f);
				}
				else if (EpsEqual(t0, outProjectionInfo.max))
				{
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.type = ContactSet::FACE;
					outProjectionInfo.sMax.faces.push_back(f);
				}

				if (EpsLessThan(t0, outProjectionInfo.min))
				{
					outProjectionInfo.min = t0;
					outProjectionInfo.sMin.type = ContactSet::FACE;
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.faces.push_back(f);
				}
				else if (EpsEqual(t0, outProjectionInfo.min))
				{
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.type = ContactSet::FACE;
					outProjectionInfo.sMin.faces.push_back(f);
				}
			}

			// An edge is perpendicular to the direction d and is the candidate to be a contact set
			else if (EpsEqual(t0, t1))
			{
				if (EpsGreaterThan(t0, outProjectionInfo.max) || (EpsEqual(t0, outProjectionInfo.max) && outProjectionInfo.sMax.type != ContactSet::FACE))
				{
					outProjectionInfo.max = t0;
					outProjectionInfo.sMax.type = ContactSet::EDGE;
					outProjectionInfo.sMax.vertices.clear();
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.edges.push_back(Edge(&f, (int) f.I(0), (int)f.I(1)));
				}
			
				if (EpsLessThan(t0, outProjectionInfo.min) || (EpsEqual(t0, outProjectionInfo.min) && outProjectionInfo.sMin.type != ContactSet::FACE))
				{
					outProjectionInfo.min = t0;
					outProjectionInfo.sMin.type = ContactSet::EDGE;
					outProjectionInfo.sMin.vertices.clear();
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.edges.push_back(Edge(&f, (int) f.I(0), (int)f.I(1)));
				}
			}

			// An edge is perpendicular to the direction d and is the candidate to be a contact set
			else if (EpsEqual(t1, t2))
			{
				// It's also equal becouse it is possible that we have already computed a vertex of this edge as projection max
				if (EpsGreaterThan(t1, outProjectionInfo.max) || (EpsEqual(t1, outProjectionInfo.max) && outProjectionInfo.sMax.type != ContactSet::FACE))
				{
					outProjectionInfo.max = t1;
					outProjectionInfo.sMax.type = ContactSet::EDGE;
					outProjectionInfo.sMax.vertices.clear();
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.edges.push_back(Edge(&f, (int)f.I(1), (int)f.I(2)));
				}

				if (EpsLessThan(t1, outProjectionInfo.min) || (EpsEqual(t1, outProjectionInfo.min) && outProjectionInfo.sMin.type != ContactSet::FACE))
				{
					outProjectionInfo.min = t1;
					outProjectionInfo.sMin.type = ContactSet::EDGE;
					outProjectionInfo.sMin.vertices.clear();
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.edges.push_back(Edge(&f, (int)f.I(1), (int)f.I(2)));
				}
			}

			// An edge is perpendicular to the direction d and is the candidate to be a contact set
			else if (EpsEqual(t2, t0))
			{
				if (EpsGreaterThan(t2, outProjectionInfo.max) || (EpsEqual(t2, outProjectionInfo.max) && outProjectionInfo.sMax.type != ContactSet::FACE))
				{
					outProjectionInfo.max = t2;
					outProjectionInfo.sMax.type = ContactSet::EDGE;
					outProjectionInfo.sMax.vertices.clear();
					outProjectionInfo.sMax.edges.clear();
					outProjectionInfo.sMax.faces.clear();
					outProjectionInfo.sMax.edges.push_back(Edge(&f, (int)f.I(2), (int)f.I(0)));
				}
				
				if (EpsLessThan(t2, outProjectionInfo.min) || (EpsEqual(t2, outProjectionInfo.min) && outProjectionInfo.sMin.type != ContactSet::FACE))
				{
					outProjectionInfo.min = t2;
					outProjectionInfo.sMin.type = ContactSet::EDGE;
					outProjectionInfo.sMin.vertices.clear();
					outProjectionInfo.sMin.faces.clear();
					outProjectionInfo.sMin.edges.clear();
					outProjectionInfo.sMin.edges.push_back(Edge(&f, (int)f.I(2), (int)f.I(0)));
				}
			}

			// Otherwise only a vertex could be a candidate for the contact set
			if (!EpsEqual(t0, t1) || !EpsEqual(t1, t2) || !EpsEqual(t2, t0))
			{
				if (EpsGreaterThan(t0, outProjectionInfo.max))
				{
					outProjectionInfo.max = t0;
					outProjectionInfo.sMax.type = ContactSet::VERTEX;
					outProjectionInfo.sMax.vertices.clear();
					outProjectionInfo.sMax.vertices.push_back(f.I(0));
				}

				if (EpsGreaterThan(t1, outProjectionInfo.max))
				{
					outProjectionInfo.max = t1;
					outProjectionInfo.sMax.type = ContactSet::VERTEX;
					outProjectionInfo.sMax.vertices.clear();
					outProjectionInfo.sMax.vertices.push_back(f.I(1));
				}

				if (EpsGreaterThan(t2, outProjectionInfo.max))
				{
					outProjectionInfo.max = t2;
					outProjectionInfo.sMax.type = ContactSet::VERTEX;
					outProjectionInfo.sMax.vertices.clear();
					outProjectionInfo.sMax.vertices.push_back(f.I(2));
				}
				
				if (EpsLessThan(t0, outProjectionInfo.min))
				{
					outProjectionInfo.min = t0;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertices.clear();
					outProjectionInfo.sMin.vertices.push_back(f.I(0));
				}

				if (EpsLessThan(t1, outProjectionInfo.min))
				{
					outProjectionInfo.min = t1;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertices.clear();
					outProjectionInfo.sMin.vertices.push_back(f.I(1));
				}

				if (EpsLessThan(t2, outProjectionInfo.min))
				{
					outProjectionInfo.min = t2;
					outProjectionInfo.sMin.type = ContactSet::VERTEX;
					outProjectionInfo.sMin.vertices.clear();
					outProjectionInfo.sMin.vertices.push_back(f.I(2));
				}
			}

		}
	}


	bool NoIntersect(float tMax, const float& speed, ProjectionInfo& projectionInfo0, ProjectionInfo& projectionInfo1,
		ProjectionInfo& pCurr0, ProjectionInfo& pCurr1, int& side, float& tFirst, float& tLast)
	{
	
		// c1 is on the "left" of c0 on the direction d		
		if (projectionInfo1.max < projectionInfo0.min)
		{
			if (speed <= 0)  return true;	// intervals are moving apart -> no intersection

			float t = (projectionInfo0.min - projectionInfo1.max) / speed; // time of the first contact
			if (t > tFirst) 
			{ 
				tFirst = t;
				side = -1;
				pCurr0 = projectionInfo0;
				pCurr1 = projectionInfo1;
			}
			if (tFirst > tMax) return true;	// no intersection until tMax

			t = (projectionInfo0.max - projectionInfo1.min) / speed;	// time of the last contact, when c1 surpass c0
			if (t < tLast) tLast = t;
			if (tFirst > tLast) return true;	// No intersection
		}

		// c1 is on the "right" of c0 on the direction d
		else if (projectionInfo0.max < projectionInfo1.min)
		{
			if (speed >= 0)  return true;	// intervals are moving apart -> no intersection

			float t = (projectionInfo0.max - projectionInfo1.min) / speed; // time of the first contact
			if (t > tFirst) 
			{
				tFirst = t;
				side = +1;
				pCurr0 = projectionInfo0;
				pCurr1 = projectionInfo1;
			}
			
			if (tFirst > tMax) return true;	// no intersection until tMax

			t = (projectionInfo0.min - projectionInfo1.max) / speed;	// time of the last contact, when c1 surpass c0
			if (t < tLast) tLast = t;
			if (tFirst > tLast) return true;	// No intersection
		}

		// Intervals are already overlapping 
		else
		{
			if (speed > 0)
			{
				float t = (projectionInfo0.max - projectionInfo1.min) / speed;
				if (t < tLast) tLast = t;
				if (tFirst > tLast) return true;	// No intersection
			}
			if (speed < 0)
			{
				float t = (projectionInfo0.min - projectionInfo1.max) / speed;	// time of the last contact, when c1 surpass c0
				if (t < tLast) tLast = t;
				if (tFirst > tLast) return true;	// No intersection
			}
		}

		return false;
	}


	/**
	 * Compute the intersection set of two polyhedra that are in contact
	 *
	 * @param c0 the first convex polyhedron
	 * @param c0 the first convex polyhedron* 
	 */
	void GetIntersection(RigidBody& c0, RigidBody& c1,
		ProjectionInfo& pInfo0, ProjectionInfo& pInfo1, int side, float tFirst, std::vector<Contact>& outContacts)
	{
		// c0 MAX meet C1 MIN
		if (side == 1)
		{
			// c0 VERTEX- c1 FACE contact (vertex-vertex and vertex-edge are not considered contact)
			if(pInfo0.sMax.type == ContactSet::VERTEX)
			{
				if (pInfo1.sMin.type == ContactSet::FACE) 
				{
					glm::vec3 v = (c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMax.vertices[0]], 1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst);
					
					Contact c 
					(
						Contact::Type::VERTEX_FACE,
						c0,
						c1,
						v,
						c1.OrientationMatrix * pInfo1.sMin.faces[0].N()
					);
					outContacts.push_back(c);
				}
			}

			// c1 VERTEX- c0 FACE contact
			else if (pInfo1.sMin.type == ContactSet::VERTEX)
			{
				if (pInfo0.sMax.type == ContactSet::FACE)
				{
					glm::vec3 v = (c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMin.vertices[0]], 1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst);
					Contact c
					(
						Contact::Type::VERTEX_FACE,
						c1,
						c0,
						v,
						c0.OrientationMatrix * pInfo0.sMax.faces[0].N()
					);
					outContacts.push_back(c);
				}
			}

			// c0 EDGE - c1 FACE || EDGE contact (edge-edge contacts are not considered here
			else if (pInfo0.sMax.type == ContactSet::EDGE)
			{
				glm::vec3 e0[2] = 
				{
					(c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMax.edges.at(0).I(0)],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst),
					(c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMax.edges.at(0).I(1)],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst)
				};

				// c0 EDGE - c1 EDGE
				if (pInfo1.sMin.type == ContactSet::EDGE) 
				{
					glm::vec3 e1[2] =
					{
						(c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMin.edges.at(0).I(0)], 1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst),
						(c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMin.edges.at(0).I(1)], 1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst)
					};
					outContacts = GetEdgeEdgeIntersection(c0, c1, e0[0], e0[1], e1[0], e1[1]);
				}

				// c0 EDGE - c1 FACE
				else if (pInfo1.sMin.type == ContactSet::FACE)
				{
					// Compute the intersection between the edge and all the faces edges 
					std::map<std::string, glm::ivec2> edgesMap;
					
					// Create a list of edges from the face list removing common edges
					for(Face f : pInfo1.sMin.faces) 
					{
						std::stringstream eId, eIdR;
						eId << f.I(1) << f.I(0);
						eIdR << f.I(0) << f.I(1);
						if (edgesMap.find(eId.str()) != edgesMap.end()) edgesMap.erase(eId.str());
						else if (edgesMap.find(eIdR.str()) != edgesMap.end()) edgesMap.erase(eIdR.str());
						else edgesMap.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(0), f.I(1))));					
					
						eId.str(""), eIdR.str("");
						eId << f.I(2) << f.I(1);
						eIdR << f.I(1) << f.I(2);
						if (edgesMap.find(eId.str()) != edgesMap.end()) edgesMap.erase(eId.str());
						else if (edgesMap.find(eIdR.str()) != edgesMap.end()) edgesMap.erase(eIdR.str());
						else edgesMap.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(1), f.I(2))));

						eId.str(""), eIdR.str("");
						eId << f.I(0) << f.I(2);
						eIdR << f.I(2) << f.I(0);
						if (edgesMap.find(eId.str()) != edgesMap.end()) edgesMap.erase(eId.str());
						else if (edgesMap.find(eIdR.str()) != edgesMap.end()) edgesMap.erase(eIdR.str());
						else edgesMap.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(2), f.I(0))));
					
					}

					std::vector<Object3D> edges;
					for (auto it = edgesMap.begin(); it != edgesMap.end(); ++it)
					{
						Object3D edge =
						{
							Object3D::EDGE ,
							{
								(c1.ModelMatrix * glm::vec4(c1.Vertices[it->second.x],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst),
								(c1.ModelMatrix * glm::vec4(c1.Vertices[it->second.y],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst)
							},
							{ 
								Edge(&pInfo1.sMin.faces[0], 0, 1)
							}
						};
						edges.push_back(edge);
					}

					outContacts = GetEdgeFacesIntersection(c0, c1, e0[0], e0[1], edges);
				}
			}

			// c0 FACE - c1 EDGE || FACE  contact
			else if (pInfo0.sMax.type == ContactSet::FACE)
			{
					std::map<std::string, glm::ivec2> edgesMap0, edgesMap1;
					std::vector<Object3D> edges0, edges1;

					// Transform the first face into a list of edges
					for (Face f : pInfo0.sMax.faces)
					{
						std::stringstream eId, eIdR;
						eId << f.I(1) << f.I(0);
						eIdR << f.I(0) << f.I(1);
						if (edgesMap0.find(eId.str()) != edgesMap0.end()) edgesMap0.erase(eId.str());
						else if (edgesMap0.find(eIdR.str()) != edgesMap0.end()) edgesMap0.erase(eIdR.str());
						else edgesMap0.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(0), f.I(1))));

						eId.str(""), eIdR.str("");
						eId << f.I(2) << f.I(1);
						eIdR << f.I(1) << f.I(2);
						if (edgesMap0.find(eId.str()) != edgesMap0.end()) edgesMap0.erase(eId.str());
						else if (edgesMap0.find(eIdR.str()) != edgesMap0.end()) edgesMap0.erase(eIdR.str());
						else edgesMap0.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(1), f.I(2))));

						eId.str(""), eIdR.str("");
						eId << f.I(0) << f.I(2);
						eIdR << f.I(2) << f.I(0);
						if (edgesMap0.find(eId.str()) != edgesMap0.end()) edgesMap0.erase(eId.str());
						else if (edgesMap0.find(eIdR.str()) != edgesMap0.end()) edgesMap0.erase(eIdR.str());
						else edgesMap0.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(2), f.I(0))));
					}
					
					for (auto it = edgesMap0.begin(); it != edgesMap0.end(); ++it)
					{
						Object3D edge =
						{
							Object3D::EDGE ,
							{
								(c0.ModelMatrix * glm::vec4(c0.Vertices[it->second.x],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst),
								(c0.ModelMatrix * glm::vec4(c0.Vertices[it->second.y],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst)
							},
							{
								Edge(&pInfo1.sMax.faces[0], 0, 1)
							}
						};
						edges0.push_back(edge);
					}

					// Transform the second face into a list of edges
					for (Face f : pInfo1.sMin.faces)
					{
						std::stringstream eId, eIdR;
						eId << f.I(1) << f.I(0);
						eIdR << f.I(0) << f.I(1);
						if (edgesMap1.find(eId.str()) != edgesMap1.end()) edgesMap1.erase(eId.str());
						else if (edgesMap1.find(eIdR.str()) != edgesMap1.end()) edgesMap1.erase(eIdR.str());
						else edgesMap1.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(0), f.I(1))));

						eId.str(""), eIdR.str("");
						eId << f.I(2) << f.I(1);
						eIdR << f.I(1) << f.I(2);
						if (edgesMap1.find(eId.str()) != edgesMap1.end()) edgesMap1.erase(eId.str());
						else if (edgesMap1.find(eIdR.str()) != edgesMap1.end()) edgesMap1.erase(eIdR.str());
						else edgesMap1.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(1), f.I(2))));

						eId.str(""), eIdR.str("");
						eId << f.I(0) << f.I(2);
						eIdR << f.I(2) << f.I(0);
						if (edgesMap1.find(eId.str()) != edgesMap1.end()) edgesMap1.erase(eId.str());
						else if (edgesMap1.find(eIdR.str()) != edgesMap1.end()) edgesMap1.erase(eIdR.str());
						else edgesMap1.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(2), f.I(0))));
					}
					for (auto it = edgesMap1.begin(); it != edgesMap1.end(); ++it)
					{
						Object3D edge =
						{
							Object3D::EDGE ,
							{
								(c1.ModelMatrix * glm::vec4(c1.Vertices[it->second.x],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst),
								(c1.ModelMatrix * glm::vec4(c1.Vertices[it->second.y],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst)
							},
							{
								Edge(&pInfo1.sMin.faces[0], 0, 1)
							}
						};
						edges1.push_back(edge);
					}
					
					if ((pInfo1.sMin.type == ContactSet::EDGE)) 
					{
						Object3D outIntersection;
						glm::vec3 e1[2] =
						{
							(c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMin.edges.at(0).I(0)], 1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst),
							(c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMin.edges.at(0).I(1)], 1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst)
						};

						outContacts = GetEdgeFacesIntersection(c1, c0, e1[0], e1[1], edges0);
					}
					else if ((pInfo1.sMin.type == ContactSet::FACE))
					{
						outContacts = GetCoplanarFaceFaceIntersection(c0, c1, edges0, edges1);
					}
			}
		}


		// c1 MAX meet c0 MIN
		if (side == -1)
		{
			// c1 VERTEX- c0 FACE contact (vertex-vertex and vertex-edge are not considered contact)
			if (pInfo1.sMax.type == ContactSet::VERTEX)
			{
				if (pInfo0.sMin.type == ContactSet::FACE)
				{
					glm::vec3 v = (c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMax.vertices[0]], 1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst);
					Contact c
					(
						Contact::Type::VERTEX_FACE,
						c1,
						c0,
						v,
						c0.OrientationMatrix * pInfo0.sMax.faces[0].N()
					);
					outContacts.push_back(c);
				}
			}

			// c0 VERTEX- c1 FACE contact 
			else if (pInfo0.sMin.type == ContactSet::VERTEX)
			{
				if (pInfo1.sMax.type == ContactSet::FACE) 
				{
					glm::vec3 v = (c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMin.vertices[0]], 1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst);
					Contact c
					(
						Contact::Type::VERTEX_FACE,
						c0,
						c1,
						v,
						c1.OrientationMatrix * pInfo1.sMax.faces[0].N()
					);
					outContacts.push_back(c);
				}
			}

			// c1 EDGE - c0 EDGE || FACE  contact
			else if (pInfo1.sMax.type == ContactSet::EDGE)
			{
				glm::vec3 e1[2] =
				{
					(c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMax.edges.at(0).I(0)],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst),
					(c1.ModelMatrix * glm::vec4(c1.Vertices[pInfo1.sMax.edges.at(0).I(1)],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst)
				};

				// c1 EDGE - c0 EDGE
				if (pInfo0.sMin.type == ContactSet::EDGE)
				{
					glm::vec3 e0[2] =
					{
						(c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMin.edges.at(0).I(0)], 1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst),
						(c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMin.edges.at(0).I(1)], 1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst)
					};
					outContacts = GetEdgeEdgeIntersection(c1, c0, e1[0], e1[1], e0[0], e0[1]);
				}

				// c1 EDGE - c0 FACE
				else if (pInfo0.sMin.type == ContactSet::FACE)
				{
					// Compute the intersection between the edge and all the faces edges 
					std::map<std::string, glm::ivec2> edgesMap;

					// Create a list of edges from the face list removing common edges
					for (Face f : pInfo0.sMin.faces)
					{
						std::stringstream eId, eIdR;
						eId << f.I(1) << f.I(0);
						eIdR << f.I(0) << f.I(1);
						if (edgesMap.find(eId.str()) != edgesMap.end()) edgesMap.erase(eId.str());
						else if (edgesMap.find(eIdR.str()) != edgesMap.end()) edgesMap.erase(eIdR.str());
						else edgesMap.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(0), f.I(1))));

						eId.str(""), eIdR.str("");
						eId << f.I(2) << f.I(1);
						eIdR << f.I(1) << f.I(2);
						if (edgesMap.find(eId.str()) != edgesMap.end()) edgesMap.erase(eId.str());
						else if (edgesMap.find(eIdR.str()) != edgesMap.end()) edgesMap.erase(eIdR.str());
						else edgesMap.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(1), f.I(2))));

						eId.str(""), eIdR.str("");
						eId << f.I(0) << f.I(2);
						eIdR << f.I(2) << f.I(0);
						if (edgesMap.find(eId.str()) != edgesMap.end()) edgesMap.erase(eId.str());
						else if (edgesMap.find(eIdR.str()) != edgesMap.end()) edgesMap.erase(eIdR.str());
						else edgesMap.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(2), f.I(0))));
					}


					std::vector<Object3D> edges;
					for (auto it = edgesMap.begin(); it != edgesMap.end(); ++it)
					{
						Object3D edge =
						{
							Object3D::EDGE ,
							{
								(c0.ModelMatrix * glm::vec4(c0.Vertices[it->second.x],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst),
								(c0.ModelMatrix * glm::vec4(c0.Vertices[it->second.y],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst)
							},
							{
								Edge(&pInfo1.sMin.faces[0], 0, 1)
							}
						};
						edges.push_back(edge);
					}

					Object3D outIntersection;
					outContacts = GetEdgeFacesIntersection(c1, c0, e1[0], e1[1], edges);
				}
			}

			// c1 FACE - c0 EDGE || FACE
			else if (pInfo1.sMax.type == ContactSet::FACE)
			{
				std::map<std::string, glm::ivec2> edgesMap1, edgesMap0;
				std::vector<Object3D> edges1, edges0;

				// Transform the first face into a list of edges
				for (Face f : pInfo1.sMax.faces)
				{
					std::stringstream eId, eIdR;
					eId << f.I(1) << f.I(0);
					eIdR << f.I(0) << f.I(1);
					if (edgesMap1.find(eId.str()) != edgesMap1.end()) edgesMap1.erase(eId.str());
					else if (edgesMap1.find(eIdR.str()) != edgesMap1.end()) edgesMap1.erase(eIdR.str());
					else edgesMap1.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(0), f.I(1))));

					eId.str(""), eIdR.str("");
					eId << f.I(2) << f.I(1);
					eIdR << f.I(1) << f.I(2);
					if (edgesMap1.find(eId.str()) != edgesMap1.end()) edgesMap1.erase(eId.str());
					else if (edgesMap1.find(eIdR.str()) != edgesMap1.end()) edgesMap1.erase(eIdR.str());
					else edgesMap1.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(1), f.I(2))));

					eId.str(""), eIdR.str("");
					eId << f.I(0) << f.I(2);
					eIdR << f.I(2) << f.I(0);
					if (edgesMap1.find(eId.str()) != edgesMap1.end()) edgesMap1.erase(eId.str());
					else if (edgesMap1.find(eIdR.str()) != edgesMap1.end()) edgesMap1.erase(eIdR.str());
					else edgesMap1.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(2), f.I(0))));
				}

				for (auto it = edgesMap1.begin(); it != edgesMap1.end(); ++it)
				{
					Object3D edge =
					{
						Object3D::EDGE ,
						{
							(c1.ModelMatrix * glm::vec4(c1.Vertices[it->second.x],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst),
							(c1.ModelMatrix * glm::vec4(c1.Vertices[it->second.y],1.0f)).xyz() + (c1.LinearVelocity * (float)tFirst)
						},
						{
							Edge( &pInfo1.sMax.faces[0], 0, 1)
						}
					};
					edges1.push_back(edge);
				}

				// Transform the second face into a list of edges
				for (Face f : pInfo0.sMin.faces)
				{
					std::stringstream eId, eIdR;
					eId << f.I(1) << f.I(0);
					eIdR << f.I(0) << f.I(1);
					if (edgesMap0.find(eId.str()) != edgesMap0.end()) edgesMap0.erase(eId.str());
					else if (edgesMap0.find(eIdR.str()) != edgesMap0.end()) edgesMap0.erase(eIdR.str());
					else edgesMap0.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(0), f.I(1))));

					eId.str(""), eIdR.str("");
					eId << f.I(2) << f.I(1);
					eIdR << f.I(1) << f.I(2);
					if (edgesMap0.find(eId.str()) != edgesMap0.end()) edgesMap0.erase(eId.str());
					else if (edgesMap0.find(eIdR.str()) != edgesMap0.end()) edgesMap0.erase(eIdR.str());
					else edgesMap0.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(1), f.I(2))));

					eId.str(""), eIdR.str("");
					eId << f.I(0) << f.I(2);
					eIdR << f.I(2) << f.I(0);
					if (edgesMap0.find(eId.str()) != edgesMap0.end()) edgesMap0.erase(eId.str());
					else if (edgesMap0.find(eIdR.str()) != edgesMap0.end()) edgesMap0.erase(eIdR.str());
					else edgesMap0.insert(std::pair<std::string, glm::ivec2>(eId.str(), glm::ivec2(f.I(2), f.I(0))));
				}
				for (auto it = edgesMap0.begin(); it != edgesMap0.end(); ++it)
				{
					Object3D edge =
					{
						Object3D::EDGE ,
						{
							(c0.ModelMatrix * glm::vec4(c0.Vertices[it->second.x],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst),
							(c0.ModelMatrix * glm::vec4(c0.Vertices[it->second.y],1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst)
						},
						{
							Edge(&pInfo1.sMin.faces[0], 0, 1)
						}
					};
					edges0.push_back(edge);
				}

				if ((pInfo0.sMin.type == ContactSet::EDGE))
				{
					Object3D outIntersection;
					glm::vec3 e0[2] =
					{
						(c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMin.edges.at(0).I(0)], 1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst),
						(c0.ModelMatrix * glm::vec4(c0.Vertices[pInfo0.sMin.edges.at(0).I(1)], 1.0f)).xyz() + (c0.LinearVelocity * (float)tFirst)
					};
					outContacts = GetEdgeFacesIntersection(c0, c1, e0[0], e0[1], edges1);
				}
				else if ((pInfo0.sMin.type == ContactSet::FACE))
				{
					outContacts = GetCoplanarFaceFaceIntersection(c1, c0, edges1, edges0);
				}
			}
		}
	}


	std::vector<Contact> GetEdgeEdgeIntersection(RigidBody& rb0, RigidBody& rb1, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& q0, const glm::vec3& q1)
	{
		std::vector<Contact> contacts;

		glm::vec3 p = p1 - p0;
		glm::vec3 q = q1 - q0;
		glm::vec3 r = q0 - p0;

		glm::vec3 e = glm::cross(p, r);
		
		// The two edges lay on the same line, in the engine we discard this type of intersection
		if (EpsZero(e)) return contacts;
		/*
		{
			glm::vec3 d = glm::normalize(p);
			float lp0 = glm::dot(p0, d);
			float lp1 = glm::dot(p1, d);
			float lq0 = glm::dot(q0, d);
			float lq1 = glm::dot(q1, d);

			// Sort segments point so the maximum value is in lp1 and lq1 and the minimum value is in lp0 and lq0
			if (lp0 > lp1) std::swap(lp0, lp1);
			if (lq0 > lq1) std::swap(lq0, lq1);

			// Check if the two segments overlap
			if(lq0 > lp1 || lp0 > lq1) 
			{
				// The two segments do not overlap
				outIntersection.type = Object3D::EMPTY;
				return false;
			}

			// Check if the two segments share a point
			if(Equal(lq0, lp1))
			{
				// The two segments share an extreme
				outIntersection.type = Object3D::VERTEX;
				outIntersection.vertices.push_back(d * lq0);
				return true;
			}
			if (Equal(lp0, lq1))
			{
				// The two segments share an extreme
				outIntersection.type = Object3D::VERTEX;
				outIntersection.vertices.push_back(d * lp0);
				return true;
			}
		
			// The two segments overlap, the overlap set is [max of minimums, min of maximus]
			float e0 = glm::max(lp0, lq0);
			float e1 = glm::min(lp1, lq1);
			outIntersection.type = Object3D::EDGE;
			outIntersection.vertices.push_back(d * e0);
			outIntersection.vertices.push_back(d * e1);
			outIntersection.edges.push_back(glm::ivec2(0, 1));
			return true;
		}*/

		glm::vec3 t = glm::cross(q, r);
		glm::vec3 s = glm::cross(q, p);

		// Check if the two lines are parallel, but do not lay on the same line
		if (EpsZero(s))
		{
			// The two segments do not overlap
			//outIntersection.type = Object3D::EMPTY;
			return contacts;
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

			if((lp0 <= pI && pI <= lp1) && (lq0 <= qI && qI <= lq1))
			{
				//outIntersection.type = Object3D::VERTEX;
				glm::vec3 edgeA[2] = { p0, p1 };
				glm::vec3 edgeB[2] = { q0, q1 };

				contacts.push_back(
					Contact
					(
						Contact::Type::EDGE_EDGE,
						rb0,
						rb1,
						i,
						glm::normalize(glm::cross(edgeA[1] - edgeA[0], edgeB[1] - edgeB[0])),
						edgeA,
						edgeB
					)
				);

				// Select the normal as the one pointing outside the second object
				glm::vec3 normal = glm::normalize(glm::cross(edgeA[1] - edgeA[0], edgeB[1] - edgeB[0]));
				float dir = glm::dot(normal, glm::normalize(contacts[0].point - rb1.Position));
				contacts[0].normal = (dir > 0) ? normal : -1.0f * normal;

				return contacts;
			}
		}

		// Contacts is empty
		return contacts;
	}


	std::vector<Contact> GetEdgeFacesIntersection(RigidBody& rb0, RigidBody& rb1, const glm::vec3 p0, const glm::vec3 p1, std::vector<Object3D> edges)
	{
		std::vector<Contact> contacts;

		Object3D outIntersection;
		outIntersection.type = Object3D::VERTEX;
		glm::vec3 insidePoint = glm::vec3();

		// Compute the face normal
		glm::vec3 faceNormal;
		if (edges[0].vertices[0] == edges[1].vertices[1])
			faceNormal = glm::normalize(glm::cross(edges[1].vertices[1] - edges[1].vertices[0], edges[0].vertices[1] - edges[0].vertices[0]));
		else
			faceNormal = glm::normalize(glm::cross(edges[0].vertices[1] - edges[0].vertices[0], edges[1].vertices[1] - edges[1].vertices[0]));
		//glm::vec3 faceNormal = glm::normalize(glm::cross(edges[0].vertices[1] - edges[0].vertices[0], edges[1].vertices[1] - edges[1].vertices[0]));

		for(Object3D e : edges) 
		{
			// I already know that the intersection could be one point, or none
			std::vector<Contact> intersection = GetEdgeEdgeIntersection(rb0, rb1, p0, p1, e.vertices[0], e.vertices[1]);
			if (!intersection.empty())
			{
				for (Contact c : intersection) contacts.push_back(c);
				////contacts.insert(contacts.end(), intersection.begin(), intersection.end());
				//outIntersection.vertices.insert(outIntersection.vertices.end(), v.vertices.begin(), v.vertices.end());
				glm::vec3 edgeNormal = glm::normalize(glm::cross(e.vertices[1] - e.vertices[0], faceNormal));
				insidePoint = (glm::dot(p1 - p0, edgeNormal) < 0) ? p1 : p0;
			}
		}
		
		// The whole edge is inside or outside the face
		if(contacts.empty()) 
		{
			bool inside = true;
			// Check if the segment is outside the polygon
			for (Object3D e : edges)
			{
				glm::vec3 edgeNormal = glm::normalize(glm::cross(e.vertices[1] - e.vertices[0], faceNormal));
				inside = ((glm::dot(p1 - (e.vertices[1] + e.vertices[0])/2.0f, edgeNormal) < 0) ? true : false) && inside; // if one point is in front of one the face -> the point is outside -> the whole segment is outside
			}
			//inside = false;
			if(inside) {
				
				Contact c0 
				(
					Contact::Type::VERTEX_FACE,
					rb0,
					rb1,
					p0,
					faceNormal
				);
			
				Contact c1 
				(
					Contact::Type::VERTEX_FACE,
					rb0,
					rb1,
					p1,
					faceNormal
				);

				contacts.push_back(c0);
				contacts.push_back(c1);
			}
		}

		// Only one intersection -> one point	is in the face
		if (contacts.size() == 1)
		{
			contacts.push_back(
				Contact
				(
					Contact::Type::VERTEX_FACE,
					rb0,
					rb1,
					insidePoint,
					faceNormal
				));
		}

		// If intersection is 2 the points are outside the face
		// The intersection cannot be > 2
		/*for(glm::vec3 p : outIntersection.vertices) 
		{
			
			Contact c1 = Contact
			{
				Contact::VERTEX_FACE_CONTACT,
				p
			};
			outContacts.push_back(p);
		}*/
		return contacts;
	}



	std::vector<Contact> GetCoplanarFaceFaceIntersection(RigidBody& rb0, RigidBody& rb1, std::vector<Object3D> edges0, std::vector<Object3D> edges1)
	{
		std::vector<Contact> contacts;

		for (Object3D e0 : edges0)
		{
			for(Contact c : GetEdgeFacesIntersection(rb0, rb1, e0.vertices[0], e0.vertices[1], edges1)) 
			{
				if(std::find(contacts.begin(), contacts.end(), c) == contacts.end() ) contacts.push_back(c);
				//contacts.push_back(c);
			}	
		}

		// Check what points of face 1 are inside face0
		bool insideE00 = true;
		bool insideE01 = true;
		
		// Compute the face normal
		glm::vec3 face0Normal;
		if(edges0[0].vertices[0] == edges0[1].vertices[1])
			face0Normal = glm::normalize(glm::cross(edges0[1].vertices[1] - edges0[1].vertices[0], edges0[0].vertices[1] - edges0[0].vertices[0]));
		else 
			face0Normal = glm::normalize(glm::cross(edges0[0].vertices[1] - edges0[0].vertices[0], edges0[1].vertices[1] - edges0[1].vertices[0]));

		for (Object3D e : edges1)
		{
			insideE00 = true;
			insideE01 = true;
			for (Object3D e0 : edges0)
			{
				glm::vec3 edgeNormal = glm::cross(e0.vertices[1] - e0.vertices[0], face0Normal);
				insideE00 = ((glm::dot(e.vertices[0] - ((e0.vertices[0] + e0.vertices[1]) / 2.0f), edgeNormal) <= 0) ? true : false) && insideE00;
				insideE01 = ((glm::dot(e.vertices[1] - ((e0.vertices[0] + e0.vertices[1]) / 2.0f), edgeNormal) <= 0) ? true: false) && insideE01;
			}

			if (insideE00) {
				Contact c
					(
						Contact::Type::VERTEX_FACE,
						rb1,
						rb0,
						e.vertices[0],
						face0Normal
					);
				if (std::find(contacts.begin(), contacts.end(), c) == contacts.end()) contacts.push_back(c);
			}

			if (insideE01) {
				Contact c
					(
						Contact::Type::VERTEX_FACE,
						rb1,
						rb0,
						e.vertices[1],
						face0Normal
					);
				if (std::find(contacts.begin(), contacts.end(), c) == contacts.end()) contacts.push_back(c);
			}
		}

		return contacts;
	}
}