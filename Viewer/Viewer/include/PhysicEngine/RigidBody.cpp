#include <glm/gtx/quaternion.hpp>
#include <map>
#include <unordered_set>

#include "math/comparison.h"
#include "geometry/Edge.h"
#include "render/render.h"
#include "RigidBody.h"


namespace PhysicEngine
{

	RigidBody::RigidBody(std::vector<glm::vec3>& vertices, std::vector<unsigned int>& triangles)
		: Vertices(vertices)
	{
		BuildGeometry(triangles);
		BuildRenderMesh(triangles);
		ComputeUniqueEdgeDirections();
		ComputeCenterOfMassAndInertiaTensor(triangles);
		ComputeOrientedBoundingBox();

		SetForceFunction(NullForce);
		SetTorqueFunction(NullTorque);
	
		// The object is initially still and unrotated
		SetState(glm::vec3(0.0f, 0.0f, 0.0f), glm::identity<glm::quat>(), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
	}


	RenderPolyhedron RigidBody::GetRenderMesh() 
	{
		return RenderMesh;
	}


	void RigidBody::BuildGeometry(std::vector<unsigned int>& triangles)
	{

		// Merge coplanar faces.
		// Stores all the faces grouped by their normals.
		// Due the 3D mesh is convex all the face that have the same normal are adjacent and could be merged 
		// into a single face.
		std::unordered_map<glm::vec3, std::vector<glm::ivec2>, HashGlmVec3, EpsCompareGlmVec3> nfMap;

		for (size_t j = 0; j < triangles.size(); j += 3) {

			// Compute the face normal assuming the COUNTERCLOCKWISE order
			glm::vec3 normal = glm::normalize(
				glm::cross(Vertices[triangles[j + 1]] - Vertices[triangles[j]],
					Vertices[triangles[j + 2]] - Vertices[triangles[j + 1]]));

			std::vector<glm::ivec2> es;
			es.push_back({ triangles[j], triangles[j + 1] });
			es.push_back({ triangles[j + 1], triangles[j + 2] });
			es.push_back({ triangles[j + 2], triangles[j] });

			// Remove touching edges of coplanar triangles
			if (nfMap.find(normal) == nfMap.end())
			{
				nfMap[normal] = es;
			}
			else
			{
				// If the edges of the new triangle are already in the list of the edges that share the same normal
				// it means that is an internal face edges and could be removed
				for (glm::ivec2 e0 : es)
				{
					// Removing edges that appears twice (internal face edges) ...
					std::vector<glm::ivec2>& edges = nfMap[normal];

					auto eIt = std::find_if(edges.begin(), edges.end(),
						[&e0](glm::ivec2 e1)
						{
							// If there is a duplicate edge already in the list it is in reverse order
							return (e0.x == e1.y && e0.y == e1.x);
						});
					if (eIt != edges.end())
					{
						// If found, remove the duplicate edge from the list ...
						edges.erase(std::remove(edges.begin(), edges.end(), *eIt), edges.end());
					}

					// ... and add the remaing edges to the list 
					else
					{
						edges.push_back(e0);
					}
				}
			}
		}

		// Create the list of face vertices.
		std::vector<Face> faces;
		for (auto nf : nfMap)
		{
			std::vector<int> vId;

			// Sort all edges so that the first vertex of a follower is the last vertex of the previous	one.
			for (int i = nf.second.back().y; !nf.second.empty();)
			{
				vId.push_back(i);
				auto e = std::find_if(nf.second.begin(), nf.second.end(),
					[&i](glm::ivec2 e0)
					{
						return e0.x == i;
					}
				);
				i = (*e).y;
				nf.second.erase(e);
			}

			faces.push_back(Face(this, vId));
		}
	}


	void RigidBody::ComputeUniqueEdgeDirections() 
	{
		for (Face f : Faces)
		{
			for (Edge e : f.Edges)
			{
				if (std::find_if(EdgesDirs.begin(), EdgesDirs.end(),
					[&e](glm::vec3 d)
					{
						return EpsEqual(e.D(), d) || EpsEqual(e.D(), -d);
					}) == EdgesDirs.end())
				{
					EdgesDirs.push_back(e.D());
				}
			}
		}
	}


	void RigidBody::BuildRenderMesh(std::vector<unsigned int> triangles)
	{
		for (size_t i = 0; i < triangles.size()-2; i++)
		{
			// Computing normal 
			glm::vec3 n = glm::normalize(
				glm::cross(Vertices[triangles[i + 1]] - Vertices[triangles[i]], Vertices[triangles[i + 2]] - Vertices[triangles[i + 1]])
				);

			RenderMesh.Vertices.push_back(RenderVertex{ Vertices[triangles[i]], n });
			RenderMesh.Vertices.push_back(RenderVertex{ Vertices[triangles[i + 1]], n });
			RenderMesh.Vertices.push_back(RenderVertex{ Vertices[triangles[i + 2]], n });
		}

		RenderMesh.Triangles = triangles;
	}


	void RigidBody::ComputeCenterOfMassAndInertiaTensor(std::vector<unsigned int>& triangles) {

		float integral[10] = { 0,0,0,0,0,0,0,0,0,0 };

		for (int i = 0; i < triangles.size(); i +=3) {

			// Get the verticies of this triangle
			glm::vec3 v0 = Vertices[triangles[i]];
			glm::vec3 v1 = Vertices[triangles[i + 1]];
			glm::vec3 v2 = Vertices[triangles[i + 2]];

			// Get cross product of edges [v1-v0] x [v2-v1] 
			glm::vec3 edge0 = v1 - v0;
			glm::vec3 edge1 = v2 - v0;
			glm::vec3 d = glm::cross(edge0, edge1);

			float f1x, f2x, f3x, f1y, f2y, f3y, f1z, f2z, f3z, g0x, g1x, g2x, g0y, g1y, g2y, g0z, g1z, g2z;

			ComputeCenterOfMassAndInertiaTensorSubExpression(v0.x, v1.x, v2.x, f1x, f2x, f3x, g0x, g1x, g2x);
			ComputeCenterOfMassAndInertiaTensorSubExpression(v0.y, v1.y, v2.y, f1y, f2y, f3y, g0y, g1y, g2y);
			ComputeCenterOfMassAndInertiaTensorSubExpression(v0.z, v1.z, v2.z, f1z, f2z, f3z, g0z, g1z, g2z);

			integral[0] += d.x * f1x;
			integral[1] += d.x * f2x;
			integral[2] += d.y * f2y;
			integral[3] += d.z * f2z;
			integral[4] += d.x * f3x;
			integral[5] += d.y * f3y;
			integral[6] += d.z * f3z;
			integral[7] += d.x * (v0.y * g0x + v1.y * g1x + v2.y * g2x);
			integral[8] += d.y * (v0.z * g0y + v1.z * g1y + v2.z * g2y);
			integral[9] += d.z * (v0.x * g0z + v1.x * g1z + v2.x * g2z);
		}

		integral[0] *= oneDiv6;
		integral[1] *= oneDiv24;
		integral[2] *= oneDiv24;
		integral[3] *= oneDiv24;
		integral[4] *= oneDiv60;
		integral[5] *= oneDiv60;
		integral[6] *= oneDiv60;
		integral[7] *= oneDiv120;
		integral[8] *= oneDiv120;
		integral[9] *= oneDiv120;

		Mass = integral[0];
		InvertedMass = 1 / Mass;

		CenterOfMass.x = integral[1] / Mass;
		CenterOfMass.y = integral[2] / Mass;
		CenterOfMass.z = integral[3] / Mass;

		// Intertia tensor relative to world origin
		IntertiaTensor[0][0] = integral[5] + integral[6]; // xx
		IntertiaTensor[1][1] = integral[4] + integral[6]; // yy
		IntertiaTensor[2][2] = integral[4] + integral[5]; // zz
		IntertiaTensor[0][1] = IntertiaTensor[1][0] = -integral[7]; // xy, yx
		IntertiaTensor[1][2] = IntertiaTensor[2][1] = -integral[8]; //yz, zy
		IntertiaTensor[0][2] = IntertiaTensor[2][0] = -integral[9];  //xz, zx

		// Intertia tensor relative to center of mass
		IntertiaTensor[0][0] -= Mass * (CenterOfMass.y * CenterOfMass.y + CenterOfMass.z * CenterOfMass.z);
		IntertiaTensor[1][1] -= Mass * (CenterOfMass.z * CenterOfMass.z + CenterOfMass.x * CenterOfMass.x);
		IntertiaTensor[2][2] -= Mass * (CenterOfMass.x * CenterOfMass.x + CenterOfMass.y * CenterOfMass.y);
		IntertiaTensor[0][1] += Mass * CenterOfMass.x * CenterOfMass.y;
		IntertiaTensor[1][0] += Mass * CenterOfMass.y * CenterOfMass.z;
		IntertiaTensor[0][2] += Mass * CenterOfMass.z * CenterOfMass.x;

		InvertedIntertiaTensor = glm::inverse(IntertiaTensor);
	}


	void RigidBody::ComputeCenterOfMassAndInertiaTensorSubExpression(float& w0, float& w1, float& w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2) {

		float temp0, temp1, temp2;

		temp0 = w0 + w1;
		f1 = temp0 + w2;
		temp1 = w0 * w0;
		temp2 = temp1 + w1 * temp0;
		f2 = temp2 + w2 * f1;
		f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
		g0 = f2 + w0 * (f1 + w0);
		g1 = f2 + w1 * (f1 + w1);
		g2 = f2 + w2 * (f1 + w2);
	}


	void RigidBody::ComputeModelMatrix()
	{
		ModelMatrix = glm::translate(glm::mat4(1.0f), Position) * glm::mat4(OrientationQuaternion);
	}


	void RigidBody::ComputeOrientedBoundingBox() {

		Eigen::Matrix<float, 3, 3> covarianceMatrix;
		float means[3] = { 0, 0, 0 };

		for (glm::vec3 v : Vertices) {
			means[0] += v.x,
				means[1] += v.y,
				means[2] += v.z;
		}

		means[0] /= Vertices.size();
		means[1] /= Vertices.size();
		means[2] /= Vertices.size();

		// Fill the covariance matrix
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {

				covarianceMatrix(i, j) = 0.0;
				for (glm::vec3 v : Vertices) {
					covarianceMatrix(i, j) += (means[i] - v[i]) * (means[j] - v[j]);
				}

				covarianceMatrix(i, j) /= Vertices.size() - 1;
			}
		}

		// Use Eigen library to compute covariance matrix eigenvectors 
		Eigen::EigenSolver<Eigen::Matrix<float, 3, 3> > s(covarianceMatrix);
		glm::vec3 c0 = { s.eigenvectors().col(0)[0].real(), s.eigenvectors().col(0)[1].real(), s.eigenvectors().col(0)[2].real() };
		glm::vec3 c1 = { s.eigenvectors().col(1)[0].real(), s.eigenvectors().col(1)[1].real(), s.eigenvectors().col(1)[2].real() };
		glm::vec3 c2 = { s.eigenvectors().col(2)[0].real(), s.eigenvectors().col(2)[1].real(), s.eigenvectors().col(2)[2].real() };

		// Compute the projection of points over the eigenvectors direction
		glm::vec3 min = { 9999, 9999, 9999 };
		glm::vec3 max = { -9999, -9999, -9999 };
		for (glm::vec3 v : Vertices) {
			min = glm::min(glm::vec3(glm::dot(c0, v), glm::dot(c1, v), glm::dot(c2, v)), min);
			max = glm::max(glm::vec3(glm::dot(c0, v), glm::dot(c1, v), glm::dot(c2, v)), max);
		}

		// Center of the bouding box
		glm::vec3 center = (min + max) / 2.0f;

		// Half of a side 
		glm::vec3 extension = (max - min) / 2.0f;

		// Trasform center to woold coordinates
		glm::vec3 b0 = { s.eigenvectors().row(0)[0].real(), s.eigenvectors().row(0)[1].real(), s.eigenvectors().row(0)[2].real() };
		glm::vec3 b1 = { s.eigenvectors().row(1)[0].real(), s.eigenvectors().row(1)[1].real(), s.eigenvectors().row(1)[2].real() };
		glm::vec3 b2 = { s.eigenvectors().row(2)[0].real(), s.eigenvectors().row(2)[1].real(), s.eigenvectors().row(2)[2].real() };
		center = glm::vec3(glm::dot(b0, center), glm::dot(b1, center), glm::dot(b2, center));

		// Save the transformation matrix
		glm::mat4 M = {
			s.eigenvectors().row(0)[0].real(), s.eigenvectors().row(1)[0].real(), s.eigenvectors().row(2)[0].real(), 0,
			s.eigenvectors().row(0)[1].real(), s.eigenvectors().row(1)[1].real(), s.eigenvectors().row(2)[1].real(), 0,
			s.eigenvectors().row(0)[2].real(), s.eigenvectors().row(1)[2].real(), s.eigenvectors().row(2)[2].real(), 0,
			center[0], center[1], center[2], 1
		};

		boundingBox = {
			{(min + max) / 2.0f},	// Untrasformed center
			{extension},			// Half of the extension of a side of the bounding box
			{ M },					// Bounding box transformation matrix
			{   // Bounding box trasformed vertices 
				center - c0 * extension.x - c1 * extension.y - c2 * extension.z,
				center + c0 * extension.x - c1 * extension.y - c2 * extension.z,
				center + c0 * extension.x - c1 * extension.y + c2 * extension.z,
				center - c0 * extension.x - c1 * extension.y + c2 * extension.z,
				center - c0 * extension.x + c1 * extension.y - c2 * extension.z,
				center + c0 * extension.x + c1 * extension.y - c2 * extension.z,
				center + c0 * extension.x + c1 * extension.y + c2 * extension.z,
				center - c0 * extension.x + c1 * extension.y + c2 * extension.z
			},
			{	// Boundign sixteen box edges, each two integer represent an edge
				0, 1, 1, 2, 2, 3, 3 ,0 , 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
			}
		};
	}


	void RigidBody::UpdateState(float t, float dt)
	{
	/*	float halfdt = dt / 2.0f;
		float sixthdt = dt / 6.0f;

		glm::vec3 Xn, Pn, Ln, Vn, Wn;
		glm::quat Qn;
		glm::mat3 Rn;

		// k1 = F(t, State_0) 
		glm::vec3 k1_X = LinearVelocity;
		glm::quat k1_Q = 0.5f * (glm::quat(0.0f, AngularVelocity) * OrientationQuaternion);
		glm::vec3 k1_P = InternalForce + ExternalForce;
		glm::vec3 k1_L = InternalTorque + ExternalTorque;

		// State_1 = State_0 + dt/2 * k1
		Xn = Position + halfdt * k1_X;
		Qn = OrientationQuaternion + halfdt * k1_Q;
		Pn = LinearMomentum + halfdt * k1_P;
		Ln = AngularMomentum + halfdt * k1_L;
		ComputeSecondaryState(Xn, Qn, Pn, Ln, Rn, Vn, Wn);

		// k2 = F(t + dt/2, State_1)
		glm::vec3 k2_X = Vn;
		glm::quat k2_Q = 0.5f * (glm::quat(0.0f, Wn) * Qn);
		glm::vec3 k2_P = Force(t + halfdt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);
		glm::vec3 k2_L = Torque(t + halfdt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);

		// Set the internal torque and force (contact forces) to zero becouse we take into account
		// contact force only at the current step. The following steps of the Runge-Kutta methods are 
		// ahead of this thime and contact forces do not exists anymore (maybe :))
		InternalForce = InternalTorque = glm::vec3(0);
	
		// State_2 = State_1 + dt/2 * k2
		Xn = Position + halfdt * k2_X;
		Qn = OrientationQuaternion + halfdt * k2_Q;
		Pn = LinearMomentum + halfdt * k2_P;
		Ln = AngularMomentum + halfdt * k2_L;
		ComputeSecondaryState(Xn, Qn, Pn, Ln, Rn, Vn, Wn);
 
		// k3 = F(t + dt/2, State_2)
		glm::vec3 k3_X = Vn;
		glm::quat k3_Q = 0.5f * (glm::quat(0.0f, Wn) * Qn);
		glm::vec3 k3_P = Force(t + halfdt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);
		glm::vec3 k3_L = Torque(t + halfdt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);

		// State_3 = State_0 + dt * k3
		Xn = Position + dt * k3_X;
		Qn = OrientationQuaternion + dt * k3_Q;
		Pn = LinearMomentum + dt * k3_P;
		Ln = AngularMomentum + dt * k3_L;
		ComputeSecondaryState(Xn, Qn, Pn, Ln, Rn, Vn, Wn);

		// k4 = F(t + dt, State_3)
		glm::vec3 k4_X = Vn;
		glm::quat k4_Q = 0.5f * (glm::quat(0.0f, Wn) * Qn);
		glm::vec3 k4_P = Force(t + dt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);
		glm::vec3 k4_L = Torque(t + dt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);

		// Solution = State_0 + dt * ((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4); 
		Position = Position + sixthdt * (k1_X + 2.0f * (k2_X + k3_X) + k4_X);
		OrientationQuaternion = OrientationQuaternion + sixthdt * (k1_Q + 2.0f * (k2_Q + k3_Q) + k4_Q);
		LinearMomentum = LinearMomentum + sixthdt * (k1_P + 2.0f * (k2_P + k3_P) + k4_P);
		AngularMomentum = AngularMomentum + sixthdt * (k1_L + 2.0f * (k2_L + k3_L) + k4_L);
		glm::normalize(OrientationQuaternion);

		ComputeSecondaryState(Position, OrientationQuaternion, LinearMomentum, AngularMomentum, 
			OrientationMatrix, LinearVelocity, AngularVelocity);
	*/	
		ComputeModelMatrix();

		// Update exteranl force and torque
		//ExternalForce = Force(t + dt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);
		//ExternalTorque = Torque(t + dt, Mass, Xn, Qn, Pn, Ln, Rn, Vn, Wn);
	}


	void RigidBody::ComputeSecondaryState(
		const glm::vec3& Position,
		const glm::quat& OrientationQuaternion,
		const glm::vec3& LinearMomentum,
		const glm::vec3& AngularMomentum,
		glm::mat3& OrientationMatrix,
		glm::vec3& LinearVelocity,
		glm::vec3& AngularVelocity)
	{
		OrientationMatrix = glm::toMat4(OrientationQuaternion);
		LinearVelocity = InvertedMass * LinearMomentum;
		AngularVelocity = OrientationMatrix * InvertedIntertiaTensor * glm::transpose(OrientationMatrix) * AngularMomentum;
	}


	void RigidBody::GetState(glm::vec3& position, glm::quat& orientation, glm::vec3& linearMomentum, glm::vec3& angularMomentum)
	{
		position = Position;
		orientation = OrientationQuaternion;
		linearMomentum = LinearMomentum;
		angularMomentum = AngularMomentum;
	}


	void RigidBody::SetState(const glm::vec3& position, const glm::quat& orientation, 
		const glm::vec3& linearMomentum, const glm::vec3& angularMomentum)
	{
		Position = position;
		OrientationQuaternion = orientation;
		LinearMomentum = linearMomentum;
		AngularMomentum = angularMomentum;

		ComputeSecondaryState(Position, OrientationQuaternion, LinearMomentum, AngularMomentum,
			OrientationMatrix, LinearVelocity, AngularVelocity);

		ComputeModelMatrix();
	}


	void RigidBody::SetForceFunction(ForceFunction force) 
	{
		Force = force;
		ExternalForce = Force(0, Mass, Position, OrientationQuaternion, LinearMomentum, AngularMomentum, OrientationMatrix, LinearVelocity, AngularVelocity);
	}


	void RigidBody::SetTorqueFunction(TorqueFunction torque) 
	{
		Torque = torque;
		ExternalTorque = Torque(0, Mass, Position, OrientationQuaternion, LinearMomentum, AngularMomentum, OrientationMatrix, LinearVelocity, AngularVelocity);
	}

}
