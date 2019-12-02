#include "RigidBody.h"

#include <iostream> 
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <map>
#include <set>
#include <unordered_set>

using namespace PhysicEngine;


RigidBody::RigidBody(std::vector<glm::vec3>& vertices, std::vector<unsigned int>& triangles)
	: Vertices(vertices)
{

	// Stores all the faces grouped by their normals.
	// Due the 3D mesh is convex all the face that have the same normal are adjacent and could be merged 
	// into a single face.

	// Comparator for the normals multimap
	auto vec3Hash = [](const glm::vec3& hs) {
		return std::hash<int>()(hs.x) ^ std::hash<int>()(hs.y) ^ std::hash<int>()(hs.y);
	};

	auto vec3KeyEqual = [](const glm::vec3& lhs, const glm::vec3& rhs) {
		return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
	};

	std::unordered_map<glm::vec3, std::vector<Edge>, decltype(vec3Hash), decltype(vec3KeyEqual)> normalFaceMap(1, vec3Hash, vec3KeyEqual);
	for (unsigned int j = 0; j < triangles.size(); j += 3) {

		unsigned int i0 = triangles[j];
		unsigned int i1 = triangles[j + 1];
		unsigned int i2 = triangles[j + 2];

		glm::vec3 normal = glm::normalize(glm::cross(vertices[i1] - vertices[i0], vertices[i2] - vertices[i1]));
		std::vector<Edge> edges = { { i0, i1 }, { i1, i2 }, { i2, i0 } };

		if (normalFaceMap.find(normal) == normalFaceMap.end())
		{
			normalFaceMap[normal] = edges;
		}
		else
		{
			for (Edge e0 : edges)
			{
				// Removing edges that appears twice (internal face edges) ...
				std::vector<Edge>& newFaceEdges = normalFaceMap[normal];
				if (std::count_if(newFaceEdges.begin(), newFaceEdges.end(), [&e0](Edge e1)
					{ return (e0.x == e1.y && e0.y == e1.x); }) == 1)
				{
					newFaceEdges.erase(std::remove(newFaceEdges.begin(), newFaceEdges.end(), Edge{ e0.y, e0.x }), newFaceEdges.end());
				}

				// ... add the others
				else
				{
					newFaceEdges.push_back(e0);
				}
			}
		}
	}

	// Sort all edges so that the first vertex of a follower is the last vertex of the previous	
	for (auto kv : normalFaceMap)
	{
		Face face;
		std::vector<Edge> faceEdges;

		faceEdges.push_back(kv.second.back());
		Edges.push_back((kv.second.back()));
		face.vertexIds.push_back(kv.second.back().x);
		kv.second.pop_back();

		for (Edge e0 = faceEdges.back(); !kv.second.empty(); e0 = faceEdges.back())
		{
			auto e = std::find_if(kv.second.begin(), kv.second.end(),
				[&e0](Edge e1)
				{
					return e0.y == e1.x;
				}
			);

			faceEdges.push_back(*e);
			Edges.push_back((*e));
			face.vertexIds.push_back((*e).x);
			kv.second.erase(e);
		}

		//Faces.push_back(face);
	}

	// Remove duplicate edges
	auto EdgeHash = [](const Edge& hs) {
		return std::hash<int>()(hs.x) ^ std::hash<int>()(hs.y);
	};
	auto EdgeEqual = [](const Edge& lhs, const Edge& rhs) {
		return (lhs.x == rhs.x && lhs.y == rhs.y) || (lhs.x == rhs.y && lhs.y == rhs.x);
	};
	std::unordered_set<Edge, decltype(EdgeHash), decltype(EdgeEqual) > et(1, EdgeHash, EdgeEqual);
	for (Edge e : Edges) et.insert(e);
	Edges.clear();
	std::copy(et.begin(), et.end(), std::back_inserter(Edges));

	// Save faces from triangles array
	for (int i = 0; i < triangles.size(); i += 3) {

		Face f = { { triangles[i], triangles[i + 1], triangles[i + 2]}, {} };

		// Compute face normal
		glm::vec3 e0 = vertices[f.vId[1]] - vertices[f.vId[0]];
		glm::vec3 e1 = vertices[f.vId[2]] - vertices[f.vId[0]];
		f.n = glm::normalize(glm::cross(e0, e1));

		Faces.push_back(f);
	}

	ComputeCenterOfMassAndInertiaTensor();
	ComputeOrientedBoundingBox();

	SetForceFunction(NullForce);
	SetTorqueFunction(NullTorque);

	// The object is initially still and unrotated
	SetState(glm::vec3(0.0f, 0.0f, 0.0f), glm::identity<glm::quat>(), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
}


PhysicEngine::RenderPolyhedronData RigidBody::GetRenderPolyhedronData()
{
	PhysicEngine::RenderPolyhedronData data;

	for (Face f : Faces)
	{
		data.vertices.push_back(RenderVertex{ Vertices[f.vId[0]], f.n });
		data.elements.push_back(data.vertices.size() - 1);

		data.vertices.push_back(RenderVertex{ Vertices[f.vId[1]], f.n });
		data.elements.push_back(data.vertices.size() - 1);

		data.vertices.push_back(RenderVertex{ Vertices[f.vId[2]], f.n });
		data.elements.push_back(data.vertices.size() - 1);
	}

	return data;
}


void RigidBody::ComputeCenterOfMassAndInertiaTensor() {

	float integral[10] = { 0,0,0,0,0,0,0,0,0,0 };

	for (Face face : Faces) {

		// Get the verticies of this triangle
		Vertex v0 = Vertices[face.vId[0]];
		Vertex v1 = Vertices[face.vId[1]];
		Vertex v2 = Vertices[face.vId[2]];

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

	for (Vertex v : Vertices) {
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
			for (Vertex v : Vertices) {
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
	for (Vertex v : Vertices) {
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
	float halfdt = dt / 2.0f;
	float sixthdt = dt / 6.0f;

	glm::vec3 Xn, Pn, Ln, Vn, Wn;
	glm::quat Qn;
	glm::mat3 Rn;

	// k1 = F(t, State_0) 
	glm::vec3 k1_X = LinearVelocity;
	glm::quat k1_Q = 0.5f * (glm::quat(0.0f, AngularVelocity) * OrientationQuaternion);
	glm::vec3 k1_P = Force(t, Mass, Position, OrientationQuaternion, LinearMomentum, AngularMomentum, OrientationMatrix, LinearVelocity, AngularVelocity);
	glm::vec3 k1_L = Torque(t, Mass, Position, OrientationQuaternion, LinearMomentum, AngularMomentum, OrientationMatrix, LinearVelocity, AngularVelocity);

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
	
	ComputeModelMatrix();
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
}


void RigidBody::SetTorqueFunction(TorqueFunction torque) 
{
	Torque = torque;
}



