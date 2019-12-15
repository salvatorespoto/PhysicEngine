#pragma once

#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include "External/Eigen/Eigenvalues"

#include "DataTypes.h"
#include "RigidBody.h"
#include "force.h"
#include "Memory.h"
#include "QuadraticSolver.h"



namespace PhysicEngine
{

	/**
	 * Coefficient of restitution in colliding contact
	 */
	const float RESTITUTION_COEFFICIENT = 0.1f;


	/**
	 * A contact between two RigidBody
	 */
	class Contact
	{
	
	public:	
	
		enum class Type { VERTEX_FACE = 0, EDGE_EDGE };	// Possible contacts types
													// If the type is VERTEX_FACE, there is the assumption that the vertex is from RigidBody A 
													// and the face is from the RigidBody B

		Type type = Type::VERTEX_FACE;									// The type of this contact
		RigidBody& rb0;								// First RigidBody involved in contact
		RigidBody& rb1;								// Second RigidBody involved in contact
		glm::vec3 point = glm::vec3(0);							// The contact point between the two RigidBody
		glm::vec3 normal = glm::vec3(0);							// If it is a VERTEX_FACE contact, holds the normal to the face
		glm::vec3 edgeA[2] = { glm::vec3(0), glm::vec3(0) };							// If it is an EDGE_EDGE contact the edge from the RigidBody A, stores the two
													// edge vertices
		glm::vec3 edgeB[2] = { glm::vec3(0), glm::vec3(0) };							// If it is an EDGE_EDGE contact the edge from the RigidBody B, stores the two
													// edge vertices

		Contact(Contact::Type type, RigidBody& rb0, RigidBody& rb1, glm::vec3 point, glm::vec3 normal)
			: type(type), rb0(rb0), rb1(rb1), point(point), normal(normal) {};

		Contact(Contact::Type type, RigidBody& rb0, RigidBody& rb1, glm::vec3 point, glm::vec3 normal, glm::vec3 edgeA[2], glm::vec3 edgeB[2]) 
			: type(type), rb0(rb0), rb1(rb1), point(point), normal(normal), edgeA{ edgeA[0], edgeA[1]}, edgeB{ edgeB[0], edgeB[1] } {}

		bool operator ==(Contact c)
		{
			return point == c.point;
		}

		bool operator !=(Contact c)
		{
			return point != c.point;
		};
	};


	/**
	 * Compute collision response

	 * Compute the collision response taking into account both the colliding and the resting contact for the
	 * rigid bodies in the scene, given a contact set detected in the previous collision detection phase.
	 *
	 * @param t	The current simulation time
	 * @param dt The time step
	 * @param contacts The list of contacts detected in the collision detection phase
	 * @param bodies The list of RigidBodies in the scene
	 */
	void DoCollisionResponse(float t, float dt, const std::vector<Contact>& contacts, const std::vector<RigidBody*>& bodies);


	/** 
	 * Compute relative pre contact velocities
	 *
	 * Compute the objects contact points relative velocities of the for each pair involved in a contact
	 *
	 * @param contacts The contacts vector detected in the previous collision detection phase
	 * @return A vector of relative velocities between objects for each object pairs involved in a contact
	 */
	std::vector<float> ComputePreContactVelocities(const std::vector<Contact>& contacts);

	/**
	 * Compute 	 *
	 * Compute 
	 *
	 * @param 
	 * @return 
	 */
	void Minimize(int N, float** M, const std::vector<float>& preContactVelocities, std::vector<float>& outPostContactVelocities, float* impulsesMagnitude);



	/**
	 * Process a colliding contact bewtween two RigidBody
	 *
	 * @param	rbA				first RigidBody involved in the collision
	 * @param	rbB				second RigidBody in the collision
	 * @param   collidingPoint	the point of collision between two bodies
	 * @param	normal			the collding face normal
	 */
	void ProcessCollidingContact(RigidBody& rbA, RigidBody& rbB, const glm::vec3& collidingPoint, const glm::vec3& normal);

	void DoMotion(float t, float dt, std::vector<Contact> contacts, float* restingMagnitute, std::vector<RigidBody*> bodies);
	
	void DoImpulse(std::vector<Contact> contacts, float* impulsesMagnitude);
	void ComputingRestingContactVector(std::vector<Contact> contacts, float* b);
	void ComputeLCPMatrix(std::vector<Contact> contacts, float** M, int N);

}