#pragma once

#include <vector>

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include "PhysicEngine/External/Eigen/Eigenvalues"

#include "PhysicEngine/DataTypes.h"
#include "PhysicEngine/geometry/intersection.h"
#include "PhysicEngine/physics/RigidBody.h"
#include "PhysicEngine/force.h"
#include "PhysicEngine/Memory.h"
#include "PhysicEngine/QuadraticSolver.h"



namespace PhysicEngine
{

	const float RESTITUTION_COEFFICIENT = 0.1f;		/**< Coefficient of restitution in colliding contact */


	/**
	 * Test if two rigid body collide
	 *
	 * Test if two rigid body collide between currentTime and currentTime + timeStep and return the contact set
	 * between them.
	 * If the two objects do not collide an empty Contact vector is .
	 * 
	 * @param currentTime Current simulation time
	 * @param timeStep The simulation timestep
	 * @param rbA The first RigidBody
	 * @param rbB The second RigidBody
	 *
	 * @return A Contact vector holding the contact set between the two objects
	 */
	std::vector<Contact> ComputeRigidBodyContact(
		const float currentTime, 
		const float timeStep, 
		RigidBody& rbA, 
		RigidBody& rbB
	);


	/**
	 *
	 * @return True if the two body intersect
	 */
	bool CheckRigidBodyCollision(
		float tMax, 
		const float& speed, 
		AxisProjection& projectionInfo0, 
		AxisProjection& projectionInfo1,
		AxisProjection& currProj0, 
		AxisProjection& currProj1, 
		int& side, 
		float& tFirst, 
		float& tLast
	);
	

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
	void DoCollisionResponse(
		float t, 
		float dt, 
		const std::vector<Contact>& contacts, 
		const std::vector<RigidBody*>& bodies);


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
	void Minimize(
		int N, 
		float** M, 
		const std::vector<float>& preContactVelocities, 
		std::vector<float>& outPostContactVelocities, 
		float* impulsesMagnitude
	);



	/**
	 * Process a colliding contact bewtween two RigidBody
	 *
	 * @param	rbA				first RigidBody involved in the collision
	 * @param	rbB				second RigidBody in the collision
	 * @param   collidingPoint	the point of collision between two bodies
	 * @param	normal			the collding face normal
	 */
	void ProcessCollidingContact(
		RigidBody& rbA, 
		RigidBody& rbB, 
		const glm::vec3& collidingPoint, 
		const glm::vec3& normal
	);

	void DoMotion(float t, float dt, std::vector<Contact> contacts, float* restingMagnitute, std::vector<RigidBody*> bodies);
	
	void DoImpulse(std::vector<Contact> contacts, float* impulsesMagnitude);
	void ComputingRestingContactVector(std::vector<Contact> contacts, float* b);
	void ComputeLCPMatrix(std::vector<Contact> contacts, float** M, int N);

}