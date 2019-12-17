#pragma once

#include <vector>

#include "DataTypes.h"
#include "Functions.h"
#include "RigidBody.h"
#include "collision.h"


/**
 * The physics engine namespace
 *
 * All the Physics engine classes are defined inside this namespace
 */
namespace PhysicEngine
{

	/**
	 * The physics engine main class
	 *
	 * This is the physics engine main class, it holds the list of all the objects, 
	 * computes the effects of forces on objects, collisions detection,
	 * collision responce and update the new object position accordingly.
	 */
	class RigidBodyEngine
	{

	public:

		/**
		 * Execute the physics tick.
		 *
		 * This function execute the physics tick:
		 * 1. apply extenal forces
		 * 2. do collision detection
		 * 3. compute collision response
		 * 4. update the objects accordingly
		 */
		void Tick();

		/** 
		 * Set the simulation starting time
		 *
		 * @param t The simulation starting time
		 */
		void SetStartingTime(float t);

		/**
		 * Set the simulation step
		 *
		 * At each tick the simulation is updated of a time step dt
		 *
		 * @param dt The time step
		 */
		void SetTimeStep(float dt);

		/**
		 * Add a rigid body to the scene
		 *
		 * @param rb A pointer to a RigidBody object
		 */
		void AddRigidBody(RigidBody* rb);

		/**
		 * Return a vector of RigidBody objects in the scene
		 *
		 * @return A vector of RigidBody pointers to the objects in the scene
		 */
		const std::vector<RigidBody*>& GetRigidBodies();

		/**
		 * Return a vector of the Contacts among objects in the scene
		 *
		 * @return A vector of Contacts, these are the contact among all objects in the scene
		 */
		const std::vector<Contact>& GetContacts();


	private: 

		float currentTime = 0.0f;							/**< Current simulation time */
		const float DEFAULT_TIME_STEP = 1.0f / 60.0f;		/**< Default simulator frequency is 60HZ*/
		float timeStep = DEFAULT_TIME_STEP;					/**< The simulation time step */

		std::vector<RigidBody*> Objects; 					/**< List of Rigid Body objects in the scene */
		std::vector<Contact> Contacts;						/**< Current detected contacts among objects */
		 
		/**
		 * Detect collsion among objects in the scene
		 *
		 * This function test all object for collisions and compute the contact sets of 
		 * colliding object, filling the Contacts vector property.
		 * The contact list is resetted at each tick.
		 */
		void DetectCollisions();

		/**
		 * Compute all collisions responses among objects 
		 *
		 * This function compute all resulting forces from objects interactions, 
		 * both colliding contacts and resting contacts.
		 */
		void ProcessCollisions();

		/** 
		 * Update the objects state 
		 *
		 * This function compute the updated state after a time "timeStep" from the previous
		 * tick. 
		 * The updated state is the result of the collision processed in ProcessCollision and
		 * the extenaral enviroment forces (e.g. gravity). 
		 */
		void Update();
	};
}