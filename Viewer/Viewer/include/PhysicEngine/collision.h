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


/**
 * Coefficient of restitution in colliding contact
 */
const float RESTITUTION_COEFFICIENT = 0.8f;


/**
 * A contact between two RigidBody
 * /
typedef struct
{
	enum Type { VERTEX_FACE = 0, EDGE_EDGE };	// Possible contacts types
												// If the type is VERTEX_FACE, there is the assumption that the vertex is from RigidBody A 
												// and the face is from the RigidBody B

	Type type;									// The type of this contact
	RigidBody A;								// First RigidBody involved in contact
	RigidBody B;								// Second RigidBody involved in contact
	glm::vec3 point;							// The contact point between the two RigidBody
	glm::vec3 normal;							// If it is a VERTEX_FACE contact, holds the normal to the face
	glm::vec3 edgeA[2];							// If it is an EDGE_EDGE contact the edge from the RigidBody A, stores the two
												// edge vertices
	glm::vec3 edgeB[2];							// If it is an EDGE_EDGE contact the edge from the RigidBody B, stores the two
												// edge vertices
} Contact;
*/

/** 
 * Process a colliding contact bewtween two RigidBody
 * 
 * @param	rbA				first RigidBody involved in the collision 
 * @param	rbB				second RigidBody in the collision
 * @param   collidingPoint	the point of collision between two bodies
 * @param	normal			the collding face normal 
 */
void ProcessCollidingContact(RigidBody& rbA, RigidBody& rbB, const glm::vec3& collidingPoint, const glm::vec3& normal);