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

#include "constants.h"
#include "custom_map.h"
#include "DataTypes.h"
#include "force.h"
#include "Edge.h"



namespace PhysicEngine
{

	/**
	 * A convex rigid 3D object
	 *
	 * This class computes and stores all geometric and physics data relative to a 3D rigid body
	 */
	class RigidBody
	{

	public:

		/** GEOMETRIC PROPERTIES */
		std::vector<Vertex> Vertices;		/**< List of verticies */
		std::vector<Edge> Edges;			/**< List of edges */
		std::vector<Face> Faces;			/**< List of faces */

		/** PHYSIC PROPERTIES */
		glm::vec3 CenterOfMass;				/**< Center of mass */
		float Mass;							/**< Mass */
		float InvertedMass;					/**< Inverted mass = 1/Mass */
		glm::mat3 IntertiaTensor;			/**< Intertia tensor */
		glm::mat3 InvertedIntertiaTensor;	/**< Inverted inertia tensor = IntertiaTensor^-1 */

		/* The PRIMARY STATE is made from all the variables that are necessary to define the physic object 
		   state at the current simulation time */
		glm::vec3 Position;					/**< PRIMARY STATE: position of the object center of mass in wolrd coordinates */
		glm::quat OrientationQuaternion;	/**< PRIMARY STATE: a quaternion storing the object orientation */
		glm::vec3 LinearMomentum;			/**< PRIMARY STATE: linear momentum */
		glm::vec3 AngularMomentum;			/**< PRIMARY STATE: angular momentum */

		/* The SECONDARY STATE holds physical quantities derived from the PRIMARY STATE */
		glm::mat3 OrientationMatrix;		/**< SECONDARY STATE: a matrix storing the object orientation */
		glm::vec3 LinearVelocity;			/**< SECONDARY STATE: linear velocity */
		glm::vec3 AngularVelocity;			/**< SECONDARY STATE: angular velocity */

		/** Model transformation matrix */
		glm::mat4 ModelMatrix;				/**< Store the object rotation and translation in the world, computed from object SECONDARY STATE */


		/* Forces function acting on the object */
		ForceFunction Force;

		/* Torque function acting on the object */
		TorqueFunction Torque;

		/** The external force applyed to the object */
		glm::vec3 ExternalForce;

		/** The external torque applyed to the object */
		glm::vec3 ExternalTorque;

		/** The internal force applyed to the object */
		glm::vec3 InternalForce;

		/** The external torque applyed to the object */
		glm::vec3 InternalTorque;

		bool IsColliding = false;

		bool IsImmovable = false;

		/** Oriented bounding box that contains this polyhedron */
		BoundingBox boundingBox;


		RigidBody() {}

		/**
		 * Construct a convex polyhedron from a vertex list and a triangle face list.
		 * Note that only triangulated convex polygon are supported,
		 * non-convex objects or objects with faces different from triangles are not supported.
		 *
		 * @param the list of Polyhedron Vertex
		 * @param the list of Polyhedron Edge
		 * @param the list of Polyhedron Face
		 */
		RigidBody(std::vector<glm::vec3>& vertices, std::vector<unsigned int>& triangles);

		/**
		 * Set the force function acting on the rigid body
		 */
		void SetImmovable(bool b)
		{
			this->IsImmovable = b;
			InvertedMass = 0;
			InvertedIntertiaTensor = glm::mat3(0.0f);
		}

		/**
		 * Get the PRIMARY STATE of the rigid body
		 */
		void GetState(glm::vec3& position, glm::quat& orientation, glm::vec3& linearMomentum, glm::vec3& angularMomentum);

		/**
		 * Set the PRIMARY STATE of the rigid body
		 */
		void SetState(const glm::vec3& position, const glm::quat& orientation, const glm::vec3& linearMomentum, const glm::vec3& angularMomentum);

		/**
		 * Set the force function acting on the rigid body
		 */
		void SetForceFunction(ForceFunction force);

		/**
		 * Set the force function acting on the rigid body
		 */
		void SetTorqueFunction(TorqueFunction torque);

		/**
		 * Add an external force
		 */
		void AddInternalForce(glm::vec3 force)
		{
			InternalForce += force;
		}

		/**
		 * Add an external torque
		 */
		void AddInternalTorque(glm::vec3 torque)
		{
			InternalTorque += torque;
		}


		/**
		 * Update the rigid body STATE given.
		 * This method implements a 4th order Runge Kutta method to solve the equations of the motion.
		 */
		void UpdateState(float t, float dt);

		/**
		 * Compute the mass, the center of mass and the inertia tensor for the polyhedron
		 */
		void ComputeCenterOfMassAndInertiaTensor();

		/**
		 * Compute the oriented bounding box that contains the rigid body
		 */
		void ComputeOrientedBoundingBox();

		/**
		 * Get data suited for the rendering of the rigid body
		 */
		RenderPolyhedronData GetRenderPolyhedronData();
	
	private:

		/**
		 * Helper function used in ComputeCenterOfMassAndIntertiaTensor()
		 */
		void ComputeCenterOfMassAndInertiaTensorSubExpression(
			float& w0, float& w1, float& w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2);

		/**
		 *  Helper function used to compute from a
		 *  PRIMARY STATE { position, orientation quaternion, linear momentum angular momentum } the
		 *  SECONDARY STATE { orientation matrix, linear velocity, angular velocity }
		 */
		void ComputeSecondaryState(const glm::vec3& Position, const glm::quat& OrientationQuaternion,
			const glm::vec3& LinearMomentum, const glm::vec3& AngularMomentum,
			glm::mat3& OrientationMatrix, glm::vec3& LinearVelocity, glm::vec3& AngularVelocity);

		/**
		 * Update the model matrix from the PRIMARY and SECONDARY state
		 */
		void ComputeModelMatrix();
	};
	
}


