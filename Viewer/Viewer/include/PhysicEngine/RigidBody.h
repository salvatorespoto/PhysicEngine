#ifndef RIGID_BODY_H
#define RIGID_BODY_H

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
#include "force.h"

using namespace PhysicEngine;

namespace PhysicEngine
{

	/** 
	 * A convex rigid 3D object 
	 */
	class RigidBody
	{

	private:

		const float oneDiv6 = 1.0f / 6.0f;
		const float oneDiv24 = 1.0f / 24.0f;
		const float oneDiv60 = 1.0f / 60.0f;
		const float oneDiv120 = 1.0f / 120.0f;

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


	public:

		/* GEOMETRIC properties */
		
		std::vector<Vertex> Vertices;
		std::vector<Edge> Edges;
		std::vector<Face> Faces;


		/* PHYSIC properties */
		
		glm::vec3 CenterOfMass;
		float Mass;
		float InvertedMass;
		glm::mat3 IntertiaTensor;
		glm::mat3 InvertedIntertiaTensor;


		/* The PRIMARY STATE properties */
		
		glm::vec3 Position;
		glm::quat OrientationQuaternion;
		glm::vec3 LinearMomentum;
		glm::vec3 AngularMomentum;
		
		/** Model transformation matrix */
		glm::mat4 ModelMatrix;


		/* The SECONDARY STATE properties*/
		
		glm::mat3 OrientationMatrix;
		glm::vec3 LinearVelocity;
		glm::vec3 AngularVelocity;


		/* Forces acting on the object */
		ForceFunction Force;
		
		/* Torque acting on the object */
		TorqueFunction Torque;

		bool IsColliding = false;

		/** Oriented bounding box that contains this polyhedron */
		BoundingBox boundingBox;


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

		/** */
		void GetFaceFromEdges(glm::ivec2 edges[3]) {}

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
	};
}

#endif

