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

		/** Helper function used in ComputeCenterOfMassAndIntertiaTensor() */
		void ComputeCenterOfMassAndInertiaTensorSubExpression(float& w0, float& w1, float& w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2);


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

		/* The STATE properties */
		glm::vec3 Position;
		glm::quat Orientation;
		glm::vec3 LinearMomentum;
		glm::vec3 AngularMomentum;

		/* Quatities derived from state */
		glm::mat4 OrientationMatrix;
		glm::vec3 LinearVelocity;
		glm::vec3 AngularVelocity;



		bool IsColliding = false;

		/** Model transformation matrix */
		glm::mat4 ModelMatrix;

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

		/** Get the state of the rigid body */
		void GetState(glm::vec3 x, glm::quat Q, glm::vec3 P, glm::vec3 L);

		/** Set the state of the rigid body */
		void SetState(glm::vec3 x, glm::quat Q, glm::vec3 P, glm::vec3 L);

		/** Compute the mass, the center of mass and the inertia tensor for the polyhedron */
		void ComputeCenterOfMassAndInertiaTensor();

		/** Compute the oriented bounding box */
		void ComputeOrientedBoundingBox();

		/** Get data suited for polyhedron rendering */
		RenderPolyhedronData GetRenderPolyhedronData();
	};
}

#endif

