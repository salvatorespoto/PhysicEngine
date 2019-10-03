#ifndef DATA_TYPES_H
#define DATA_TYPES_h

#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>

#include "External/Eigen/Eigenvalues"

namespace PhysicEngine
{

	/** A vertex with additional inforation, suitable for rendering */
	typedef struct {
		glm::vec3 position;
		glm::vec3 normal;
	} RenderVertex;

	/** A struct containing information to render a whole physic object */
	typedef struct {
		std::vector<RenderVertex> vertices;
		std::vector<unsigned int> elements;
	} RenderPolyhedronData;


	/** 3d object vertex */
	typedef glm::vec3 Vertex;


	/**
	 * A 3d object triangular face
	 */
	typedef struct Face {
		glm::ivec3 vId;
		glm::vec3 n;
	} Face;


	/**
	 * A 3d bounding box. Faces are quads.
	 */
	typedef struct BoundingBox {
		glm::vec3 vertices[8];
		unsigned int edges[32];
	} BoundingBox;


	/**
	 * A Convex 3D object
	 */
	class ConvexPolyhedron
	{

	private:

		const float oneDiv6 = 1 / 6;
		const float oneDiv24 = 1 / 24;
		const float oneDiv60 = 1 / 60;
		const float oneDiv120 = 1 / 120;

		/**
		 * Helper function used in ComputeCenterOfMassAndIntertiaTensor()
		 */
		void SubExpression(float& w0, float& w1, float& w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2);

	public:

		std::vector<Vertex> Vertices;
		std::vector<Face> Faces;

		float mass;
		glm::vec3 centerOfMass;
		glm::mat3 intertiaTensor;

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
		ConvexPolyhedron(std::vector<glm::vec3>& vertices, std::vector<unsigned int>& triangles);


		/** Compute the mass, the center of mass and the inertia tensor for the polyhedron */
		void ComputeCenterOfMassAndIntertiaTensor();


		/** Compute the oriented bounding box */
		void ComputeOrientedBoundingBox();

		/** */
		PhysicEngine::RenderPolyhedronData GetRenderPolyhedronData();
	};
}

#endif

