#ifndef DATA_TYPES_H
#define DATA_TYPES_H

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

	/** 3d object edge (edge's indexes in the vertex array) */
	typedef glm::ivec2 Edge;

	/** A face */
	typedef struct {
		glm::ivec3 vId;
		glm::vec3 n;
		std::vector<unsigned int> vertexIds;
	} Face;

	/** 
	 * A contact between two objects. 
	 * All the contact that could occur between two objects are resolved to two types of contact:
	 *	VERTEX_FACE_CONTACT and EDGE_EDGE_CONTACT.
	 */
	typedef struct Contact {
		enum Type { VERTEX_FACE_CONTACT = 0, EDGE_EDGE_CONTACT };	// Type of contact 
		Type type;				// Type of contact
		//ConvexPolyhedron C0;	// Body containing vertex
		//ConvexPolyhedron C1;	// Body containing face
		glm::vec3 Point;		// Contact point
		glm::vec3 Normal;		// Outward versor normal to the face 
		glm::vec3 EdgeC0;		// The direction of the edge from C0 object
		glm::vec3 EdgeC1;		// The direction of the edge from C1 object
		glm::vec3 E00;		
		glm::vec3 E01;
		glm::vec3 E10;
		glm::vec3 E11;
	} Contact;

	/** A 3d bounding box. Faces are quads */
	typedef struct BoundingBox {
		glm::vec3 center;			// Untrasformed center
		glm::vec3 extension;		// Half a side 
		glm::mat4 M;				// Transformation matrix
		glm::vec3 vertices[8];		// Trasformed vertices of the bounding box 
		unsigned int edges[24];		// Index list of the edges of the bounding box
	} BoundingBox;


	/* The contact set could be a vector, one or more edges or one or more faces */
	typedef struct ContactSet {
		enum Type { VERTEX = 1, EDGE, FACE };	// Type of contact set
		Type type;								
		std::vector<int> vertices;				// If the type is VERTEX, the edges list will be stored here
		std::vector<Edge> edges;				// If the type is EDGE, the edges list will be stored here
		std::vector<Face> faces;				// If the type is FACE, the edges list will be stored here
	} ContactSet;	


	/* The contact set could be a vector, one or more edges or one or more faces */
	typedef struct Object3D {
		enum Type { EMPTY = 0, VERTEX, EDGE, FACE };	// Type of polygon
		Type type;								
		std::vector<Vertex> vertices;			// If the type is VERTEX, the edges list will be stored here
		std::vector<Edge> edges;				// If the type is EDGE, the edges list will be stored here
		std::vector<Face> faces;				// If the type is FACE, the edges list will be stored here
	} Object3D;


	/** The contact set between two convex polyhedrons */
	typedef struct {
		double min, max;						// [max, min] interval of the projection
		ContactSet sMin, sMax;
	} ProjectionInfo;

	
	/** A Convex 3D object */
	class ConvexPolyhedron
	{

	private:

		const float oneDiv6 = 1.0f / 6.0f;
		const float oneDiv24 = 1.0f / 24.0f;
		const float oneDiv60 = 1.0f / 60.0f;
		const float oneDiv120 = 1.0f / 120.0f;

		/** Helper function used in ComputeCenterOfMassAndIntertiaTensor() */
		void ComputeCenterOfMassAndInertiaTensorSubExpression(float& w0, float& w1, float& w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2);


	public:

		std::vector<Vertex> Vertices;
		std::vector<Edge> Edges;
		std::vector<Face> Faces;

		float Mass;
		glm::vec3 CenterOfMass;
		glm::mat3 IntertiaTensor;

		glm::vec3 Velocity;

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
		ConvexPolyhedron(std::vector<glm::vec3>& vertices, std::vector<unsigned int>& triangles);

		/** Compute the mass, the center of mass and the inertia tensor for the polyhedron */
		void GetFaceFromEdges(glm::ivec2 edges[3]) {
		
		}

		/** Compute the mass, the center of mass and the inertia tensor for the polyhedron */
		void ComputeCenterOfMassAndInertiaTensor();

		/** Compute the oriented bounding box */
		void ComputeOrientedBoundingBox();

		/** Get data suited for polyhedron rendering */
		RenderPolyhedronData GetRenderPolyhedronData();
	};
}

#endif

