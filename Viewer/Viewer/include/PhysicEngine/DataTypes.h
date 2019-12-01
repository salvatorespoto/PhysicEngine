#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include<glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include "External/Eigen/Eigenvalues"

using glm::vec3;


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
	 * /
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
	*/
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

}

#endif

