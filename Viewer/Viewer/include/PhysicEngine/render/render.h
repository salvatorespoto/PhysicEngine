#pragma once

namespace PhysicEngine 
{

	/**
	 * A 3D vertex 
	 */
	typedef struct 
	{
		glm::vec3 v;		/**< Vertex position */
		glm::vec3 n;		/**< Vertex normal */
	} RenderVertex;


	/** 
	 * A struct that represent a triangulated polyhedron
	 */
	typedef struct 
	{
		std::vector<RenderVertex> Vertices;		/**< The verticies of the polyhedron */
		std::vector<unsigned int> Triangles;	/**< The polyhedron triangles */
	} RenderPolyhedron;

}