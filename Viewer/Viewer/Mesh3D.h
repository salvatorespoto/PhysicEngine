#ifndef MESH_3D_H
#define MESH_3D_H

#include <boost/log/trivial.hpp>
#include <boost/unordered_map.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "include/glad/glad.h"

#include "Vertex3D.h"



/** A mesh 3D mesh */
class Mesh3D 
{

public:

	/** Load mesh Exception */
	struct LoadMeshException : public std::exception
	{
		LoadMeshException(const char* msg) : std::exception(msg) {}
	};

	/** 
	 * Load a mesh from an .obj file 
	 *
	 * @param meshFilePath the file path of the mesh to load
	 */
	Mesh3D(const boost::filesystem::path& objFilePath);

	/** 
	 * Load a mesh from an .obj 
	 *
	 * @param meshFilePath the file path of the mesh to load
	 */
	void LoadFromObjFile(const boost::filesystem::path& objFilePath);


	/** Draw the mesh */
	void Draw();

private:

	std::vector<Vertex3D> vertices;
	std::vector<unsigned int> indices;

	/** Vertex Array Object */
	unsigned int VAO;
	
	/** Vertex Buffer Object */
	unsigned int VBO;

	/** Element Buffer Object */
	unsigned int EBO;

	/** Setup mesh for rendering */
	void SetupRender();
};

#endif
