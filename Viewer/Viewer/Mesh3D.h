#ifndef MESH_3D_H
#define MESH_3D_H

#include <boost/log/trivial.hpp>
#include <boost/unordered_map.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "include/glad/glad.h"
#include "include/tini_obj_loader/tini_obj_loader.h"

#include "Vertex3D.h"

#include "include/PhysicEngine/DataTypes.h"


/** A struct containing info and data for vertex array object, vertex buffer and element buffer */
typedef struct VAOAttributes {
	GLuint VaoId;
	GLuint VboId;
	GLuint EboId;
	std::vector<Vertex3D> vertices;
	std::vector<unsigned int> elements;
} VAOAttributes;


/** A mesh 3D mesh */
class Mesh3D 
{

public:

	/** Show / hide bounding box*/
	bool RenderBoundingBox = false;

	/** Show / hide convex hull */
	bool RenderConvexHull = false;

	/** Show / hide model */
	bool RenderModel = true;

	/** The mesh model transfrom matrix */
	glm::mat4 ModelMatrix = glm::mat4(1.0f);

	/** Convex hull used from the physic engine */
	PhysicEngine::ConvexPolyhedron* physicConvexHull;

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

	/** Get the model transformation matrix */
	glm::mat4 GetModelMatrix();

	/** Draw the mesh */
	void Draw(GLuint shaderProgramId);

private:
	
	/** Rendering data for the model mesh */
	VAOAttributes modelVAO;

	/** Rendering data for the hull mesh */
	VAOAttributes hullVAO;

	/** Rendering data for bounding box */
	VAOAttributes boundingBoxVAO;

	/** Rendering data for the center of mass */
	VAOAttributes centerOfMassVAO;

	/** Setup mesh for rendering */
	void SetupRender();

	/** 
	 * Build a vertex and and index attributes for a mesh 
	 * 
	 * Build a vertex and and index attributes for a mesh taking into accounts the info included in the vertex, 
	 * E.g. a mesh which vertex includes inforamtion about position, normals and textcoords will have different
	 * vertices and index arrays from the one including onlly position and normals.
	 *
	 * @param mesh A mesh object loaded with tinyobj library
	 * @param attrib The attrib objet associated to the mesh, loaded with tinyobj library
	 * @param position If true, the vertex will include position info 
	 * @param normal If true, the vertex will include normal info 
	 * @param color If true, the vertex will include color info 
	 * @param textCoord If true, the vertex will include texture coordinate info 
	 * @param outVertices The output vertex array
	 * @param outIndices Tehe output index array
	 */
	void LoadMesh(const tinyobj::mesh_t& mesh, const tinyobj::attrib_t& attrib,
		std::vector<Vertex3D>& outVertices, std::vector<unsigned int>& outIndices,
		const bool normal, const bool color, const bool textCooord);

	/**
	 * Build the physic conves hull 
	 *
	 * @param mesh A mesh object loaded with tinyobj library
	 * @param attrib The attrib objet associated to the mesh, loaded with tinyobj library
	 */
	void LoadConvexHull(const tinyobj::mesh_t& mesh, const tinyobj::attrib_t& attrib);
};

#endif
