#pragma once

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
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "include/glad/glad.h"
#include "include/tini_obj_loader/tini_obj_loader.h"

#include "Vertex3D.h"

#include "include/PhysicEngine/DataTypes.h"
#include "include/PhysicEngine/Physics/RigidBody.h"


/** 
 * A struct containing info and data for vertex array object, vertex buffer and element buffer 
 */
typedef struct VAOAttributes {
	GLuint VaoId;
	GLuint VboId;
	GLuint EboId;
	std::vector<Vertex3D> vertices;
	std::vector<unsigned int> elements;
} VAOAttributes;


/** 
 * A 3D mesh 
 */
class Mesh3D 
{

public:

	bool RenderBoundingBox = false;			/** Show / hide bounding box*/
	bool RenderConvexHull = false;			/** Show / hide convex hull */
	bool RenderModel = true;				/** Show / hide model */

	glm::mat4 ScaleMatrix = glm::mat4(1.0f);
	glm::mat4 RotationMatrix = glm::mat4(1.0f);
	glm::mat4 TranslationMatrix = glm::mat4(1.0f);

	PhysicEngine::RigidBody* PhysicRigidBody;

	/** 
	 * Load a mesh from an .obj file 
	 *
	 * @param meshFilePath the file path of the mesh to load
	 */
	Mesh3D(const boost::filesystem::path& objFilePath);

	/** 
	 * Load a mesh from an .obj 
	 *
	 * Load the mesh and its convex hull from .obj file
	 * The supported file format is an .obj file with two shapes: the first has a name that starts with "Model" and
	 * is the 3D model mesh, whale the second it's the mesh convex hull and its name starts with "Hull"
	 * @param meshFilePath the file path of the mesh to load
	 */
	void LoadFromObjFile(const boost::filesystem::path& objFilePath);

	/** 
	 * Get the model transformation matrix 
	 */
	glm::mat4 GetModelMatrix();

	/** 
	 *
	 */
	void SetModelMatrix(glm::mat4 M) 
	{
		glm::vec3 scale;
		glm::quat rotation;
		glm::vec3 translation;
		glm::vec3 skew;
		glm::vec4 perspective;
		glm::decompose(M, scale, rotation, translation, skew, perspective);
		RotationMatrix = glm::toMat4(rotation);
		TranslationMatrix = glm::translate(glm::mat4(1.0f), translation);
	}
	
	/**
	 *
	 */
	void Translate(glm::vec3 translation) 
	{
		TranslationMatrix = glm::translate(TranslationMatrix, translation);
	}

	/**
	 *
	 */
	void Rotate(float angle, glm::vec3 axis)
	{
		RotationMatrix = glm::rotate(RotationMatrix, angle, axis);
	}

	/** 
	 * Draw the mesh 
	 */
	void Render(GLuint shaderProgramId);

	struct LoadMeshException : public std::exception
	{
		LoadMeshException(const char* msg) : std::exception(msg) {}
	};


private:
	
	VAOAttributes modelVAO;				/** Rendering data for the model mesh */
	VAOAttributes hullVAO;				/** Rendering data for the hull mesh */
	VAOAttributes boundingBoxVAO;		/** Rendering data for bounding box */
	VAOAttributes centerOfMassVAO;		/** Rendering data for the center of mass */
	

	/** 
	 * Setup mesh for rendering 
	 */
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
	void LoadMesh(
		const tinyobj::mesh_t& mesh, 
		const tinyobj::attrib_t& attrib,
		std::vector<Vertex3D>& outVertices, 
		std::vector<unsigned int>& outIndices,
		const bool normal, 
		const bool color, 
		const bool textCooord
	);

	/**
	 * Build the physic conves hull 
	 *
	 * @param mesh A mesh object loaded with tinyobj library
	 * @param attrib The attrib objet associated to the mesh, loaded with tinyobj library
	 */
	void LoadConvexHull(
		const tinyobj::mesh_t& mesh, 
		const tinyobj::attrib_t& attrib
	);
};