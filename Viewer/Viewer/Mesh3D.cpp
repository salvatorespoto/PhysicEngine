#include "Mesh3D.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "include/tini_obj_loader/tini_obj_loader.h"

#include <algorithm>    

Mesh3D::Mesh3D(const boost::filesystem::path& objFilePath) 
{	
	LoadFromObjFile(objFilePath);
	SetupRender();
}


void Mesh3D::LoadFromObjFile(const boost::filesystem::path& objFilePath) 
{
	// Load the mesh and its convex hull from .obj file
	// The supported file format is an .obj file with two shapes: the first has a name that starts with "Model" and
	// is the 3D model mesh, whale the second it's the mesh convex hull and its name starts with "Hull"

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn;
	std::string err;

	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, 
		objFilePath.string().c_str(), objFilePath.root_path().string().c_str());
	
	if (!warn.empty()) BOOST_LOG_TRIVIAL(warning) << warn;
	if (!err.empty()) BOOST_LOG_TRIVIAL(error) << err;
	if (!ret) throw new LoadMeshException("Error loading Obj file ");

	// Try to load from the .obj file two shapes: the first is the Model itself, while the second is the object convex hull
	int modelShapeId, hullShapeId;
	modelShapeId = hullShapeId = -1;
	if (boost::starts_with(shapes[0].name, "Model")) modelShapeId = 0;
	else if (boost::starts_with(shapes[1].name, "Model")) modelShapeId = 1;
	if (boost::starts_with(shapes[0].name, "Hull")) hullShapeId = 0;
	else if (boost::starts_with(shapes[1].name, "Hull")) hullShapeId = 1;
	if(modelShapeId == -1 || hullShapeId == -1) 
		throw new LoadMeshException("Error loading Obj file: missing shape 'Model' or 'Hull'");

    // Load the "Model" 
	LoadMesh(shapes[modelShapeId].mesh, attrib, 
		modelVAO.vertices, modelVAO.elements, true, true, true);

	// Load the "Convex hull"
	LoadConvexHull(shapes[hullShapeId].mesh, attrib);

	// The convex hull model matrix is a reference the the mesh model matrix
	PhysicRigidBody->ModelMatrix = glm::mat4(GetModelMatrix());
}


void Mesh3D::LoadMesh(const tinyobj::mesh_t& mesh, const tinyobj::attrib_t& attrib,
	std::vector<Vertex3D>& outVertices, std::vector<unsigned int>& outIndices,
	const bool normal, const bool color, const bool textCooord)
{
	// Load "Model" shape
	boost::unordered_map<Vertex3D, unsigned int> uniqueVertices = {};
	for (const auto& index : mesh.indices) {

		// Parse and save vertex data
		Vertex3D vertex = {};
		vertex.Position = {
			attrib.vertices[3 * index.vertex_index + 0],
			attrib.vertices[3 * index.vertex_index + 1],
			attrib.vertices[3 * index.vertex_index + 2]
		};

		if (normal)
			vertex.Normal = {
				attrib.normals[3 * index.normal_index + 0],
				attrib.normals[3 * index.normal_index + 1],
				attrib.normals[3 * index.normal_index + 2]
		};

		if (color)
			vertex.Color = {
				1.0f,
				1.0f,
				0.0f
		};

		if (textCooord)
			vertex.TexCoords = {
				attrib.texcoords[2 * index.texcoord_index + 0],
				1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
		};

		// Save vertex avoiding duplicates
		if (uniqueVertices.count(vertex) == 0) {
			uniqueVertices[vertex] = static_cast<uint32_t>(outVertices.size());
			outVertices.push_back(vertex);
		}

		// Save vertex index
		outIndices.push_back(uniqueVertices[vertex]);
	}
}


void Mesh3D::LoadConvexHull(const tinyobj::mesh_t& mesh, const tinyobj::attrib_t& attrib)
{
	std::vector<Vertex3D> tempVertices;
	std::vector<unsigned int> tempElements;

	// The physic convex polyhedron only need info about vertex position and face
	LoadMesh(mesh, attrib, tempVertices, tempElements, false, false, false);

	// Convert vertices and elements format 
	std::vector<glm::vec3> vertices;
	for (Vertex3D v : tempVertices)
		vertices.push_back(glm::vec3{ v.Position.x, v.Position.y, v.Position.z });

	PhysicRigidBody = new PhysicEngine::RigidBody(vertices, tempElements);
}


void Mesh3D::SetupRender()
{
	// Setup Model rendering
	glGenVertexArrays(1, &modelVAO.VaoId);
	glGenBuffers(1, &modelVAO.VboId);
	glGenBuffers(1, &modelVAO.EboId);

	glBindVertexArray(modelVAO.VaoId);

		glBindBuffer(GL_ARRAY_BUFFER, modelVAO.VboId);
		glBufferData(GL_ARRAY_BUFFER, modelVAO.vertices.size() * sizeof(Vertex3D), &modelVAO.vertices[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, modelVAO.EboId);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, modelVAO.elements.size() * sizeof(unsigned int), &modelVAO.elements[0], GL_STATIC_DRAW);
	
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)0);

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, Normal));

		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, Color));

		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, TexCoords));
		
	glBindVertexArray(0);


	// Setup physic convex hull rendering 
	PhysicEngine::RenderPolyhedronData renderPolyHedronData = PhysicRigidBody->GetRenderPolyhedronData();
	
	hullVAO.vertices.resize(renderPolyHedronData.vertices.size());
	std::transform(renderPolyHedronData.vertices.begin(), renderPolyHedronData.vertices.end(), 
		hullVAO.vertices.begin(), [](PhysicEngine::RenderVertex& rv) { Vertex3D v{rv.position, rv.normal}; return v; });
	
	hullVAO.elements = renderPolyHedronData.elements;

	glGenVertexArrays(1, &hullVAO.VaoId);
	glGenBuffers(1, &hullVAO.VboId);
	glGenBuffers(1, &hullVAO.EboId);

	glBindVertexArray(hullVAO.VaoId);

	glBindBuffer(GL_ARRAY_BUFFER, hullVAO.VboId);
	glBufferData(GL_ARRAY_BUFFER, renderPolyHedronData.vertices.size() * sizeof(Vertex3D), &hullVAO.vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hullVAO.EboId);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, hullVAO.elements.size() * sizeof(unsigned int), &hullVAO.elements[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, Normal));

	glBindVertexArray(0);

	
	// Setup center of mass point rendering 
	centerOfMassVAO.vertices.push_back(Vertex3D { PhysicRigidBody->CenterOfMass } );
	centerOfMassVAO.elements.push_back(0);

	glGenVertexArrays(1, &centerOfMassVAO.VaoId);
	glGenBuffers(1, &centerOfMassVAO.VboId);
	glGenBuffers(1, &centerOfMassVAO.EboId);

	glBindVertexArray(centerOfMassVAO.VaoId);

	glBindBuffer(GL_ARRAY_BUFFER, centerOfMassVAO.VboId);
	glBufferData(GL_ARRAY_BUFFER, 1 * sizeof(Vertex3D), &centerOfMassVAO.vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, centerOfMassVAO.EboId);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 1 * sizeof(unsigned int), &centerOfMassVAO.elements[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

	glBindVertexArray(0);


	// Setup oriented bounding box rendering 
	PhysicRigidBody->boundingBox;
	boundingBoxVAO.vertices.resize(8);
	std::transform(std::begin(PhysicRigidBody->boundingBox.vertices), std::end(PhysicRigidBody->boundingBox.vertices),
		boundingBoxVAO.vertices.begin(), [](glm::vec3& inV) { Vertex3D v{ inV }; return v; });
	boundingBoxVAO.elements.resize(24);
	std::transform(std::begin(PhysicRigidBody->boundingBox.edges), std::end(PhysicRigidBody->boundingBox.edges),
		boundingBoxVAO.elements.begin(), [](unsigned int i) { return i; });

	glGenVertexArrays(1, &boundingBoxVAO.VaoId);
	glGenBuffers(1, &boundingBoxVAO.VboId);
	glGenBuffers(1, &boundingBoxVAO.EboId);

	glBindVertexArray(boundingBoxVAO.VaoId);

	glBindBuffer(GL_ARRAY_BUFFER, boundingBoxVAO.VboId);
	glBufferData(GL_ARRAY_BUFFER, boundingBoxVAO.vertices.size() * sizeof(Vertex3D), &boundingBoxVAO.vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boundingBoxVAO.EboId);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, boundingBoxVAO.elements.size() * sizeof(unsigned int), &boundingBoxVAO.elements[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)0);

	glBindVertexArray(0);
}


glm::mat4 Mesh3D::GetModelMatrix()
{
	return TranslationMatrix * RotationMatrix * ScaleMatrix;
}


void Mesh3D::Draw(GLuint shaderProgramId) 
{
	// Set up model transform
	GLuint modelLocation = glGetUniformLocation(shaderProgramId, "model");
	glUniformMatrix4fv(modelLocation, 1, GL_FALSE, glm::value_ptr(GetModelMatrix()));
	
	if (RenderModel)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glBindVertexArray(modelVAO.VaoId);
		glDrawElements(GL_TRIANGLES, modelVAO.elements.size(), GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}

	if (RenderConvexHull) 
	{
		// Draw center of mass
		glBindVertexArray(centerOfMassVAO.VaoId);
		glPointSize(20.0f);
		glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		// Draw convex hull
		glDisable(GL_CULL_FACE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);	
		glBindVertexArray(hullVAO.VaoId);
		glDrawElements(GL_TRIANGLES, hullVAO.elements.size(), GL_UNSIGNED_INT, 0);
		glDisable(GL_BLEND);
		glBindVertexArray(0);
	}

	if (RenderBoundingBox)
	{
		glBindVertexArray(boundingBoxVAO.VaoId);
		glDrawElements(GL_LINES, boundingBoxVAO.elements.size(), GL_UNSIGNED_INT, 0);
		glDisable(GL_BLEND);
		glBindVertexArray(0);
	}
}

