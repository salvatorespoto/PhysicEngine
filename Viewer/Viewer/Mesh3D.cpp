#include "Mesh3D.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "include/tini_obj_loader.h"


Mesh3D::Mesh3D(const boost::filesystem::path& objFilePath) {
	LoadFromObjFile(objFilePath);
	SetupRender();
}


void Mesh3D::LoadFromObjFile(const boost::filesystem::path& objFilePath) {

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


    // Load "Model" shape
	boost::unordered_map<Vertex3D, unsigned int> uniqueVertices = {};
	for (const auto& index : shapes[modelShapeId].mesh.indices) {

		// Parse and save vertex data
		Vertex3D vertex = {};
		vertex.Position = {
			attrib.vertices[3 * index.vertex_index + 0],
			attrib.vertices[3 * index.vertex_index + 1],
			attrib.vertices[3 * index.vertex_index + 2]
		};
		vertex.Normal = {
			attrib.normals[3 * index.normal_index + 0],
			attrib.normals[3 * index.normal_index + 1],
			attrib.normals[3 * index.normal_index + 2]
		};
		vertex.Color = {
			1.0f,
			1.0f,
			0.0f
		};
		vertex.TexCoords = {
			attrib.texcoords[2 * index.texcoord_index + 0],
			1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
		};

		// Save vertex avoiding duplicates
		if (uniqueVertices.count(vertex) == 0) {
			uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
			vertices.push_back(vertex);
		}

		// Save vertex index
		indices.push_back(uniqueVertices[vertex]);
	}	
}


void Mesh3D::SetupRender()
{
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex3D), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)0);

	glBindVertexArray(0);
}


void Mesh3D::Draw() {

	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}
