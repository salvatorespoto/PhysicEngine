#include "DataTypes.h"

using namespace PhysicEngine;

ConvexPolyhedron::ConvexPolyhedron(std::vector<glm::vec3>& vertices, std::vector<glm::ivec3>& faces) 
	: Vertices(vertices) {

	// Store faces
	for (glm::ivec3 face : faces) {

		Face f;
		f.verticesIndexes = face;

		// Normal il the vector cross product of the two edges v1-v0 X v2-v0, normalized
		glm::vec3 edge0 = vertices[face[1]] - vertices[face[0]];
		glm::vec3 edge1 = vertices[face[2]] - vertices[face[0]];

		f.normal = glm::normalize(glm::cross(edge0, edge1));
	}

	ComputeOrientedBoundingBox();
	//ComputeCenterOfMassAndIntertiaTensor();
	
}


void ConvexPolyhedron::ComputeCenterOfMassAndIntertiaTensor() {

	float integral[10] = { 0,0,0,0,0,0,0,0,0,0 };

	// Iterate all the faces
	for (Face face : Faces) {

		// Get the verticies of this triangle
		Vertex v0 = Vertices[face.verticesIndexes[0]];
		Vertex v1 = Vertices[face.verticesIndexes[1]];
		Vertex v2 = Vertices[face.verticesIndexes[2]];

		// Get cross product of edges [v1-v0] x [v2-v1] 
		glm::vec3 edge0 = Vertices[face.verticesIndexes[1]] - Vertices[face.verticesIndexes[0]];
		
		glm::vec3 edge1 = Vertices[face.verticesIndexes[2]] - Vertices[face.verticesIndexes[0]];
		glm::vec3 d = glm::cross(edge0, edge1);

		float f1x, f2x, f3x, f1y, f2y, f3y, f1z, f2z, f3z, g0x, g1x, g2x, g0y, g1y, g2y, g0z, g1z, g2z;

		SubExpression(v0.x, v1.x, v2.x, f1x, f2x, f3x, g0x, g1x, g2x);
		SubExpression(v0.y, v1.y, v2.y, f1y, f2y, f3y, g0y, g1y, g2y);
		SubExpression(v0.z, v1.z, v2.z, f1z, f2z, f3z, g0z, g1z, g2z);
		
		integral[0] = d.x * f1x;
		integral[1] = d.x * f2x;
		integral[2] = d.y * f2y;
		integral[3] = d.z * f2z;
		integral[4] = d.x * f3x;
		integral[5] = d.y * f3y;
		integral[6] = d.z * f3z;
		integral[7] = d.x * (v0.y * g0x + v1.y * g1x + v2.y * g2x);
		integral[8] = d.x * (v0.z * g0y + v1.z * g1y + v2.z * g2y);
		integral[9] = d.x * (v0.x * g0z + v1.x * g1z + v2.x * g2z);
	}

	integral[0] *= oneDiv6;
	integral[1] *= oneDiv24;
	integral[2] *= oneDiv24;
	integral[3] *= oneDiv24;
	integral[4] *= oneDiv60;
	integral[5] *= oneDiv60;
	integral[6] *= oneDiv60;
	integral[7] *= oneDiv120;
	integral[8] *= oneDiv120;
	integral[9] *= oneDiv120;

	mass = integral[0];

	centerOfMass.x = integral[1] / mass;
	centerOfMass.y = integral[2] / mass;
	centerOfMass.z = integral[3] / mass;

	// Intertia tensor relative to world origin
	intertiaTensor[1][1] = integral[5] + integral[6]; //xx
	intertiaTensor[2][2] = integral[4] + integral[6]; //yy
	intertiaTensor[2][2] = integral[4] + integral[5]; //zz
	intertiaTensor[1][2] = integral[7]; //xy
	intertiaTensor[2][3] = integral[8]; //yz
	intertiaTensor[1][3] = integral[9]; //xz

	// Intertia tensor relative to center of mass
	intertiaTensor[1][1] = mass * (centerOfMass.y * centerOfMass.y + centerOfMass.z * centerOfMass.z); //xx
	intertiaTensor[2][2] = mass * (centerOfMass.z * centerOfMass.z + centerOfMass.x * centerOfMass.x); //yy
	intertiaTensor[3][3] = mass * (centerOfMass.x * centerOfMass.x + centerOfMass.y * centerOfMass.y); //zz
	intertiaTensor[1][2] = mass * centerOfMass.x * centerOfMass.y; //xy
	intertiaTensor[2][1] = mass * centerOfMass.y * centerOfMass.z; //yz
	intertiaTensor[1][3] = mass * centerOfMass.z * centerOfMass.x; // xz
}


void ConvexPolyhedron::SubExpression(float& w0, float& w1, float& w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2) {

	float temp0, temp1, temp2;

	temp0 = w0 + w1;
	f1 = temp0 + w2;
	temp1 = w0 + w0;
	temp2 = temp1 + w1 * temp0;
	f2 = temp2 + w2 * f1;
	f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
	g0 = f2 + w0 * (f1 + w0);
	g1 = f2 + w1 * (f1 + w1);
	g2 = f2 + w2 * (f1 + w2);
}



void ConvexPolyhedron::ComputeOrientedBoundingBox() {
	

	Eigen::Matrix<float, 3, 3> covarianceMatrix;
	float means[3] = { 0, 0, 0 };

	
	for (Vertex v : Vertices) {
		means[0] += v.x,
		means[1] += v.y,
		means[2] += v.z;
	}

	means[0] /= Vertices.size(); 
	means[1] /= Vertices.size(); 
	means[2] /= Vertices.size();

	// Fill the covariance matrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {

			covarianceMatrix(i,j) = 0.0;
			for (Vertex v : Vertices) {
				covarianceMatrix(i,j) += (means[i] - v[i]) * (means[j] - v[j]);
			}
			covarianceMatrix(i,j) /= Vertices.size() - 1;
		}
	}

	// Use Eigen library to compute covariance matrix eigenvectors 
	Eigen::EigenSolver<Eigen::Matrix<float, 3, 3> > s(covarianceMatrix);
	glm::vec3 c0 = { s.eigenvectors().col(0)[0].real(), s.eigenvectors().col(0)[1].real(), s.eigenvectors().col(0)[2].real() };
	glm::vec3 c1 = { s.eigenvectors().col(1)[0].real(), s.eigenvectors().col(1)[1].real(), s.eigenvectors().col(1)[2].real() };
	glm::vec3 c2 = { s.eigenvectors().col(2)[0].real(), s.eigenvectors().col(2)[1].real(), s.eigenvectors().col(2)[2].real() };

	// Compute the projection of points over the eigenvectors direction
	glm::vec3 min = { 0, 0, 0 };
	glm::vec3 max = { 0, 0, 0 };
	for (Vertex v : Vertices) {
		min = glm::min(glm::vec3(glm::dot(v, c0), glm::dot(v, c1), glm::dot(v, c2)), min);
		max = glm::max(glm::vec3(glm::dot(v, c0), glm::dot(v, c1), glm::dot(v, c2)), max);
	}

	// Compute the center of the bouding box
	glm::vec3 center = (min + max) / 2.0f;
	glm::vec3 extension = (max - min) / 2.0f;

	boundingBox = {
		{   // Bounding box vertices 
			center + glm::vec3(extension.x, extension.y, extension.z),
			center + glm::vec3(extension.x, extension.y, -extension.z),
			center + glm::vec3(-extension.x, extension.y, -extension.z),
			center + glm::vec3(-extension.x, extension.y, extension.z),
			center + glm::vec3(extension.x, -extension.y, extension.z),
			center + glm::vec3(extension.x, -extension.y, -extension.z),
			center + glm::vec3(-extension.x, -extension.y, -extension.z),
			center + glm::vec3(-extension.x, -extension.y, extension.z)
		},
		{	// Boundign sixteen box edges, each two integer represent an edge
			1, 2, 2, 3, 3, 4, 4 ,1, 5, 6, 6, 7, 7, 8, 8, 5, 1, 5, 2, 6, 3, 7, 4, 8
		}
	};

	int pippo = 1;
}