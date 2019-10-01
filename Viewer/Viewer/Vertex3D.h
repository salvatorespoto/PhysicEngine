#ifndef VERTEX_3D
#define VERTEX_3D

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>


/** A 3D mesh vertex */
struct Vertex3D {

	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec3 Color;
	glm::vec2 TexCoords;

	friend bool operator==(const Vertex3D& v1, const Vertex3D& v2) {
		return v1.Position == v2.Position && v1.Normal == v2.Normal && v1.Color == v2.Color && v1.TexCoords == v2.TexCoords;
	}

	friend std::size_t hash_value(Vertex3D const& v)
	{
		std::size_t seed = 0;

		boost::hash_combine(seed, v.Position.x);
		boost::hash_combine(seed, v.Position.y);
		boost::hash_combine(seed, v.Position.z);

		boost::hash_combine(seed, v.Normal.x);
		boost::hash_combine(seed, v.Normal.y);
		boost::hash_combine(seed, v.Normal.z);

		boost::hash_combine(seed, v.Color.x);
		boost::hash_combine(seed, v.Color.y);
		boost::hash_combine(seed, v.Color.z);

		boost::hash_combine(seed, v.TexCoords.x);
		boost::hash_combine(seed, v.TexCoords.y);

		return seed;
	}
};

#endif
