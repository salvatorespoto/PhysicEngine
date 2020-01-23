#pragma once

#include <math.h>    

#define GLM_FORCE_SWIZZLE 
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/epsilon.hpp>

#include "PhysicEngine/geometry/Vertex.h"
#include "PhysicEngine/geometry/Edge.h"
#include "PhysicEngine/geometry/Face.h"


namespace PhysicEngine 
{

	inline const float EPSILON_ZERO = 0.00001f;		/**< Floats that differ for less than EPSILON_ZERO are considered equals */

	/* FLOAT EPSILON COMPARISONS */

	inline bool EpsZero(float f)
	{
		return fabs(f) < EPSILON_ZERO ? true : false;
	}

	inline bool EpsEqual(float f0, float f1)
	{
		return fabs(f1 - f0) < EPSILON_ZERO ? true : false;
	}

	inline bool EpsGreaterThan(float f0, float f1)
	{
		return (f0 - f1) > EPSILON_ZERO ? true : false;
	}

	inline bool EpsLessThan(float f0, float f1)
	{
		return (f0 - f1) < -EPSILON_ZERO ? true : false;
	}

	inline bool EpsEqual(const glm::vec3& v0, const glm::vec3& v1)
	{
		glm::bvec3 b = glm::epsilonEqual(v0, v1, EPSILON_ZERO);
		return b[0] && b[1] && b[2];
	}

	inline bool EpsZero(const glm::vec3& v0)
	{
		glm::bvec3 b = glm::epsilonEqual(v0, glm::vec3(0, 0, 0), EPSILON_ZERO);
		return b[0] && b[1] && b[2];
	}

	/**
	 * Custom comparator for glm::vec3 type
	 */
	struct EpsCompareGlmVec3 
	{
		bool operator()(const glm::vec3& v0, const glm::vec3& v1) const;
	};

	/**
	 * Custom hash for glm::vec3 type
	 */
	struct HashGlmVec3 
	{
		bool operator()(const glm::vec3& v) const;
	};

}

namespace std {

	template <>
	struct hash<glm::vec3>
	{
		std::size_t operator()(const glm::vec3& v) const
		{
			using std::size_t;
			using std::hash;

			return ((hash<float>()(v.x)
				^ (hash<float>()(v.y) << 1)) >> 1)
				^ (hash<float>()(v.z) << 1);
		}
	};

	template <>
	struct hash<glm::vec4>
	{
		std::size_t operator()(const glm::vec4& v) const
		{
			using std::size_t;
			using std::hash;

			return (hash<glm::vec3>()(v))
				^ (hash<float>()(v.w) >> 1);
		}
	};

	template <>
	struct hash<PhysicEngine::Vertex>
	{
		std::size_t operator()(const PhysicEngine::Vertex& v) const
		{
			using std::size_t;
			using std::hash;

			return hash<glm::vec4>()(v.V());
		}
	};

	template <>
	struct hash<std::vector<glm::vec3>>
	{
		std::size_t operator()(const std::vector<glm::vec3>& vv) const
		{
			using std::size_t;
			using std::hash;

			size_t result = 0;
			for(glm::vec3 v : vv)
			{
				result += hash<glm::vec3>()(v);
			}
			return result;
		}
	};

}