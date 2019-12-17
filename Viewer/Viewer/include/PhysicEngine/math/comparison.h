#pragma once

#include <math.h>    
#include <glm/vec3.hpp>
#include <glm/gtc/epsilon.hpp>

#include "../geometry/Edge.h"


namespace PhysicEngine 
{

	inline const float EPSILON_ZERO = 0.00001f;		/**< Floats that differ for less than EPSILON_ZERO are considered equals */

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
	struct EpsCompareGlmVec3 {
		bool operator()(const glm::vec3& v0, const glm::vec3& v1) const;
	};

	/**
	 * Custom hash for glm::vec3 type
	 */
	struct HashGlmVec3 {
		bool operator()(const glm::vec3& v) const;
	};

	/**
	 * Custom hash for Edge Type
	 */
	struct HashEdge {
		bool operator()(const Edge& v) const;
	};

}