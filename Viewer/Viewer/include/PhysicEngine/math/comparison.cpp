#include "comparison.h"

namespace PhysicEngine 
{

	bool EpsCompareGlmVec3::operator()(const glm::vec3& v0, const glm::vec3& v1) const 
	{
		return EpsEqual(v0, v1);
	};

	bool HashGlmVec3::operator()(const glm::vec3& v) const
	{
		return std::hash<float>()(v.x) ^ std::hash<float>()(v.y) ^ std::hash<float>()(v.z);
	};
	
}