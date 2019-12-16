#pragma once

#include <map>
#include <set>
#include <unordered_set>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>



auto vec3Hash = [](const glm::vec3& v) {
	return std::hash<float>()(v.x) ^ std::hash<float>()(v.y) ^ std::hash<float>()(v.z);
};

auto vec3KeyEqual = [](const glm::vec3& vA, const glm::vec3& vB) {
	return vA.x == vB.x && vB.y == vA.y && vB.z == vA.z;
};
