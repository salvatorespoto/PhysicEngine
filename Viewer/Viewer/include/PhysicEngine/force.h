#pragma once

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

typedef glm::vec3(*ForceFunction) (float, glm::vec3, glm::quat, glm::vec3, glm::vec3, glm::mat3, glm::vec3, glm::vec3);
typedef glm::vec3(*TorqueFunction) (float, glm::vec3, glm::quat, glm::vec3, glm::vec3, glm::mat3, glm::vec3, glm::vec3);