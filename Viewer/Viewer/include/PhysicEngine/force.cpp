#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#include "force.h"

glm::vec3 NullForce(
	float t,
	float ObjectMass,
	glm::vec3 Position,
	glm::quat OrientationQuaternion,
	glm::vec3 LinearMomentum,
	glm::vec3 AngularMomentum,
	glm::mat3 OrientationMatrix,
	glm::vec3 LinearVelocity,
	glm::vec3 AngularVelocity)
{
	return glm::vec3(0, 0, 0);
}


glm::vec3 NullTorque(
	float t,
	float ObjectMass,
	glm::vec3 Position,
	glm::quat OrientationQuaternion,
	glm::vec3 LinearMomentum,
	glm::vec3 AngularMomentum,
	glm::mat3 OrientationMatrix,
	glm::vec3 LinearVelocity,
	glm::vec3 AngularVelocity)
{
	return glm::vec3(0, 0, 0);
}


glm::vec3 GravityForce(
	float t,
	float ObjectMass,
	glm::vec3 Position,
	glm::quat OrientationQuaternion,
	glm::vec3 LinearMomentum,
	glm::vec3 AngularMomentum,
	glm::mat3 OrientationMatrix,
	glm::vec3 LinearVelocity,
	glm::vec3 AngularVelocity)
{
	return glm::vec3(0.0f, ObjectMass * -9.8f, 0.0f);
}

