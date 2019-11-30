#pragma once

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

typedef glm::vec3(*ForceFunction) (
	float t, 
	float ObjectMass,
	glm::vec3 Position, 
	glm::quat OrientationQuaternion, 
	glm::vec3 LinearMomentum,
	glm::vec3 AngularMomentum, 
	glm::mat3 OrientationMatrix, 
	glm::vec3 LinearVelocity, 
	glm::vec3 AngularVelocity);

typedef glm::vec3(*TorqueFunction) (
	float t,
	float ObjectMass,
	glm::vec3 Position, 
	glm::quat OrientationQuaternion, 
	glm::vec3 LinearMomentum,
	glm::vec3 AngularMomentum, 
	glm::mat3 OrientationMatrix, 
	glm::vec3 LinearVelocity, 
	glm::vec3 AngularVelocity);

glm::vec3 GravityForce(
	float t,
	float ObjectMass,
	glm::vec3 Position,
	glm::quat OrientationQuaternion,
	glm::vec3 LinearMomentum,
	glm::vec3 AngularMomentum,
	glm::mat3 OrientationMatrix,
	glm::vec3 LinearVelocity,
	glm::vec3 AngularVelocity);
