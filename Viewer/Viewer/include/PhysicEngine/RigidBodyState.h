#pragma once

#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>


#include "DataTypes.h"

using namespace PhysicEngine;
class RigidBodyState 
{
	

	void RigidBodyState::ComputeSecondaryState(glm::vec3 Position, glm::quat OrientationMatrix, glm::vec3 LinearMomentum, glm::vec3 AngularMomentum,
		glm::mat3 OrientationMatrix, glm::vec3 LinearVelocity, glm::vec3 AngularVelocity)
	{
		OrientationMatrix = glm::toMat4(OrientationQuaternion);
		LinearVelocity = InvertedMass * LinearMomentum;
		AngularVelocity = OrientationMatrix * InvertedIntertiaTensor * glm::transpose(OrientationMatrix) * AngularMomentum;
	}


};