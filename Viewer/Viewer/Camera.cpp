#include "Camera.h"
#include <boost/log/trivial.hpp>

Camera::Camera() : PositionVector(glm::vec3(0.0f, 0.0f, -1.0f)), ForwardVector(glm::vec3(0.0f, 0.0f, 1.0f)), 
	UpVector(glm::vec3(0.0f, 1.0f, 0.0f)), RightVector(glm::vec3(0.1f, 0.0f, 0.0f))
{	
	glm::quat OrientationQuaternion = glm::normalize(glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)));
}


void Camera::SetPosition(const glm::vec3& positionVector)
{
	PositionVector = positionVector;
	TranslationMatrix = glm::translate(glm::mat4(1.0f), -PositionVector);
}


void Camera::Translate(const Translation direction, const float deltaTime)
{
	float velocity = TranslationSpeed * deltaTime;

	// The forward vector point in the backward camera direction 
	if (direction == FORWARD) PositionVector -= ForwardVector * velocity;
	if (direction == BACKWARD) PositionVector += ForwardVector * velocity;

	if (direction == RIGHT) PositionVector += RightVector * velocity;
	if (direction == LEFT) PositionVector -= RightVector * velocity;
	
	if (direction == UP) PositionVector += UpVector * velocity;
	if (direction == DOWN) PositionVector -= UpVector * velocity;

	// Compute new traslation matrix, the wolrd translation is the inverse 
	// of the camera tranlation (-PositionVector)
	TranslationMatrix = glm::translate(glm::mat4(1.0f), -PositionVector);
}


void Camera::Rotate(const float yaw, const float pitch, const float roll, const float deltaTime) 
{
	// Compute new camera quaternion orientation basing on yaw, pitch and roll
	// Yaw, pitch, roll are counterclockwisee rotation about the Y, X and Z axis
	Yaw += yaw * RotationSpeed * deltaTime;
	Pitch += pitch * RotationSpeed * deltaTime;
	Roll += roll * RotationSpeed * deltaTime;
	
	// Build a quaternion from euler angles (pitch, yaw, roll), in radians 
	// and get the inverse camera rotation
	RotationQuaternion = glm::inverse(glm::normalize(glm::quat(glm::vec3(Pitch, Yaw, Roll))));
	
	// Compute new camera vectors 
	ForwardVector = glm::vec3(0.0f, 0.0f, 1.0f) * RotationQuaternion;
	UpVector = glm::vec3(0.0f, 1.0f, 0.0f) * RotationQuaternion;
	RightVector = glm::vec3(1.0f, 0.0f, 0.0f) * RotationQuaternion;	
}


void Camera::Zoom(const float value)
{
	if (FieldOfViewValue >= 1.0f && FieldOfViewValue <= 45.0f) FieldOfViewValue -= (value * ZoomSpeed);
	if (FieldOfViewValue <= 1.0f) FieldOfViewValue = 1.0f;
	if (FieldOfViewValue >= 45.0f) FieldOfViewValue = 45.0f;
}


glm::mat4 Camera::GetViewMatrix()
{
	if(IsTrackBall) 
	{
		// Trackball camera
		return TranslationMatrix * glm::mat4_cast(RotationQuaternion);
	}
	else 
	{
		// First Person Shooter camera
		return glm::mat4_cast(RotationQuaternion) * TranslationMatrix;
	}
}