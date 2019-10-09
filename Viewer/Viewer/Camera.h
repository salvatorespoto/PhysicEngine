#ifndef CAMERA_H
#define CAMERA_H

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp> 
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>



/** 
  * A camera class
  *
  * There are two types of supported camera mode: FPS and track ball.
  * FPS is the canonical First Person Shooter style camera, while in trackball mode
  * all the world rotate around the center (0,0)
  */
class Camera
{

public:
	
	/** Translation camera movements */
	enum Translation 
	{
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT,
		UP,
		DOWN
	};

	/** The camera looking direction. The forward vector actually will points "backward" in camera space */
	glm::vec3 ForwardVector;

	/** The camera up direction */
	glm::vec3 UpVector;

	/** The camera right */
	glm::vec3 RightVector;

	/** The field of view angle in degree */
	float FieldOfViewValue = 45.0f;

	float TranslationSpeed = 0.01f;
	float RotationSpeed = 0.001f;
	float ZoomSpeed = 0.01f;

	/** Enable / dsable track ball mode camera */
	float IsTrackBall = false;


	/** 
	 * Costructor 
	 *
	 * The camera has the following default values: 
	 * FrontVector (0,0,1), Upvector(0,1,0), RightVector(1,0,0), FieldOfViewValue 45 degree
	 * The FrontVector is facing backward the camera.
	 */
	Camera();

	/** Set the camera position */
	void SetPosition(const glm::vec3& positionVector);

	/** Traslate the camera */
	void Translate(const Translation direction, const float deltaTime);

	/** Rotate the camera around its local X, Y and Z axis. The rotation angles are multiplied by RotationSpeed */
	void Rotate(const float yaw, const float pitch, const float roll, const float deltaTime);

	/** Zoom the camera */
	void Zoom(const float yoffset);

	/** Returns the view matrix calculated using Euler Angles and the LookAt Matrix */
	glm::mat4 GetViewMatrix();


private:

	float Yaw = 0.0f;
	float Pitch = 0.0f;
	float Roll = 0.0f; 

	/** Current camera position */
	glm::vec3 PositionVector;
	
	
	
	/** The camera tralation */
	glm::mat4 TranslationMatrix = glm::mat4(1.0f);

	/** The camera orientation (rotation) quaternion */
	glm::quat RotationQuaternion = glm::normalize(glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)));
};

#endif
