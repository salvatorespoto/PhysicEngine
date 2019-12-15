// Copyright 2019 Salvatore Spoto

#ifndef VIEWERAPP_H
#define VIEWERAPP_H

#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/from_stream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/filter_parser.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "include/glad/glad.h"

#include "Utils.h"
#include "Shader.h"
#include "Mesh3D.h"
#include "Camera.h"

#include "PhysicEngine/Engine.h"
#include "PhysicEngine/DataTypes.h"
#include "PhysicEngine/Functions.h"
#include "PhysicEngine/RigidBody.h"
#include "PhysicEngine/collision.h"


/** 
 * Callback used from GLFW to report errors 
 */
void glfwErrorCallback(int error, const char* description);


/** 
  * This class initializes and runs the Viewer application 
  */
class ViewerApp {

public:

	/** 
	 * Return the singleton instance of ViewerApp
	 */
	static ViewerApp& GetInstance();

	/**
	 * Run the application (allocate resources, handle inputs, enters the rendering loop, free resources on exit)
	 */
	void Run();
	
	/** 
	 * Handle the keyboard inputs 
	 */
	void HandleKeyboardInput();

	/** 
	 * Handle mouse inputs, both movement and buttons 
	 */
	void HandleMouseInput();

	/**
	 * Callback Called on framebuffer resize
	 *
	 * This callback is registered and called when a framebuffer resize event occur
	 * 
	 * @param window An handle to the current window
	 * @param width	The new framebuffer width
	 * @param height The new framebuffer height
	 */
	static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);


private:

	bool RenderModel = true;				/**< Show / hide object models */
	bool RenderBoundingBox = false;			/**< Show / hide object bounding boxes */			
	bool RenderConvexHull = false;			/**< Show / hide object convex hulls */

	unsigned int WindowWidth;				/**< Window width */
	unsigned int WindowHeight;				/**< Window height */
	GLFWwindow* Window;						/**< Handle to the current window */

	int ViewportWidth;						/**< Viewport width */
	int ViewportHeight;						/**< Viewport height */

	double MouseXCoordinate = 99999.0f;		/**< Track the X mouse coordinate */
	double MouseYCoordinate = 99999.0f;		/**< Track the Y mouse coordinate */

	GLuint ShaderProgram;
	
	/** All shader programs with their name */
	std::map<std::string, Shader*> ShadersMap;
	
	/** All loaded meshes */
	std::unordered_map<std::string, Mesh3D*> MeshMap;

	/** A pointer to the selected mesh */
	Mesh3D* SelectedMesh;

	/** Currently dragging a mesh */
	bool draggingMesh = false;

	/** Viewer First Person Shooter style camera */
	Camera CameraFPS;

	/** Trackball camera */
	Camera TrackBall;
	
	/** App properties loaded from configuration file */
	boost::property_tree::ptree ViewerProperties;

	/** Current Viewer app execution path */
	boost::filesystem::path CurrentPath;

	/** Current pressed mouse button. -1 means no mouse button pressed */
	int PressedMouseButton = -1; 

	/** Current frame time */
	int CurrentFrameTime = 0;

	/** The projection matrix */
	glm::mat4 ProjectionMatrix;


	PhysicEngine::RigidBodyEngine physicEngine;			/**< The instance of the rigid body physic engine */

	/** Private constructor, ViewerApp it's a Singleton*/
	ViewerApp();

	/** Load App configuration and initialize logs */
	void InitUtils();

	/** Initialize application window with GLFW */
	void InitGLFW();

	/** Init OpenGL */
	void InitOpenGL();

	/** Load meshes */
	void LoadMeshes();

	/** Process window inputs */
	void ProcessInput();

	/** True if the mouse is over a mesh */
	bool CheckMouseObjectsIntersection(Mesh3D& mesh, float& outDistance);

	/** Application main rendering loop */
	void RenderLoop();

	/** Update the camera */
	void UpdateCamera();

	/** Setup the 3D scene */
	void SetupScene();

	/** Rendering scene */
	void DrawWorld();

	/** Free all allocated resources */
	void ShutDown();
};

#endif
