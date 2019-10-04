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



/** Callback used from GLFW to report errors */
void glfwErrorCallback(int error, const char* description);


/** This class initializes and runs the Viewer application */
class ViewerApp {

public:

	/** Return the singleton instance */
	static ViewerApp& GetInstance();

	/** Called on framebuffer resize */
	static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

	/** Run the application (allocate resources, handle inputs, enters the rendering loop, free resources on exit) */
	void Run();
	
	/** Handle all the keyboard input */
	void HandleKeyboardInput();

	/** Handle all mouse input, both movement and buttons */
	void HandleMouseInput();


private:

	/** Show / hide object models */
	bool RenderModel = true;

	/** Show / hide object bounding boxes */
	bool RenderBoundingBox = false;

	/** Show / hide object convex hulls */
	bool RenderConvexHull = false;

	/** Handle to the current window */
	GLFWwindow* Window;				

	unsigned int WindowWidth;
	unsigned int WindowHeight;
	
	/** Keep track of the X mouse input coord */
	double MouseXCoordinate = 99999.0f;

	/** Keep track of the Y mouse input coord */
	double MouseYCoordinate = 99999.0f;

	int ViewportWidth;
	int ViewportHeight;
	
	GLuint ShaderProgram;
	
	/** All shader programs with their name */
	std::map<std::string, Shader*> ShadersMap;
	
	/** All loaded meshes */
	std::list<Mesh3D*> MeshList;

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

	/** Application main rendering loop */
	void RenderLoop();

	/** Update the camera */
	void UpdateCamera();

	/** Rendering scene */
	void DrawWorld();

	/** Free all allocated resources */
	void ShutDown();
};

#endif
