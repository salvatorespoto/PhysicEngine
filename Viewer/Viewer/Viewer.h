#pragma once

#include <limits>

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
#include "PhysicEngine/physics/RigidBody.h"
#include "PhysicEngine/collision/collision.h"


/** 
 * Callback used from GLFW to report errors 
 *
 * @param error The reported error code
 * @param description A description of the error occurred
 */
void glfwErrorCallback(int error, const char* description);



/** 
  * The Viewer class initializes and runs the viewer application
  */
class Viewer 
{
	public:

		/** 
		 * Return the singleton instance of the Viewer
		 */
		static Viewer& GetInstance();

		/**
		 * Run the application
		 */
		void Run();

		/**
		 * Callback Called on framebuffer resize event
		 *
		 * @param window An handle to the current window
		 * @param width	The new framebuffer width
		 * @param height The new framebuffer height
		 */
		static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);


	private:

		boost::property_tree::ptree ViewerProperties;	/**< Stores app properties loaded from the configuration file */
		boost::filesystem::path CurrentPath;			/**< Current execution path */

		GLFWwindow* Window;						/**< Handle to the current window */
		unsigned int WindowWidth;				/**< Window width */
		unsigned int WindowHeight;				/**< Window height */
		int ViewportWidth;						/**< Viewport width */
		int ViewportHeight;						/**< Viewport height */


		int PressedMouseButton = -1;			/**< Current pressed mouse button. -1 means no mouse button is pressed */
		double MouseXCoordinate = std::numeric_limits<double>::max();	/**< Track the X mouse coordinate */
		double MouseYCoordinate = std::numeric_limits<double>::max();	/**< Track the Y mouse coordinate */

		Camera CameraFPS;						/** First Person Shooter style camera */
		Camera TrackBall;						/** Trackball camera */
	
		bool RenderModel = true;				/**< Show / hide object models */
		bool RenderBoundingBox = false;			/**< Show / hide object oriented bounding boxes */
		bool RenderConvexHull = false;			/**< Show / hide object convex hulls */

		GLuint ShaderProgram;
	
		std::map<std::string, Shader*> ShadersMap;			/**< A map "Shader name" -> Shader*. Stores all used shaders */
		std::unordered_map<std::string, Mesh3D*> MeshMap;	/**< A map "Mesh name" -> Medh3D*. Stores all 3d mesh loaded */
	
		Mesh3D* SelectedMesh;					/**< A pointer to the selected mesh */
		bool draggingMesh = false;				/**< True if a mesh is currently dragged in the viewport */
	
		glm::mat4 ProjectionMatrix;				/**< The projection matrix */
		
		PhysicEngine::RigidBodyEngine physicEngine;		/**< The instance of the rigid body physic engine */


		Viewer();					/**< Private constructor, ViewerApp it's a Singleton*/
				
		void InitUtils();			/**< Load App configuration and initialize logs */
		void InitGLFW();			/**< Initialize application window with GLFW */
		void InitOpenGL();			/**< Init OpenGL */
		void SetupScene();			/** Setup the 3D scene */
		void RenderLoop();			/** Application main rendering loop */
		void ProcessInput();		/** Process window inputs */
		void HandleKeyboardInput();	/**< Handle keyboard inputs */
		void HandleMouseInput();	/**< Handle mouse inputs */
		bool CheckMouseObjectsIntersection(Mesh3D& mesh, float& outDistance);	/** True if the mouse is over a mesh */
		void DrawWorld();			/**< Rendering scene */
		void ShutDown();			/**< Free all resources */
};