#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/from_stream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/filter_parser.hpp>
#include <boost/range/adaptors.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "include/glad/glad.h"



/* 
 *  This class initialize and run the Viewer application 
 */
class ViewerApp {

private:

	boost::property_tree::ptree appProperties;				// Stores app properties loaded from configuration file
	
	GLFWwindow* window;										// App window instance
	int windowWidth;										
	int windowHeight;										
	int viewportWidth;										
	int viewportHeight;										

	
public:
	
	// Callback for errors reported from GLFW library
	static void glfwErrorCallback(int error, const char* description);

	// Callback used from GLFW when a key is pressed
	static void glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

	// Callback called on framebuffer resize
	static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

	// Load App configuration and initialize logs 
	void initUtils();

	// Initialize application window with GLFW 
	void initGLFW();

	// Init OpenGL
	void initOpenGL();

	// Start the Viewer application 
	void run();

	// Process window inputs
	void processInput();

	// Application main loop 
	void mainLoop();

	// Free all allocated resources 
	void shutDown();
};