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

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <optional>
#include <set>
#include <functional>
#include <numeric>

#include "VulkanConfig.h"


/* Vulkan remove debug validation layer callback */
void DestroyDebugUtilsMessengerEXT(VkInstance instance, 
	VkDebugUtilsMessengerEXT debugMessenger, 
	const VkAllocationCallbacks* pAllocator);


/* This call initializa and run the Viewer application */
class ViewerApp {

private:

	boost::property_tree::ptree appProperties;				// Stores app properties loaded from configuration file
	int windowWidth;										// App window width
	int windowHeight;										// App window height
	GLFWwindow* window;										// App window instance

	VulkanConfig vulkanConfig;								// Util class to configure and initialize Vulkan
	
	
public:
	
	// Start the Viewer application 
	void run();
	
	// Load App configuration and initialize logs 
	void initUtils();

	// Initialize application window with GLFW 
	void setupAppWindow();

	// Initialize Vulkan API 
	void initVulkan();
	
	// Free all allocated resources 
	void shutDown();

	// Application main loop 
	void mainLoop();
};