#include "ViewerApp.h"





/* Start the Viewer application */
void ViewerApp::run() {

	// Load configuration file and init logs
	initUtils();

	// Create the window app using GLSW
	setupAppWindow();
	
	// Config and start Vulkan API
	initVulkan();

	// Enter the application main loop
	mainLoop();

	// Free resources
	shutDown();
}


/* Load App configuration and initialize logs */
void ViewerApp::initUtils() {

	// Init logs 
	boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
	boost::log::add_file_log
	(
		boost::log::keywords::file_name = "viewer.log",
		boost::log::keywords::format = "[%TimeStamp%] [%Severity%] %Message%"
	);
	boost::log::add_common_attributes();
	BOOST_LOG_TRIVIAL(info) << "Logs initialized.";

	// Load configuration file
	boost::property_tree::ini_parser::read_ini("config.ini", appProperties);
	BOOST_LOG_TRIVIAL(info) << "Configuration file 'config.ini' loaded";

	// Set logs level
	BOOST_LOG_TRIVIAL(info) << "Setting logs level to " << appProperties.get<std::string>("Logs.Level");
	boost::log::core::get()->set_filter
	(
		boost::log::trivial::severity >= boost::lexical_cast<boost::log::trivial::severity_level>(appProperties.get<std::string>("Logs.Level"))
	);

	// Load configuration variables from config.ini
	windowWidth = boost::lexical_cast<int>(appProperties.get<std::string>("Application.Width"));
	windowHeight = boost::lexical_cast<int>(appProperties.get<std::string>("Application.Width"));
}


/* Initialize application window with GLFW */
void ViewerApp::setupAppWindow() {

	glfwInit();
	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	window = glfwCreateWindow
	(
		windowWidth,
		windowHeight,
		"Physics engine viewer",
		nullptr,
		nullptr
	);

	BOOST_LOG_TRIVIAL(info) << "Application windows initialized with GLFW.";
}


/* Initialize Vulkan API */
void ViewerApp::initVulkan() {

	/* Configure validation layers */
	vulkanConfig.enableValidationLayers = boost::lexical_cast<bool>(appProperties.get<std::string>("Vulkan.EnableValidationLayers"));
	vulkanConfig.validationLayers = { "VK_LAYER_KHRONOS_validation" };


	/* Configure required instance extensions */
	std::vector<const char*> instanceExtensions;

	// Add extensions required from GLFW.
	uint32_t count = 0;
	const char** glfwRequiredExtensions;
	glfwRequiredExtensions = glfwGetRequiredInstanceExtensions(&count);
	BOOST_LOG_TRIVIAL(debug) << "Required Vulkan Extensions from GLFW:";
	for (uint32_t i = 0; i < count; i++) BOOST_LOG_TRIVIAL(debug) << " " << glfwRequiredExtensions[i];
	instanceExtensions.insert(instanceExtensions.end(), glfwRequiredExtensions, glfwRequiredExtensions + count);

	// Add extension required for utils message debug
	if (vulkanConfig.enableValidationLayers) instanceExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);


	/* Configure Vulkan instance */
	vulkanConfig.applicationName = "Physic Engine Viewer";
	vulkanConfig.enableCreateGLFWSurface = true;
	vulkanConfig.glfwWindow = window;
	vulkanConfig.instanceExtensions = instanceExtensions;
	vulkanConfig.deviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };

	/* Initialize Vulkan API */
	vulkanConfig.setUp();
}


/* Free all allocated resources */
void ViewerApp::shutDown() {

	vulkanConfig.cleanUp();

	glfwDestroyWindow(window);
	BOOST_LOG_TRIVIAL(debug) << "GLFW Windows destroyed";

	glfwTerminate();
	BOOST_LOG_TRIVIAL(debug) << "GLFW terminated";

	BOOST_LOG_TRIVIAL(debug) << "All resources freed";
}


/* Application main loop */
void ViewerApp::mainLoop() {

	BOOST_LOG_TRIVIAL(debug) << "Entering mainloop";
	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();
	}

	BOOST_LOG_TRIVIAL(debug) << "Exiting mainloop";
}