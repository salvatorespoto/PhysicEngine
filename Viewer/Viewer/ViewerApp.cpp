#include "ViewerApp.h"


/* 
 * Load App configuration and initialize logs  
 */
void ViewerApp::initUtils() {

	// Init logs 
	boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
	boost::log::add_file_log
	(
		boost::log::keywords::file_name = "viewer.log",
		boost::log::keywords::format = "[%TimeStamp%] [%Severity%] %Message%"
	);
	boost::log::add_common_attributes();
	BOOST_LOG_TRIVIAL(info) << "Logs initialized";

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


/* 
 * Initialize application window with GLFW 
 */
void ViewerApp::initGLFW() {

	glfwInit();
	
	glfwSetErrorCallback(glfwErrorCallback);
	
	window = glfwCreateWindow
	(
		windowWidth,
		windowHeight,
		"Physics engine viewer",
		nullptr,
		nullptr
	);
	if (!window) {
		glfwTerminate();
		throw std::runtime_error("Error initializing GLFW window");
	}
	viewportWidth = windowWidth;
	viewportHeight = windowHeight;
	
	glfwSetKeyCallback(window, glfwKeyCallback);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	BOOST_LOG_TRIVIAL(info) << "Application windows initialized with GLFW";
}



/*
 * Initialize application window with GLFW
 */
void ViewerApp::initOpenGL() {

	glfwMakeContextCurrent(window);
	glfwGetFramebufferSize(window, &viewportWidth, &viewportHeight);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		BOOST_LOG_TRIVIAL(error) << "Failed to initialize extension loader library GLAD";
		return;
	}

	BOOST_LOG_TRIVIAL(info) << "OpenGL initialized";
}


/*
 * Error callback used from GLFW
 */
void ViewerApp::glfwErrorCallback(int error, const char* description) {
	BOOST_LOG_TRIVIAL(error) << "GLFW Error: " << description;
}


/*
 * Callback used from GLFW when a key is pressed
 */
void ViewerApp::glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
}


/*
 * Callback called on framebuffer resize
 */
void ViewerApp::framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}


/*
 * Process window inputs
 */
void ViewerApp::processInput()
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}


/*
 * Start the Viewer application
 */
void ViewerApp::run() {

	initUtils();		// Load configuration file and init logs
	initGLFW();			// Init the application window using GLWF library
	initOpenGL();		// Init OpenGL library
	mainLoop();			// Enter the viewer main loop
	shutDown();			// Free resources
}


/*
 * Application main loop
 */
void ViewerApp::mainLoop() {

	BOOST_LOG_TRIVIAL(debug) << "Entering mainloop";

	while (!glfwWindowShouldClose(window)) {

		processInput();
		glfwSwapBuffers(window);
		
		// Poll events calling appropriate callbacks
		glfwPollEvents();

	}

	BOOST_LOG_TRIVIAL(debug) << "Exiting mainloop";
}


/* 
 * Free all allocated resources 
 */
void ViewerApp::shutDown() {

	glfwDestroyWindow(window);
	BOOST_LOG_TRIVIAL(debug) << "GLFW Windows destroyed";
	
	glfwTerminate();
	BOOST_LOG_TRIVIAL(debug) << "GLFW terminated";

	BOOST_LOG_TRIVIAL(info) << "All resources freed";
}


