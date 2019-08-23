#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/from_stream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/filter_parser.hpp>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <optional>


/* Vulkan debug validation layer callback */
static VKAPI_ATTR VkBool32 VKAPI_CALL vulkanDebugCallback(
	VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
	VkDebugUtilsMessageTypeFlagsEXT messageType,
	const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
	void* pUserData) {

	BOOST_LOG_TRIVIAL(info) << "Validation layer: " << pCallbackData->pMessage << ".";
	return VK_FALSE;
}

/* Vulkan remove debug validation layer callback */
void DestroyDebugUtilsMessengerEXT(VkInstance instance, 
	VkDebugUtilsMessengerEXT debugMessenger, 
	const VkAllocationCallbacks* pAllocator);

class ViewerApp {

private:

	boost::property_tree::ptree appProperties;				// Stores app properties loaded from configuration file
	GLFWwindow* window;										// App window instance
	VkInstance instance;									// Vulkan instance
	
	// List of required validation layers
	std::vector<const char*> requiredValidationLayers = {
		"VK_LAYER_KHRONOS_validation"
	};

	// Used to declare the debug function for the validation layer
	VkDebugUtilsMessengerEXT vulkanDebugMessenger;				


public:

	void run() {
		
		initUtils();
		initGLFWWindow();
		initVulkan();

		mainLoop();

		cleanup();
	}

	/* Load configuration and initialize logs */
	void initUtils() {
		
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
		BOOST_LOG_TRIVIAL(info) << "Configuration file 'config.ini' loaded.";

		// Set logs level
		BOOST_LOG_TRIVIAL(info) << "Setting logs level to " << appProperties.get<std::string>("Logs.Level") << ".";
		boost::log::core::get()->set_filter
		(
			boost::log::trivial::severity >= boost::lexical_cast<boost::log::trivial::severity_level>(appProperties.get<std::string>("Logs.Level"))
		);
	}


	/* Initialize application window with GLFW */
	void initGLFWWindow() {

		glfwInit();
		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
		glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
		window = glfwCreateWindow
		(
			boost::lexical_cast<int>(appProperties.get<std::string>("Application.Width")),
			boost::lexical_cast<int>(appProperties.get<std::string>("Application.Height")),
			"Vulkan",
			nullptr,
			nullptr
		);
		BOOST_LOG_TRIVIAL(info) << "Initialize application windows with GLFW.";
	}

	/* Initialize Vulkan API */
	void initVulkan() {

		// VALIDATION LAYERS ------------------------------------------------------------------
		// Get all supported validation layers
		uint32_t layerCount;
		vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
		std::vector<VkLayerProperties> availableLayers(layerCount);
		vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());
		BOOST_LOG_TRIVIAL(debug) << "Supported Vulkan Layers:";
		for (const auto& layer : availableLayers) {
			BOOST_LOG_TRIVIAL(debug) << " " << layer.layerName;
		}

		// Check if requested validation layers are available
		std::for_each(requiredValidationLayers.begin(), requiredValidationLayers.end(),
			[availableLayers](const char* requiredLayer) 
			{
				if (std::find_if(availableLayers.begin(), availableLayers.end(), 
					[requiredLayer](const VkLayerProperties layer) 
					{
						return (std::strcmp(layer.layerName, requiredLayer) == 0);
					}) == availableLayers.end())
				{
					throw std::runtime_error("Requested Layer " + std::string(requiredLayer) + " not available");
				}
			});
		BOOST_LOG_TRIVIAL(debug) << "Required Vulkan Layers:";
		for (const auto& layer : requiredValidationLayers) {
			BOOST_LOG_TRIVIAL(debug) << " " << layer;
		}

		// EXTENSIONS ------------------------------------------------------------------
		// Get the list of supported Vulkan extensions
		uint32_t extensionCount = 0;
		vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);
		std::vector<VkExtensionProperties> extensions(extensionCount);
		vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, extensions.data());
		BOOST_LOG_TRIVIAL(debug) << "Supported Vulkan Extensions:";
		for (const auto& extension : extensions) {
			BOOST_LOG_TRIVIAL(debug) << " " << extension.extensionName;
		}

		// Query GLFW for the list of extensions it needs
		uint32_t glfwRequiredExtensionsCount = 0;
		const char** glfwRequiredExtensions;
		glfwRequiredExtensions = glfwGetRequiredInstanceExtensions(&glfwRequiredExtensionsCount);
		BOOST_LOG_TRIVIAL(debug) << "Required Vulkan Extensions from GLFW:";
		for (uint32_t i = 0; i < glfwRequiredExtensionsCount; i++) {
			BOOST_LOG_TRIVIAL(debug) << " " << glfwRequiredExtensions[i];
		}
		std::vector<const char*> requiredExtensions(glfwRequiredExtensions, glfwRequiredExtensions + glfwRequiredExtensionsCount);
		
		// Add the extensions we need
		std::string a = appProperties.get<std::string>("Vulkan.EnableValidationLayers");
		if (boost::lexical_cast<bool>(appProperties.get<std::string>("Vulkan.EnableValidationLayers"))) {
			requiredExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
		}

		// Setup DEBUG FUNCTION ---------------------------------------------------------
		VkDebugUtilsMessengerCreateInfoEXT createDebugFunctionInfo = {};
		createDebugFunctionInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
		createDebugFunctionInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
		createDebugFunctionInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
		createDebugFunctionInfo.pfnUserCallback = vulkanDebugCallback;
		createDebugFunctionInfo.pUserData = nullptr;

		// Create Vulkan instance 
		VkApplicationInfo appInfo = {};
		appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		appInfo.pApplicationName = "Physic Engine Viewer";
		appInfo.applicationVersion = VK_MAKE_VERSION(0, 0, 1);
		appInfo.apiVersion = VK_API_VERSION_1_0;
		VkInstanceCreateInfo createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		createInfo.pApplicationInfo = &appInfo;
		if (boost::lexical_cast<bool>(appProperties.get<std::string>("Vulkan.EnableValidationLayers"))) {
			createInfo.enabledLayerCount = static_cast<uint32_t>(requiredValidationLayers.size());
			createInfo.ppEnabledLayerNames = requiredValidationLayers.data();

			// Add the debug funtion for "create instance" and "destroy instance"
			createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*)& createDebugFunctionInfo;
		}
		else {
			createInfo.enabledLayerCount = 0;
			createInfo.pNext = nullptr;
		}
		createInfo.enabledExtensionCount = static_cast<uint32_t>(requiredExtensions.size());
		createInfo.ppEnabledExtensionNames = requiredExtensions.data();
		VkResult result = vkCreateInstance(&createInfo, nullptr, &instance);
		if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create instance!");
		}
		BOOST_LOG_TRIVIAL(info) << "Created Vulkan instance.";


		// Set up vulkan debug function for validation layers 
		if (boost::lexical_cast<bool>(appProperties.get<std::string>("Vulkan.EnableValidationLayers"))) {
			auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
			if (func != nullptr) {
				if (func(instance, &createDebugFunctionInfo, nullptr, &vulkanDebugMessenger) != VK_SUCCESS) {
					throw std::runtime_error("Error creating Vulkan debug function");
				}
			}
			else {
				throw std::runtime_error("Error creating Vulkan debug function" + VK_ERROR_EXTENSION_NOT_PRESENT);
			}
			BOOST_LOG_TRIVIAL(info) << "Added vulkan validation layers debug function";
		}
	}

	/* Free all allocated resources */
	void cleanup() {

		if (boost::lexical_cast<bool>(appProperties.get<std::string>("Vulkan.EnableValidationLayers"))) {
			DestroyDebugUtilsMessengerEXT(instance, vulkanDebugMessenger, nullptr);
			BOOST_LOG_TRIVIAL(debug) << "Vulkan debug messeger destroyed.";
		}
		
		vkDestroyInstance(instance, nullptr);
		BOOST_LOG_TRIVIAL(debug) << "Vulkan instance destroyed.";
		
		glfwDestroyWindow(window);
		BOOST_LOG_TRIVIAL(debug) << "GLFW Windows destroyed.";
		
		glfwTerminate();
		BOOST_LOG_TRIVIAL(debug) << "GLFW terminated.";
		
		BOOST_LOG_TRIVIAL(debug) << "All resources freed.";
	}

	/* Application main loop */
	void mainLoop() {

		BOOST_LOG_TRIVIAL(debug) << "Entering mainloop";
		while (!glfwWindowShouldClose(window)) {
			glfwPollEvents();
		}
	}

};