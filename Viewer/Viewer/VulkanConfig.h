#pragma once

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


/* Stores valid ids for queue families */
struct QueueFamilyIds {

	std::optional<uint32_t> graphicsFamily;
	std::optional<uint32_t> presentFamily;

	bool isValid() {
		return graphicsFamily.has_value() && presentFamily.has_value();
	}
};

/* Store support details for swap chain */
struct SwapChainSupportDetails {
	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;
};


const int MAX_FRAMES_IN_FLIGHT = 2;

/* Utility class to setup and initialize Vulkan API */
class VulkanConfig
{

	// Vulkan instance	
	VkInstance instance;	

	// Physical device 
	VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;

	// The logical device
	VkDevice device;										

	// Handle to Vulkan debug function
	VkDebugUtilsMessengerEXT utilsMessenger;

	// Required queue family ids
	QueueFamilyIds requiredQueueFamilyIds;

	// The swap chain
	VkSwapchainKHR swapChain;
	
	// Swap chain features
	VkSurfaceFormatKHR swapChainSurfaceFormat;
	VkPresentModeKHR swapChainPresentMode;
	VkExtent2D swapChainExtent;

	// Handles to swap chain images
	std::vector<VkImage> swapChainImages;

	// Views in swap chain images
	std::vector<VkImageView> swapChainImageViews;

	VkRenderPass renderPass;
	VkPipeline graphicsPipeline;
	VkPipelineLayout pipelineLayout;
	std::vector<VkFramebuffer> swapChainFramebuffers;
	VkCommandPool commandPool;
	std::vector<VkCommandBuffer> commandBuffers;

	VkQueue graphicsQueue;
	VkQueue presentQueue;

	bool framebufferResized = false;

	/* Sync objects */
	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;
	size_t currentFrame = 0;


	void setupValidationLayers();
	void setupInstanceExtensions();
	void createInstance();
	void createGLFWSurface();
	void setupVulkanUtilsMessenger();
	void setupPhysicalDevice();		
	void setupLogicalDevice();
	
	bool isDeviceValid(const VkPhysicalDevice& device);									
	bool checkDeviceExtensionSupport(VkPhysicalDevice device);
	SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);
	void setupSwapChain();
	void setupImageViews();
	void createRenderPass();
	void createGraphicsPipeline();
	VkShaderModule createShaderModule(const std::vector<char>& code);
	void createFramebuffers();
	void createCommandPool();
	void createCommandBuffers();
	void createSyncObjects();
	void recreateSwapChain();
	void cleanupSwapChain();
	static void framebufferResizeCallback(GLFWwindow* window, int width, int height);




	// Vulkan debug callback 
	static VKAPI_ATTR VkBool32 VKAPI_CALL vulkanUtilsMessengerCallback(
		VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
		VkDebugUtilsMessageTypeFlagsEXT messageType,
		const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
		void* pUserData);

	
public:

	const char* applicationName;
	bool enableValidationLayers = false;
	
	// Rendering surface
	VkSurfaceKHR surface;

	// App window instance
	GLFWwindow* glfwWindow;										

	// If false the surface will not be created and the attribute surface have to be set from outside
	bool enableCreateGLFWSurface = true;
	
	std::vector<const char*> validationLayers;
	std::vector<const char*> instanceExtensions;
	std::vector<const char*> deviceExtensions;
	
	// A custom util messenger function could be specified from outside, replacing the one in VulkanConfig.cpp
	VkDebugUtilsMessengerCreateInfoEXT *createUtilsMessengerFunctionInfo = nullptr;

	VkInstance getInstance() { return instance; };

	void setUp();
	void drawFrame();
	void cleanUp();	
};

