#include "VulkanConfig.h"
#include "Utils.h"

using std::vector;


/*Vulkan create debug validation layer callback */
VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, 
	const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
	
	auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
	if (func != nullptr) {
		return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
	}
	else {
		return VK_ERROR_EXTENSION_NOT_PRESENT;
	}
}

/* Vulkan remove debug validation layer callback */
void destroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
	
	auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
	if (func != nullptr) {
		func(instance, debugMessenger, pAllocator);
	}
}

void VulkanConfig::setUp() {

	setupValidationLayers();
	setupInstanceExtensions();
	createInstance();
	setupVulkanUtilsMessenger();
	if (enableCreateGLFWSurface) createGLFWSurface();
	setupPhysicalDevice();
	setupLogicalDevice();
	setupSwapChain();
	setupImageViews();
}

void VulkanConfig::setupValidationLayers() {

	// Get all available layers
	uint32_t count;
	vkEnumerateInstanceLayerProperties(&count, nullptr);

	vector<VkLayerProperties> availableLayers(count);
	vkEnumerateInstanceLayerProperties(&count, availableLayers.data());
	
	BOOST_LOG_TRIVIAL(debug) << "Supported Vulkan Layers:";
	for (const auto& l : availableLayers) BOOST_LOG_TRIVIAL(debug) << " " << l.layerName;
	
	BOOST_LOG_TRIVIAL(debug) << "Required Vulkan Layers:";
	for (const auto& layer : validationLayers) BOOST_LOG_TRIVIAL(debug) << " " << layer;
	
	// Check that required layers are available
	std::for_each(validationLayers.begin(), validationLayers.end(),
		[availableLayers](const char* required)
		{
			if (std::find_if(availableLayers.begin(), availableLayers.end(),
				[required](const VkLayerProperties layer)
				{
					return (std::strcmp(layer.layerName, required) == 0);
				}) == availableLayers.end())
			{
				throw std::runtime_error("Requested Layer " + std::string(required) + " not available");
			}
		});
}


void VulkanConfig::setupInstanceExtensions() { 

	// Get all available extensions
	uint32_t count = 0;
	vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr);

	std::vector<VkExtensionProperties> extensions(count);
	vkEnumerateInstanceExtensionProperties(nullptr, &count, extensions.data());

	BOOST_LOG_TRIVIAL(debug) << "Supported Vulkan Extensions:";
	for (const auto& extension : extensions) BOOST_LOG_TRIVIAL(debug) << " " << extension.extensionName;

	// Check that the device support all required extensions
	bool extensionSupported = std::all_of(instanceExtensions.begin(), instanceExtensions.end(),
		[extensions](const char* required)
		{
			return std::find_if(extensions.begin(), extensions.end(),
				[required](const VkExtensionProperties& e)
				{
					return strcmp(required, e.extensionName) == 0;
				}) != extensions.end();
		}
	);

	if (extensionSupported) BOOST_LOG_TRIVIAL(info) << "All required extensions are supported ";
	else throw std::runtime_error("Not all required extensions are supported");
}


void VulkanConfig::createInstance() {

	/* Create the instance */
	VkApplicationInfo appInfo = {};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pApplicationName = applicationName;
	appInfo.applicationVersion = VK_MAKE_VERSION(0, 0, 1);
	appInfo.apiVersion = VK_API_VERSION_1_0;

	VkInstanceCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	createInfo.pApplicationInfo = &appInfo;

	if (enableValidationLayers) {
 		createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		createInfo.ppEnabledLayerNames = validationLayers.data();

		// Add the message util functions to debug also during "create instance" and "destroy instance" process
		createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*) createUtilsMessengerFunctionInfo;
	}
	else {
		createInfo.enabledLayerCount = 0;
		createInfo.pNext = nullptr;
	}

	createInfo.enabledExtensionCount = static_cast<uint32_t>(instanceExtensions.size());
	createInfo.ppEnabledExtensionNames = instanceExtensions.data();
	
	if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create instance!");
	}
	BOOST_LOG_TRIVIAL(info) << "Created Vulkan instance.";
}

/* Vulkan debug validation layer callback */
VKAPI_ATTR VkBool32 VKAPI_CALL VulkanConfig::vulkanUtilsMessengerCallback(
	VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
	VkDebugUtilsMessageTypeFlagsEXT messageType,
	const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
	void* pUserData) {

	BOOST_LOG_TRIVIAL(debug) << "Validation layer: " << pCallbackData->pMessage << ".";
	return VK_FALSE;
}


void VulkanConfig::setupVulkanUtilsMessenger() {

	if (enableValidationLayers) {
		
		// Setup the debug function that will receive the validation layer message
		if (createUtilsMessengerFunctionInfo == nullptr) {
			createUtilsMessengerFunctionInfo = new VkDebugUtilsMessengerCreateInfoEXT;
			createUtilsMessengerFunctionInfo->sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
			createUtilsMessengerFunctionInfo->messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
			createUtilsMessengerFunctionInfo->messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
			createUtilsMessengerFunctionInfo->pfnUserCallback = vulkanUtilsMessengerCallback;
			createUtilsMessengerFunctionInfo->pUserData = nullptr;
		}

		if (CreateDebugUtilsMessengerEXT(instance, createUtilsMessengerFunctionInfo, nullptr, &utilsMessenger) != VK_SUCCESS) {
			throw std::runtime_error("failed to set up debug messenger!");
		}
		BOOST_LOG_TRIVIAL(debug) << "Added vulkan validation layers debug function";
	}
}


void VulkanConfig::createGLFWSurface() {
	
	if (glfwCreateWindowSurface(instance, glfwWindow, nullptr, &surface) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create window surface");
	}
	BOOST_LOG_TRIVIAL(debug) << "Created GLFW surface";
}


void VulkanConfig::setupPhysicalDevice() { 

	uint32_t count = 0;
	vkEnumeratePhysicalDevices(instance, &count, nullptr);
	if (count == 0) {
		throw std::runtime_error("failed to find GPUs with Vulkan support!");
	}

	std::vector<VkPhysicalDevice> devices(count);
	vkEnumeratePhysicalDevices(instance, &count, devices.data());
	BOOST_LOG_TRIVIAL(info) << "Found " << count << " physical devices";

	// Look for a device that has all required family queues and support all the required extensions
	for (const auto& device : devices) {

		VkPhysicalDeviceProperties deviceProperties;
		vkGetPhysicalDeviceProperties(device, &deviceProperties);
		BOOST_LOG_TRIVIAL(info) << "Found device " << deviceProperties.deviceName;

		VkPhysicalDeviceFeatures deviceFeatures;
		vkGetPhysicalDeviceFeatures(device, &deviceFeatures);

		if (!isDeviceValid(device)) {
			break;
		}
		else {
			BOOST_LOG_TRIVIAL(info) << "The device " << deviceProperties.deviceName << " is suitable (has all required family queue)";
		}
		this->physicalDevice = device;
	}

	if (!requiredQueueFamilyIds.isValid()) {
		throw std::runtime_error("Cannot find a suitable device with all the required family queues");
	}
}


bool VulkanConfig::isDeviceValid(const VkPhysicalDevice& device) {

	// Check that requiered extensions are supported
	bool extensionsSupported = checkDeviceExtensionSupport(device);

	// Check that required family queues are supported
	uint32_t count = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);

	std::vector<VkQueueFamilyProperties> queueFamilies(count);
	vkGetPhysicalDeviceQueueFamilyProperties(device, &count, queueFamilies.data());

	for (auto const& e : queueFamilies | boost::adaptors::indexed(0))
	{
		// Check if the queue has valid graphic capacity
		if (e.value().queueFlags & VK_QUEUE_GRAPHICS_BIT) requiredQueueFamilyIds.graphicsFamily = static_cast<uint32_t>(e.index());

		// Check if it is a valid present family queue
		VkBool32 presentSupport = false;
		vkGetPhysicalDeviceSurfaceSupportKHR(device, static_cast<uint32_t>(e.index()), surface, &presentSupport);
		if (presentSupport) requiredQueueFamilyIds.presentFamily = static_cast<uint32_t>(e.index());
	}

	// Check that the swap chain is adeguate (it has at least one supported image format and one supported presentation mode)
	bool swapChainAdequate = false;
	if (extensionsSupported) {
		SwapChainSupportDetails swapChainSupport = querySwapChainSupport(device);
		swapChainAdequate = !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty();
	}

	return requiredQueueFamilyIds.isValid() && extensionsSupported && swapChainAdequate;
}


/* Check the device support all required extensions */
bool VulkanConfig::checkDeviceExtensionSupport(VkPhysicalDevice device) {

	uint32_t count = 0;
	vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);

	std::vector<VkExtensionProperties> extensions(count);
	vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());

	BOOST_LOG_TRIVIAL(debug) << "Supported Vulkan Extensions on device ";
	for (const auto& extension : extensions) {
		BOOST_LOG_TRIVIAL(debug) << " " << extension.extensionName;
	}

	// Check that the device support all required extensions
	bool extensionSupported = std::all_of(deviceExtensions.begin(), deviceExtensions.end(),
		[extensions](const char* required)
		{
			return std::find_if(extensions.begin(), extensions.end(),
				[required](const VkExtensionProperties& e)
				{
					return strcmp(required, e.extensionName) == 0;
				}) != extensions.end();
		}
	);

	return extensionSupported;
}


void VulkanConfig::setupLogicalDevice() {
	
	// Setup family queues
	std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
	std::set<uint32_t> uniqueQueueFamilies = { this->requiredQueueFamilyIds.graphicsFamily.value(), this->requiredQueueFamilyIds.presentFamily.value() };

	float queuePriority = 1.0f;
	for (uint32_t queueFamily : uniqueQueueFamilies) {
		VkDeviceQueueCreateInfo queueCreateInfo = {};
		queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreateInfo.queueFamilyIndex = queueFamily;
		queueCreateInfo.queueCount = 1;
		queueCreateInfo.pQueuePriorities = &queuePriority;
		queueCreateInfos.push_back(queueCreateInfo);
	}

	// Setup device features
	VkPhysicalDeviceFeatures deviceFeatures = {};

	// Create the logical device
	VkDeviceCreateInfo deviceCreateInfo = {};
	deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	deviceCreateInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
	deviceCreateInfo.pQueueCreateInfos = queueCreateInfos.data();
	deviceCreateInfo.pEnabledFeatures = &deviceFeatures;
	deviceCreateInfo.enabledExtensionCount = 1;
	deviceCreateInfo.ppEnabledExtensionNames = deviceExtensions.data();

	if (enableValidationLayers) {
		deviceCreateInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		deviceCreateInfo.ppEnabledLayerNames = validationLayers.data();
	}
	else {
		deviceCreateInfo.enabledLayerCount = 0;
	}

	if (vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &device) != VK_SUCCESS) {
		throw std::runtime_error("failed to create logical device!");
	}

	BOOST_LOG_TRIVIAL(info) << "Logical device created";
}


SwapChainSupportDetails VulkanConfig::querySwapChainSupport(VkPhysicalDevice device) {

	SwapChainSupportDetails details;
	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);

	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);
	if (formatCount != 0) {
		details.formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, details.formats.data());
	}

	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);
	if (presentModeCount != 0) {
		details.presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, details.presentModes.data());
	}

	return details;
}


/* Create a swap chan */
void VulkanConfig::setupSwapChain() {

	// Get swap chain supported features
	SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

	// Choose surface format
	swapChainSurfaceFormat = swapChainSupport.formats[0];
	for (const auto& availableFormat : swapChainSupport.formats) {
		if (availableFormat.format == VK_FORMAT_B8G8R8A8_UNORM && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
			swapChainSurfaceFormat = availableFormat;
		}
	}

	// Choose swap present mode
	swapChainPresentMode = VK_PRESENT_MODE_FIFO_KHR;
	for (const auto& availablePresentMode : swapChainSupport.presentModes) {
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
			swapChainPresentMode = availablePresentMode;
		}
	}

	// Choose swap extent
	swapChainExtent = { 800, 600 };
	if (swapChainSupport.capabilities.currentExtent.width != UINT32_MAX) {
		swapChainExtent = swapChainSupport.capabilities.currentExtent;
	}
	else {
		swapChainExtent.width = std::max(swapChainSupport.capabilities.minImageExtent.width,
			std::min(swapChainSupport.capabilities.maxImageExtent.width, swapChainExtent.width));
		swapChainExtent.height = std::max(swapChainSupport.capabilities.minImageExtent.height,
			std::min(swapChainSupport.capabilities.maxImageExtent.height, swapChainExtent.height));
	}

	// Set image cont
	uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
	if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount) {
		imageCount = swapChainSupport.capabilities.maxImageCount;
	}

	// Create the swap chain
	VkSwapchainCreateInfoKHR createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	createInfo.surface = surface;
	createInfo.minImageCount = imageCount;
	createInfo.imageFormat = swapChainSurfaceFormat.format;
	createInfo.imageColorSpace = swapChainSurfaceFormat.colorSpace;
	createInfo.imageExtent = swapChainExtent;
	createInfo.imageArrayLayers = 1;
	createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	uint32_t queueFamilyIndices[] = { requiredQueueFamilyIds.graphicsFamily.value(), requiredQueueFamilyIds.presentFamily.value() };
	if (requiredQueueFamilyIds.graphicsFamily != requiredQueueFamilyIds.presentFamily) {
		createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		createInfo.queueFamilyIndexCount = 2;
		createInfo.pQueueFamilyIndices = queueFamilyIndices;
	}
	else {
		createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
	}
	createInfo.preTransform = swapChainSupport.capabilities.currentTransform;
	createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
	createInfo.presentMode = swapChainPresentMode;
	createInfo.clipped = VK_TRUE;
	createInfo.oldSwapchain = VK_NULL_HANDLE;

	if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) {
		throw std::runtime_error("failed to create swap chain!");
	}
	BOOST_LOG_TRIVIAL(debug) << "Created swap chain";

	// Get handels to swap chain images
	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
	swapChainImages.resize(imageCount);
	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());
}


void VulkanConfig::setupImageViews() {

	swapChainImageViews.resize(swapChainImages.size());

	for (size_t i = 0; i < swapChainImages.size(); i++) {

		VkImageViewCreateInfo createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		createInfo.image = swapChainImages[i];
		createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		createInfo.format = swapChainSurfaceFormat.format;
		createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
		createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
		createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
		createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
		createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		createInfo.subresourceRange.baseMipLevel = 0;
		createInfo.subresourceRange.levelCount = 1;
		createInfo.subresourceRange.baseArrayLayer = 0;
		createInfo.subresourceRange.layerCount = 1;

		if (vkCreateImageView(device, &createInfo, nullptr, &swapChainImageViews[i]) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create image views");
		}
		BOOST_LOG_TRIVIAL(debug) << "Created image views";
	}
}


void VulkanConfig::createGraphicsPipeline() {
	auto vertShaderCode = Utils::readFile("shaders/vert.spv");
	auto fragShaderCode = Utils::readFile("shaders/frag.spv");

	VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
	VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

	VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
	vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
	vertShaderStageInfo.module = vertShaderModule;
	vertShaderStageInfo.pName = "main";

	VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
	fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
	fragShaderStageInfo.module = fragShaderModule;
	fragShaderStageInfo.pName = "main";

	VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

	vkDestroyShaderModule(device, fragShaderModule, nullptr);
	vkDestroyShaderModule(device, vertShaderModule, nullptr);
}


VkShaderModule VulkanConfig::createShaderModule(const std::vector<char>& code) {
	
	VkShaderModuleCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.codeSize = code.size();
	createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

	VkShaderModule shaderModule;
	if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
		throw std::runtime_error("failed to create shader module!");
	}

	return shaderModule;
}

void VulkanConfig::cleanUp() {

	for (auto imageView : swapChainImageViews) {
		vkDestroyImageView(device, imageView, nullptr);
	}

	vkDestroySwapchainKHR(device, swapChain, nullptr);
	vkDestroyDevice(device, nullptr);

	if (enableValidationLayers) {
		destroyDebugUtilsMessengerEXT(instance, utilsMessenger, nullptr);
	}

	vkDestroySurfaceKHR(instance, surface, nullptr);
	vkDestroyInstance(instance, nullptr);
}