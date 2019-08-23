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

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <optional>


class VulkanUtils
{

	void createInstance() {
		// Create Vulkan instance 
		VkApplicationInfo appInfo = {};
		appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		appInfo.pApplicationName = "Physic Engine Viewer";
		appInfo.applicationVersion = VK_MAKE_VERSION(0, 0, 1);
		appInfo.apiVersion = VK_API_VERSION_1_0;
		VkInstanceCreateInfo createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		createInfo.pApplicationInfo = &appInfo;
		// TODO Handle layers here ..
		createInfo.enabledLayerCount = 0;
		createInfo.enabledExtensionCount = glfwRequiredExtensionsCount;
		createInfo.ppEnabledExtensionNames = glfwRequiredExtensions;
		VkResult result = vkCreateInstance(&createInfo, nullptr, &instance);
		if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create instance!");
		}
		BOOST_LOG_TRIVIAL(info) << "Created Vulkan instance.";
	}


	void setUpLayers() {
		// Get all supported validation layers
		uint32_t layerCount;
		vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
		std::vector<VkLayerProperties> availableLayers(layerCount);
		vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

		// Check if requested validation layers are available
		if (boost::lexical_cast<int>(appProperties.get<std::string>("Vulkan.EnableValidationLayers"))) {

			for (const char* layerName : validationLayers) {
				bool layerFound = false;

				for (const auto& layerProperties : availableLayers) {
					if (strcmp(layerName, layerProperties.layerName) == 0) {
						layerFound = true;
						break;
					}
				}

				if (!layerFound) {
					throw std::runtime_error("validation layers requested, but not available!");
				}
			}

		}
	}

	void setUpExtensions() {
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
	}
	



		

		

	}

};

