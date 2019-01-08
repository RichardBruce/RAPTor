#pragma once

/* Standard headers */
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

/* Vulkan headers */
#include <vulkan/vulkan.h>

/* Sdl headers */
#include "SDL_vulkan.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

/* Common headers */
#include "base_camera.h"
#include "point_t.h"

/* Sdl Wrapper headers */
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"
#include "sdl_wrapper.h"


namespace raptor_vulkan
{
struct Vertex
{
    public:
        glm::vec3 pos;
        glm::vec3 colour;
        glm::vec2 texCoord;

        static VkVertexInputBindingDescription getBindingDescription()
        {
            VkVertexInputBindingDescription bindingDescription = {  };
            bindingDescription.binding = 0;
            bindingDescription.stride = sizeof(Vertex);
            bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
            return bindingDescription;
        }

        static std::array<VkVertexInputAttributeDescription, 3> getAttributeDescriptions()
        {
            std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions = {  };
            attributeDescriptions[0].binding = 0;
            attributeDescriptions[0].location = 0;
            attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributeDescriptions[0].offset = offsetof(Vertex, pos);

            attributeDescriptions[1].binding = 0;
            attributeDescriptions[1].location = 1;
            attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributeDescriptions[1].offset = offsetof(Vertex, colour);

            attributeDescriptions[2].binding = 0;
            attributeDescriptions[2].location = 2;
            attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
            attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

            return attributeDescriptions;
        }
};

struct InstanceData
{
    public:
        glm::vec4 col0;
        glm::vec4 col1;
        glm::vec4 col2;
        glm::vec4 col3;

        static VkVertexInputBindingDescription getBindingDescription()
        {
            VkVertexInputBindingDescription bindingDescription = {  };
            bindingDescription.binding = 1;
            bindingDescription.stride = sizeof(InstanceData);
            bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;
            return bindingDescription;
        }

        static std::array<VkVertexInputAttributeDescription, 4> getAttributeDescriptions()
        {
            std::array<VkVertexInputAttributeDescription, 4> attributeDescriptions = {  };
            attributeDescriptions[0].binding = 1;
            attributeDescriptions[0].location = 3;
            attributeDescriptions[0].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributeDescriptions[0].offset = offsetof(InstanceData, col0);

            attributeDescriptions[1].binding = 1;
            attributeDescriptions[1].location = 4;
            attributeDescriptions[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributeDescriptions[1].offset = offsetof(InstanceData, col1);

            attributeDescriptions[2].binding = 1;
            attributeDescriptions[2].location = 5;
            attributeDescriptions[2].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributeDescriptions[2].offset = offsetof(InstanceData, col2);

            attributeDescriptions[3].binding = 1;
            attributeDescriptions[3].location = 6;
            attributeDescriptions[3].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributeDescriptions[3].offset = offsetof(InstanceData, col3);

            return attributeDescriptions;
        }
};

struct UniformBufferObject
{
    glm::mat4 view;
    glm::mat4 proj;
};

struct push_constants
{
    glm::mat4 transform;
};


VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pCallback)
{
    auto func = (PFN_vkCreateDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
    if (func != nullptr)
    {
        return func(instance, pCreateInfo, pAllocator, pCallback);
    }
    else
    {
        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }
}

void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT callback, const VkAllocationCallbacks* pAllocator)
{
    auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
    if (func != nullptr)
    {
        func(instance, callback, pAllocator);
    }
}

class Vulkan
{
    public:
        Vulkan() : 
            mCamera(point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, -1.0f), 0.0001f),
            mSdlEventHandler(
                get_camera_event_handler(&mCamera),
                new std::map<SDL_WindowEventID, std::function<int (SDL_WindowEvent)>>(
                {
                    { SDL_WINDOWEVENT_RESIZED, [this](const SDL_WindowEvent) { return framebufferResized(); } }
                }),
                [this](const SDL_MouseMotionEvent event)
                {
                    const float rel_x = (event.xrel * M_PI * -0.25f) / (static_cast<float>(mWidth) * mCamera.speed());
                    const float rel_y = (event.yrel * M_PI * -0.25f) / (static_cast<float>(mHeight) * mCamera.speed());
                    mCamera.rotate_about(point_t(0.0f, 1.0f, 0.0f), rel_x);
                    mCamera.rotate_about(point_t(1.0f, 0.0f, 0.0f), rel_y);
                    // mCamera.pan(rel_x);
                    // mCamera.tilt(rel_y);
                    return 0;
                }), 
            mMsaaSamples(VK_SAMPLE_COUNT_1_BIT), 
            mPhysicalDevice(VK_NULL_HANDLE),
            mFramebufferResized(false)
        {  }

        void run()
        {
            initSdl();
            initVulkan();
            mainLoop();
            cleanup();
        }

    private:
        struct QueueFamilyIndices
        {
            std::optional<uint32_t> graphicsFamily;
            std::optional<uint32_t> presentFamily;
            
            bool isComplete() const
            {
                return graphicsFamily.has_value() && presentFamily.has_value();
            }
        };

        struct SwapChainSupportDetails
        {
            VkSurfaceCapabilitiesKHR        capabilities;
            std::vector<VkSurfaceFormatKHR> formats;
            std::vector<VkPresentModeKHR>   presentModes;
            VkSurfaceFormatKHR              chosenFormat;
            VkExtent2D                      chosenExtent;
        };

        void initSdl()
        {
            // Initialise and lock the screen
            if (sdl_set_up(&mSdlWindow, nullptr, nullptr, "Vulkan Demo", mWidth, mHeight, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE))
            {
                throw std::runtime_error("failed to start sdl");
            }

            SDL_SetRelativeMouseMode(SDL_TRUE);
        }

        int framebufferResized()
        {
            std::cout << "frame buffer explicitly resized" << std::endl;
            mFramebufferResized = true;
            return 0;
        }

        void initVulkan()
        {
            createInstance();
            createSurface();
            pickPhysicalDevice();
            createLogicalDevice();
            createSwapChain();
            createImageViews();
            createRenderPass();
            createDescriptorSetLayout();
            createGraphicsPipeline();
            createCommandPool();
            createColourResources();
            createDepthResources();
            createTextureImage();
            createTextureImageView();
            createTextureSampler();
            createFramebuffers();
            loadModel();
            createVertexBuffer();
            createIndexBuffer();
            createInstanceDataBuffer();
            createUniformBuffers();
            createDescriptorPool();
            createDescriptorSets();
            createCommandBuffers();
            createSyncObjects();
        }

        void recreateSwapChain()
        {
            // Wait while the window is minimised
            int width;
            int height;
            SDL_Vulkan_GetDrawableSize(mSdlWindow, &width, &height);
            while ((width == 0) && (height == 0))
            {
                mSdlEventHandler.wait_for_event();
                SDL_Vulkan_GetDrawableSize(mSdlWindow, &width, &height);
            }

            // Wait for inflight frames to finish
            vkDeviceWaitIdle(mDevice);

            // Delete the old swapchain
            cleanupSwapChain();

            // Make a new swapchain
            createSwapChain();
            createImageViews();
            createRenderPass();
            createColourResources();
            createDepthResources();
            createFramebuffers();
            createCommandBuffers();
        }

        bool checkValidationLayerSupport(const std::vector<const char *> &mValidationLayers)
        {
            uint32_t layerCount;
            vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

            std::vector<VkLayerProperties> availableLayers(layerCount);
            vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());
            for (const char* layerName : mValidationLayers)
            {
                bool layerFound = false;
                for (const auto& layerProperties : availableLayers)
                {
                    if (strcmp(layerName, layerProperties.layerName) == 0)
                    {
                        layerFound = true;
                        break;
                    }
                }

                if (!layerFound)
                {
                    return false;
                }
            }

            return true;
        }

        static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
            VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
            VkDebugUtilsMessageTypeFlagsEXT messageType,
            const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
            void* pUserData)
        {
            std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
            return VK_FALSE;
        }

        void createInstance()
        {
            // Check extension support
            uint32_t extensionCount = 0;
            vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

            std::vector<VkExtensionProperties> availableExtensions(extensionCount);
            vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, availableExtensions.data());
            std::cout << "Available extensions:" << std::endl;
            for (const auto extension : availableExtensions)
            {
                std::cout << '\t' << extension.extensionName << std::endl;
            }

            // Generic info
            VkApplicationInfo appInfo = {};
            appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
            appInfo.pApplicationName = "Hello Triangle";
            appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
            appInfo.pEngineName = "No Engine";
            appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
            appInfo.apiVersion = VK_API_VERSION_1_0;

            VkInstanceCreateInfo createInfo = {};
            createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
            createInfo.pApplicationInfo = &appInfo;

            // Get window extensions
            uint32_t windowExtensionCount = 0;
            if (!SDL_Vulkan_GetInstanceExtensions(mSdlWindow, &windowExtensionCount, nullptr))
            {
                throw std::runtime_error(SDL_GetError());
            }

            std::vector<const char*> extensions(windowExtensionCount, nullptr);
            SDL_Vulkan_GetInstanceExtensions(mSdlWindow, &windowExtensionCount, extensions.data());
            std::cout << "Window system requested " << windowExtensionCount << " extensions" << std::endl;
            for (const auto &e : extensions)
            {
                std::cout << "\t" << e << std::endl;
            }

            // Validation layers
#ifndef NDEBUG
            // Request debug extension
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

            // Request validations
            if (!checkValidationLayerSupport(mValidationLayers))
            {
                throw std::runtime_error("validation layers requested, but not available!");
            }
            std::cout << "Enabling validation layers" << std::endl;
            createInfo.enabledLayerCount = static_cast<uint32_t>(mValidationLayers.size());
            createInfo.ppEnabledLayerNames = mValidationLayers.data();
#else
            createInfo.enabledLayerCount = 0;
#endif

            // Final list of extension
            createInfo.enabledExtensionCount = extensions.size();
            createInfo.ppEnabledExtensionNames = extensions.data();

            // Create the instance
            if (vkCreateInstance(&createInfo, nullptr, &mInstance) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create instance!");
            }

#ifndef NDEBUG
            // Configure debug message extension
            VkDebugUtilsMessengerCreateInfoEXT createDebugInfo = {};
            createDebugInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
            createDebugInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
            createDebugInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
            createDebugInfo.pfnUserCallback = debugCallback;
            createDebugInfo.pUserData = this;
            if (CreateDebugUtilsMessengerEXT(mInstance, &createDebugInfo, nullptr, &mCallback) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to set up debug callback!");
            }
#endif
        }

        void createSurface()
        {
            if (!SDL_Vulkan_CreateSurface(mSdlWindow, mInstance,&mSurface))
            {
                throw std::runtime_error("failed to create window surface");
            }
        }

        QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device)
        {
            // Get supported queue families
            uint32_t queueFamilyCount = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

            // Check graphics operations are supported
            int i = 0;
            QueueFamilyIndices indices;
            std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());
            for (const auto& queueFamily : queueFamilies)
            {
                if (queueFamily.queueCount <= 0)
                {
                    continue;
                }

                if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT)
                {
                    indices.graphicsFamily = i;
                    std::cout << "Found graphics queue on family " << i << std::endl;
                }

                VkBool32 presentSupport = false;
                vkGetPhysicalDeviceSurfaceSupportKHR(device, i, mSurface, &presentSupport);
                if (presentSupport)
                {
                    indices.presentFamily = i;
                    std::cout << "Found present queue on family " << i << std::endl;
                }
                
                ++i;
            }

            return indices;
        }

        SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device)
        {
            // Get swap chain capabilties (number of images, size of image)
            SwapChainSupportDetails details;
            vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, mSurface, &details.capabilities);

            // Get swap chain formats (colour layout)
            uint32_t formatCount;
            vkGetPhysicalDeviceSurfaceFormatsKHR(device, mSurface, &formatCount, nullptr);

            details.formats.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(device, mSurface, &formatCount, details.formats.data());

            // Get swapchain present modes (how to switch the presented frame)
            uint32_t presentModeCount;
            vkGetPhysicalDeviceSurfacePresentModesKHR(device, mSurface, &presentModeCount, nullptr);

            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(device, mSurface, &presentModeCount, details.presentModes.data());

            return details;
        }

        bool isDeviceSuitable(VkPhysicalDevice device, const std::vector<const char *> extensions)
        {
            // Basic device information, name, type, supported vulkan version
            VkPhysicalDeviceProperties properties;
            vkGetPhysicalDeviceProperties(device, &properties);

            // Optional features, 64 bit floats, multi viewport rendering
            VkPhysicalDeviceFeatures features;
            vkGetPhysicalDeviceFeatures(device, &features);

            // Check extension support
            uint32_t extensionCount;
            vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

            std::vector<VkExtensionProperties> availableExtensions(extensionCount);
            vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

            std::set<std::string> requiredExtensions(extensions.begin(), extensions.end());
            for (const auto &extension : availableExtensions)
            {
                requiredExtensions.erase(extension.extensionName);
                std::cout << "Got device extension " << extension.extensionName << std::endl;
            }

            // Check swapchain support
            bool swapChainAdequate = false;
            if (requiredExtensions.empty())
            {
                const auto swapchainSupport = querySwapChainSupport(device);
                swapChainAdequate = !swapchainSupport.formats.empty() && !swapchainSupport.presentModes.empty();
            }

            return features.samplerAnisotropy && (properties.limits.maxPushConstantsSize >= sizeof(push_constants)) && findQueueFamilies(device).isComplete() && requiredExtensions.empty() && swapChainAdequate;
        }

        VkSampleCountFlagBits getMaxUsableSampleCount(const VkPhysicalDevice device)
        {
            VkPhysicalDeviceProperties properties;
            vkGetPhysicalDeviceProperties(device, &properties);

            const VkSampleCountFlags counts = properties.limits.framebufferColorSampleCounts & properties.limits.framebufferDepthSampleCounts;
            if (counts & VK_SAMPLE_COUNT_64_BIT) { return VK_SAMPLE_COUNT_64_BIT; }
            if (counts & VK_SAMPLE_COUNT_32_BIT) { return VK_SAMPLE_COUNT_32_BIT; }
            if (counts & VK_SAMPLE_COUNT_16_BIT) { return VK_SAMPLE_COUNT_16_BIT; }
            if (counts & VK_SAMPLE_COUNT_8_BIT) { return VK_SAMPLE_COUNT_8_BIT; }
            if (counts & VK_SAMPLE_COUNT_4_BIT) { return VK_SAMPLE_COUNT_4_BIT; }
            if (counts & VK_SAMPLE_COUNT_2_BIT) { return VK_SAMPLE_COUNT_2_BIT; }

            return VK_SAMPLE_COUNT_1_BIT;
        }

        void pickPhysicalDevice()
        {
            // Fetch devices
            uint32_t deviceCount = 0;
            vkEnumeratePhysicalDevices(mInstance, &deviceCount, nullptr);
            if (deviceCount == 0)
            {
                throw std::runtime_error("failed to find GPUs with Vulkan support!");
            }

            std::vector<VkPhysicalDevice> devices(deviceCount);
            vkEnumeratePhysicalDevices(mInstance, &deviceCount, devices.data());

            for (const auto& device : devices)
            {
                if (isDeviceSuitable(device, mDeviceExtensions))
                {
                    mPhysicalDevice = device;
                    mMsaaSamples = getMaxUsableSampleCount(device);
                    std::cout << "Multi sampling at " << mMsaaSamples << std::endl;
                    break;
                }
            }

            if (mPhysicalDevice == VK_NULL_HANDLE)
            {
                throw std::runtime_error("failed to find a suitable GPU!");
            }
        }

        void createLogicalDevice()
        {
            // Reuqested queues from the physical device
            const auto indices = findQueueFamilies(mPhysicalDevice);
            const std::set<uint32_t> uniqueQueueFamilies = { indices.graphicsFamily.value(), indices.presentFamily.value() };

            const float queuePriority = 1.0f;
            std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
            for (uint32_t queueFamily : uniqueQueueFamilies)
            {
                VkDeviceQueueCreateInfo queueCreateInfo = { };
                queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
                queueCreateInfo.queueFamilyIndex = queueFamily;
                queueCreateInfo.queueCount = 1;
                queueCreateInfo.pQueuePriorities = &queuePriority;
                queueCreateInfos.push_back(queueCreateInfo);
            }

            // Requested features from the physica device
            VkPhysicalDeviceFeatures deviceFeatures = {};
            deviceFeatures.samplerAnisotropy = VK_TRUE;

            // Create logical device
            VkDeviceCreateInfo createInfo = {};
            createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
            createInfo.pEnabledFeatures = &deviceFeatures;
            createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
            createInfo.pQueueCreateInfos = queueCreateInfos.data();
            createInfo.enabledExtensionCount = static_cast<uint32_t>(mDeviceExtensions.size());
            createInfo.ppEnabledExtensionNames = mDeviceExtensions.data();

#ifndef NDEBUG
            // Request validations
            createInfo.enabledLayerCount = static_cast<uint32_t>(mValidationLayers.size());
            createInfo.ppEnabledLayerNames = mValidationLayers.data();
#else
            createInfo.enabledLayerCount = 0;
#endif

            // Create logical device
            if (vkCreateDevice(mPhysicalDevice, &createInfo, nullptr, &mDevice) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create logical device");
            }

            // Retreive queues
            vkGetDeviceQueue(mDevice, indices.graphicsFamily.value(), 0, &mGraphicsQueue);
            vkGetDeviceQueue(mDevice, indices.presentFamily.value(), 0, &mPresentQueue);
        }

        VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &availableFormats)
        {
            if ((availableFormats.size() == 1) && (availableFormats[0].format == VK_FORMAT_UNDEFINED))
            {
                return { VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR };
            }

            for (const auto& availableFormat : availableFormats)
            {
                if ((availableFormat.format == VK_FORMAT_B8G8R8A8_UNORM) && (availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR))
                {
                    return availableFormat;
                }
            }

            return availableFormats[0];
        }

        VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR> &availablePresentModes)
        {
            VkPresentModeKHR bestMode = VK_PRESENT_MODE_FIFO_KHR;
            for (const auto& availablePresentMode : availablePresentModes)
            {
                if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR)
                {
                    return availablePresentMode;
                }
                else if (availablePresentMode == VK_PRESENT_MODE_IMMEDIATE_KHR)
                {
                    bestMode = availablePresentMode;
                }
            }

            return bestMode;
        }
        
        VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities)
        {
            if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
            {
                return capabilities.currentExtent;
            }
            else
            {
                int width;
                int height;
                SDL_Vulkan_GetDrawableSize(mSdlWindow, &width, &height);

                VkExtent2D actualExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
                actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
                actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));
                return actualExtent;
            }
        }

        void createSwapChain()
        {
            mSwapchainSupport = querySwapChainSupport(mPhysicalDevice);
            auto extent = chooseSwapExtent(mSwapchainSupport.capabilities);
            auto surfaceFormat = chooseSwapSurfaceFormat(mSwapchainSupport.formats);
            auto presentMode = chooseSwapPresentMode(mSwapchainSupport.presentModes);
            mSwapchainSupport.chosenFormat = surfaceFormat;
            mSwapchainSupport.chosenExtent = extent;

            uint32_t imageCount = mSwapchainSupport.capabilities.minImageCount + 1;
            if ((mSwapchainSupport.capabilities.maxImageCount > 0) && (imageCount > mSwapchainSupport.capabilities.maxImageCount))
            {
                imageCount = mSwapchainSupport.capabilities.maxImageCount;
            }

            VkSwapchainCreateInfoKHR createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
            createInfo.surface = mSurface;
            createInfo.minImageCount = imageCount;
            createInfo.imageFormat = surfaceFormat.format;
            createInfo.imageColorSpace = surfaceFormat.colorSpace;
            createInfo.imageExtent = extent;
            createInfo.imageArrayLayers = 1;
            createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
            createInfo.presentMode = presentMode;
            createInfo.clipped = VK_TRUE;
            createInfo.preTransform = mSwapchainSupport.capabilities.currentTransform;
            createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
            createInfo.oldSwapchain = VK_NULL_HANDLE;

            QueueFamilyIndices indices = findQueueFamilies(mPhysicalDevice);
            uint32_t queueFamilyIndices[] = { indices.graphicsFamily.value(), indices.presentFamily.value() };
            if (indices.graphicsFamily != indices.presentFamily)
            {
                createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
                createInfo.queueFamilyIndexCount = 2;
                createInfo.pQueueFamilyIndices = queueFamilyIndices;
            }
            else
            {
                createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
                createInfo.queueFamilyIndexCount = 0; // Optional
                createInfo.pQueueFamilyIndices = nullptr; // Optional
            }

            if (vkCreateSwapchainKHR(mDevice, &createInfo, nullptr, &mSwapChain) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create swapchain!");
            }

            vkGetSwapchainImagesKHR(mDevice, mSwapChain, &imageCount, nullptr);
            mSwapchainImages.resize(imageCount);
            vkGetSwapchainImagesKHR(mDevice, mSwapChain, &imageCount, mSwapchainImages.data());
        }

        VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, const int mipLevels)
        {
            VkImageViewCreateInfo createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            createInfo.image = image;
            createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            createInfo.format = format;
            createInfo.subresourceRange.aspectMask = aspectFlags;
            createInfo.subresourceRange.baseMipLevel = 0;
            createInfo.subresourceRange.levelCount = mipLevels;
            createInfo.subresourceRange.baseArrayLayer = 0;
            createInfo.subresourceRange.layerCount = 1;

            VkImageView imageView;
            if (vkCreateImageView(mDevice, &createInfo, nullptr, &imageView) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create image views!");
            }

            return imageView;
        }

        void createImageViews()
        {
            mSwapchainImageViews.resize(mSwapchainImages.size());
            for (int i = 0; i < mSwapchainImages.size(); ++i)
            {
                mSwapchainImageViews[i] = createImageView(mSwapchainImages[i], mSwapchainSupport.chosenFormat.format, VK_IMAGE_ASPECT_COLOR_BIT, 1);
            }
        }

        std::vector<char> readShader(const std::string& filename)
        {
            std::ifstream file(filename, std::ios::ate | std::ios::binary);
            if (!file.is_open())
            {
                throw std::runtime_error("failed to open file!");
            }

            const int fileSize = file.tellg();
            std::vector<char> buffer(fileSize);
            file.seekg(0);
            file.read(buffer.data(), fileSize);

            file.close();
            return buffer;
        }

        VkShaderModule createShaderModule(const std::vector<char> &code)
        {
            VkShaderModuleCreateInfo createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            createInfo.codeSize = code.size();
            createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

            VkShaderModule shaderModule;
            if (vkCreateShaderModule(mDevice, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create shader module!");
            }

            return shaderModule;
        }

        void createRenderPass()
        {
            // Colour attachment
            VkAttachmentDescription colorAttachment = {  };
            colorAttachment.format = mSwapchainSupport.chosenFormat.format;
            colorAttachment.samples = mMsaaSamples;
            colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            // Depth attachment
            VkAttachmentDescription depthAttachment = {  };
            depthAttachment.format = findDepthFormat();
            depthAttachment.samples = mMsaaSamples;
            depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;


            // Resolve attachment
            VkAttachmentDescription resolveAttachment = {  };
            resolveAttachment.format = mSwapchainSupport.chosenFormat.format;
            resolveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
            resolveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            resolveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            resolveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            resolveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            resolveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            resolveAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

            std::array<VkAttachmentDescription, 3> attachments = { colorAttachment, depthAttachment, resolveAttachment };

            // Subpasses
            VkAttachmentReference colorAttachmentRef = {  };
            colorAttachmentRef.attachment = 0;
            colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            VkAttachmentReference depthAttachmentRef = {  };
            depthAttachmentRef.attachment = 1;
            depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

            VkAttachmentReference resolveAttachmentRef = {  };
            resolveAttachmentRef.attachment = 2;
            resolveAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            VkSubpassDescription subpass = {  };
            subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
            subpass.colorAttachmentCount = 1;
            subpass.pColorAttachments = &colorAttachmentRef;
            subpass.pDepthStencilAttachment = &depthAttachmentRef;
            subpass.pResolveAttachments = &resolveAttachmentRef;

            // Dependencies
            VkSubpassDependency dependency = {  };
            dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
            dependency.dstSubpass = 0;
            dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            dependency.srcAccessMask = 0;
            dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

            // Create render pass
            VkRenderPassCreateInfo createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
            createInfo.attachmentCount = attachments.size();
            createInfo.pAttachments = attachments.data();
            createInfo.subpassCount = 1;
            createInfo.pSubpasses = &subpass;
            createInfo.dependencyCount = 1;
            createInfo.pDependencies = &dependency;
            if (vkCreateRenderPass(mDevice, &createInfo, nullptr, &mRenderPass) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create render pass!");
            }
        }

        void createDescriptorSetLayout()
        {
            // Unified buffer object layout binding
            VkDescriptorSetLayoutBinding uboLayoutBinding = { };
            uboLayoutBinding.binding = 0;
            uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            uboLayoutBinding.descriptorCount = 1;
            uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
            uboLayoutBinding.pImmutableSamplers = nullptr;

            VkDescriptorSetLayoutBinding samplerLayoutBinding = { };
            samplerLayoutBinding.binding = 1;
            samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            samplerLayoutBinding.descriptorCount = 1;
            samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
            samplerLayoutBinding.pImmutableSamplers = nullptr;

            std::array<VkDescriptorSetLayoutBinding, 2> bindings = { uboLayoutBinding, samplerLayoutBinding };

            VkDescriptorSetLayoutCreateInfo createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            createInfo.bindingCount = bindings.size();
            createInfo.pBindings = bindings.data();
            if (vkCreateDescriptorSetLayout(mDevice, &createInfo, nullptr, &mDescriptorSetLayout) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create descriptor set layout!");
            }
        }

        void createGraphicsPipeline()
        {
            // Load shader code
            const auto vertShaderCode = readShader("vert.spv");
            const auto fragShaderCode = readShader("frag.spv");
            std::cout << "Loaded " << vertShaderCode.size() << " of vertex shader and " << fragShaderCode.size() << " of fragment shader code" << std::endl;

            // Create vertex shader
            VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
            VkPipelineShaderStageCreateInfo vertCreateInfo = {  };
            vertCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            vertCreateInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
            vertCreateInfo.module = vertShaderModule;
            vertCreateInfo.pName = "main";

            // Create fragment shader
            VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);
            VkPipelineShaderStageCreateInfo fragCreateInfo = {  };
            fragCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            fragCreateInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            fragCreateInfo.module = fragShaderModule;
            fragCreateInfo.pName = "main";

            VkPipelineShaderStageCreateInfo shaderStages[] = { vertCreateInfo, fragCreateInfo };

            // Vertex input
            std::vector<VkVertexInputBindingDescription> bindingDescription(1, Vertex::getBindingDescription());
            bindingDescription.push_back(InstanceData::getBindingDescription());

            const auto vertexAttributeDescriptions(Vertex::getAttributeDescriptions());
            const auto instanceAttributeDescriptions(InstanceData::getAttributeDescriptions());
            std::vector<VkVertexInputAttributeDescription> attributeDescriptions(vertexAttributeDescriptions.begin(), vertexAttributeDescriptions.end());
            attributeDescriptions.insert(attributeDescriptions.end(), instanceAttributeDescriptions.begin(), instanceAttributeDescriptions.end());

            VkPipelineVertexInputStateCreateInfo vertexInputInfo = {  };
            vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
            vertexInputInfo.vertexBindingDescriptionCount = bindingDescription.size();
            vertexInputInfo.pVertexBindingDescriptions = bindingDescription.data();
            vertexInputInfo.vertexAttributeDescriptionCount = attributeDescriptions.size();
            vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

            // Primitive assembly
            VkPipelineInputAssemblyStateCreateInfo inputAssembly = { };
            inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
            inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
            inputAssembly.primitiveRestartEnable = VK_FALSE;

            // Viewport
            VkPipelineViewportStateCreateInfo viewportState = {  };
            viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
            viewportState.viewportCount = 1;
            viewportState.pViewports = nullptr; // Set with vkCmdSetViewport
            viewportState.scissorCount = 1;
            viewportState.pScissors = nullptr; // Set with vkCmdSetScissor

            // Rasterizer
            VkPipelineRasterizationStateCreateInfo rasterizer = {  };
            rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
            rasterizer.depthClampEnable = VK_FALSE;
            rasterizer.rasterizerDiscardEnable = VK_FALSE;
            rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
            rasterizer.lineWidth = 1.0f;
            rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
            rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
            rasterizer.depthBiasEnable = VK_FALSE;
            rasterizer.depthBiasConstantFactor = 0.0f; // Optional
            rasterizer.depthBiasClamp = 0.0f; // Optional
            rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

            // Multisampling
            VkPipelineMultisampleStateCreateInfo multisampling = {  };
            multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
            multisampling.sampleShadingEnable = VK_FALSE;
            multisampling.rasterizationSamples = mMsaaSamples;
            multisampling.minSampleShading = 1.0f; // Optional
            multisampling.pSampleMask = nullptr; // Optional
            multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
            multisampling.alphaToOneEnable = VK_FALSE; // Optional

            // Colour blending
            VkPipelineColorBlendAttachmentState colorBlendAttachment = { };
            colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
            colorBlendAttachment.blendEnable = VK_FALSE;
            colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE; // Optional
            colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
            colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD; // Optional
            colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE; // Optional
            colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
            colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD; // Optional

            VkPipelineColorBlendStateCreateInfo colorBlending = {  };
            colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            colorBlending.logicOpEnable = VK_FALSE;
            colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
            colorBlending.attachmentCount = 1;
            colorBlending.pAttachments = &colorBlendAttachment;
            colorBlending.blendConstants[0] = 0.0f; // Optional
            colorBlending.blendConstants[1] = 0.0f; // Optional
            colorBlending.blendConstants[2] = 0.0f; // Optional
            colorBlending.blendConstants[3] = 0.0f; // Optional

            // Dynamic state
            VkDynamicState dynamicStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };

            VkPipelineDynamicStateCreateInfo dynamicState = { };
            dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
            dynamicState.dynamicStateCount = 2;
            dynamicState.pDynamicStates = dynamicStates;

            // Depth stencil state
            VkPipelineDepthStencilStateCreateInfo depthStencil = {  };
            depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
            depthStencil.depthTestEnable = VK_TRUE;
            depthStencil.depthWriteEnable = VK_TRUE;
            depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
            depthStencil.depthBoundsTestEnable = VK_FALSE;
            depthStencil.minDepthBounds = 0.0f;
            depthStencil.maxDepthBounds = 1.0f;
            depthStencil.stencilTestEnable = VK_FALSE;
            depthStencil.front = {  };
            depthStencil.back = {  };

            // Push constant layout
            VkPushConstantRange pushConstantRange = {  };
            pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
            pushConstantRange.offset = 0;
            pushConstantRange.size = sizeof(push_constants);

            // Pipeline layout
            VkPipelineLayoutCreateInfo pipelinelayoutInfo = {  };
            pipelinelayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            pipelinelayoutInfo.setLayoutCount = 1;
            pipelinelayoutInfo.pSetLayouts = &mDescriptorSetLayout;
            pipelinelayoutInfo.pushConstantRangeCount = 1;
            pipelinelayoutInfo.pPushConstantRanges = &pushConstantRange;
            if (vkCreatePipelineLayout(mDevice, &pipelinelayoutInfo, nullptr, &mPipelineLayout) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create pipeline layout!");
            }

            // Create the pipeline
            VkGraphicsPipelineCreateInfo pipelineInfo = {  };
            pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            pipelineInfo.stageCount = 2;
            pipelineInfo.pStages = shaderStages;
            pipelineInfo.pVertexInputState = &vertexInputInfo;
            pipelineInfo.pInputAssemblyState = &inputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlending;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = mPipelineLayout;
            pipelineInfo.renderPass = mRenderPass;
            pipelineInfo.subpass = 0;
            pipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
            pipelineInfo.basePipelineIndex = -1; // Optional
            if (vkCreateGraphicsPipelines(mDevice, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &mGraphicsPipeline) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create graphics pipeline!");
            }

            vkDestroyShaderModule(mDevice, vertShaderModule, nullptr);
            vkDestroyShaderModule(mDevice, fragShaderModule, nullptr);
        }

        void createFramebuffers()
        {
            mSwapchainFramebuffers.resize(mSwapchainImageViews.size());
            for (size_t i = 0; i < mSwapchainImageViews.size(); ++i)
            {
                std::array<VkImageView, 3> attachments = { mColourImageView, mDepthImageView, mSwapchainImageViews[i] };

                VkFramebufferCreateInfo framebufferInfo = {  };
                framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
                framebufferInfo.renderPass = mRenderPass;
                framebufferInfo.attachmentCount = attachments.size();
                framebufferInfo.pAttachments = attachments.data();
                framebufferInfo.width = mSwapchainSupport.chosenExtent.width;
                framebufferInfo.height = mSwapchainSupport.chosenExtent.height;
                framebufferInfo.layers = 1;
                if (vkCreateFramebuffer(mDevice, &framebufferInfo, nullptr, &mSwapchainFramebuffers[i]) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create framebuffer!");
                }
            }
        }

        uint32_t findMemoryType(const uint32_t typeFilter, VkMemoryPropertyFlags properties)
        {
            VkPhysicalDeviceMemoryProperties memProperties;
            vkGetPhysicalDeviceMemoryProperties(mPhysicalDevice, &memProperties);
            for (uint32_t i = 0; i < memProperties.memoryTypeCount; ++i)
            {
                if ((typeFilter & (1 << i)) && ((memProperties.memoryTypes[i].propertyFlags & properties) == properties))
                {
                    return i;
                }
            }

            throw std::runtime_error("failed to find suitable memory type!");
        }

        void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory)
        {
            VkBufferCreateInfo bufferInfo = {  };
            bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            bufferInfo.size = size;
            bufferInfo.usage = usage;
            bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            if (vkCreateBuffer(mDevice, &bufferInfo, nullptr, &buffer) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create buffer!");
            }

            VkMemoryRequirements memRequirements;
            vkGetBufferMemoryRequirements(mDevice, buffer, &memRequirements);

            VkMemoryAllocateInfo allocInfo = {  };
            allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            allocInfo.allocationSize = memRequirements.size;
            allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);
            if (vkAllocateMemory(mDevice, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to allocate buffer memory!");
            }

            vkBindBufferMemory(mDevice, buffer, bufferMemory, 0);
        }

        void createImage(const uint32_t width, const uint32_t height, const int mipLevels, const VkSampleCountFlagBits samples, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage &image, VkDeviceMemory &imageMemory)
        {
            // Create image for texture
            VkImageCreateInfo imageInfo = {  };
            imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.extent.width = width;
            imageInfo.extent.height = height;
            imageInfo.extent.depth = 1;
            imageInfo.mipLevels = mipLevels;
            imageInfo.arrayLayers = 1;
            imageInfo.format = format;
            imageInfo.tiling = tiling;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageInfo.usage = usage;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.samples = samples;
            imageInfo.flags = 0;
            if (vkCreateImage(mDevice, &imageInfo, nullptr, &image) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create image!");
            }

            VkMemoryRequirements memRequirements;
            vkGetImageMemoryRequirements(mDevice, image, &memRequirements);

            VkMemoryAllocateInfo allocInfo = {};
            allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            allocInfo.allocationSize = memRequirements.size;
            allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            if (vkAllocateMemory(mDevice, &allocInfo, nullptr, &imageMemory) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to allocate image memory!");
            }

            vkBindImageMemory(mDevice, image, imageMemory, 0);
        }

        VkCommandBuffer beginSingleTimeCommands()
        {
            VkCommandBufferAllocateInfo allocInfo = {  };
            allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            allocInfo.commandPool = mTempCommandPool;
            allocInfo.commandBufferCount = 1;

            VkCommandBuffer commandBuffer;
            vkAllocateCommandBuffers(mDevice, &allocInfo, &commandBuffer);

            VkCommandBufferBeginInfo beginInfo = {  };
            beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            vkBeginCommandBuffer(commandBuffer, &beginInfo);

            return commandBuffer;
        }

        void endSingleTimeCommands(VkCommandBuffer commandBuffer)
        { 
            vkEndCommandBuffer(commandBuffer);

            VkSubmitInfo submitInfo = {  };
            submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            submitInfo.commandBufferCount = 1;
            submitInfo.pCommandBuffers = &commandBuffer;
            vkQueueSubmit(mGraphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
            vkQueueWaitIdle(mGraphicsQueue);

            vkFreeCommandBuffers(mDevice, mTempCommandPool, 1, &commandBuffer);
        }

        void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)
        {
            VkCommandBuffer commandBuffer = beginSingleTimeCommands();

            VkBufferCopy copyRegion = {  };
            copyRegion.srcOffset = 0;
            copyRegion.dstOffset = 0;
            copyRegion.size = size;
            vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

            endSingleTimeCommands(commandBuffer);
        }

        void copyBufferToImage(VkBuffer srcBuffer, VkImage dstImage, const uint32_t width, const uint32_t height)
        {
            VkCommandBuffer commandBuffer = beginSingleTimeCommands();

            VkBufferImageCopy copyRegion = {  };
            copyRegion.bufferOffset = 0;
            copyRegion.bufferRowLength = 0;
            copyRegion.bufferImageHeight = 0;

            copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copyRegion.imageSubresource.mipLevel = 0;
            copyRegion.imageSubresource.baseArrayLayer = 0;
            copyRegion.imageSubresource.layerCount = 1;

            copyRegion.imageOffset = { 0, 0, 0 };
            copyRegion.imageExtent = { width, height, 1, };
            vkCmdCopyBufferToImage(commandBuffer, srcBuffer, dstImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

            endSingleTimeCommands(commandBuffer);
        }

        void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, const int mipLevels)
        {
            VkCommandBuffer commandBuffer = beginSingleTimeCommands();

            VkImageMemoryBarrier barrier = {  };
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.oldLayout = oldLayout;
            barrier.newLayout = newLayout;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = image;
            barrier.subresourceRange.baseMipLevel = 0;
            barrier.subresourceRange.levelCount = mipLevels;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = 0;

            // Get aspect of destination layout
            if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
            {
                barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
                if (hasStencilComponent(format))
                {
                    barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
                }
            }
            else
            {
                barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            }

            VkPipelineStageFlags srcStage;
            VkPipelineStageFlags dstStage;
            if ((oldLayout == VK_IMAGE_LAYOUT_UNDEFINED) && (newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL))
            {
                barrier.srcAccessMask = 0;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
                dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            }
            else if ((oldLayout == VK_IMAGE_LAYOUT_UNDEFINED) && (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL))
            {
                barrier.srcAccessMask = 0;
                barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
                dstStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            }
            else if ((oldLayout == VK_IMAGE_LAYOUT_UNDEFINED) && (newLayout == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL))
            {
                barrier.srcAccessMask = 0;
                barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
                srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
                dstStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            }
            else if ((oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) && (newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL))
            {
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                srcStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
                dstStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            }
            else
            {
                throw std::runtime_error("unsupported layout transistion!");
            }

            vkCmdPipelineBarrier(commandBuffer, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);

            endSingleTimeCommands(commandBuffer);
        }

        VkFormat findSupportedFormat(const std::vector<VkFormat> &candidates, VkImageTiling tiling, VkFormatFeatureFlags features)
        {
            for (const auto format : candidates)
            {
                VkFormatProperties properties;
                vkGetPhysicalDeviceFormatProperties(mPhysicalDevice, format, &properties);
                if ((tiling == VK_IMAGE_TILING_LINEAR) && ((properties.linearTilingFeatures & features) == features))
                {
                    return format;
                }

                if ((tiling == VK_IMAGE_TILING_OPTIMAL) && ((properties.optimalTilingFeatures & features) == features))
                {
                    return format;
                }
            }

            throw std::runtime_error("failed to find supported format!");
        }

        VkFormat findDepthFormat()
        {
            return findSupportedFormat({ VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT }, VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
        }

        bool hasStencilComponent(VkFormat format)
        {
            return (format == VK_FORMAT_D32_SFLOAT_S8_UINT) || (format == VK_FORMAT_D24_UNORM_S8_UINT);
        }

        void createColourResources()
        {
            const VkFormat format = mSwapchainSupport.chosenFormat.format;
            createImage(mSwapchainSupport.chosenExtent.width, mSwapchainSupport.chosenExtent.height, 1, mMsaaSamples, format, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, mColourImage, mColourImageMemory);
            mColourImageView = createImageView(mColourImage, format, VK_IMAGE_ASPECT_COLOR_BIT, 1);
            transitionImageLayout(mColourImage, format, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, 1);
        }

        void createDepthResources()
        {
            const VkFormat depthFormat = findDepthFormat();
            createImage(mSwapchainSupport.chosenExtent.width, mSwapchainSupport.chosenExtent.height, 1, mMsaaSamples, depthFormat, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, mDepthImage, mDepthImageMemory);
            mDepthImageView = createImageView(mDepthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT, 1);
            transitionImageLayout(mDepthImage, depthFormat, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, 1);
        }

        void generateMipmaps(VkImage image, const VkFormat format, const int32_t width, const int32_t height, const uint32_t mipLevels)
        {
            VkFormatProperties properties;
            vkGetPhysicalDeviceFormatProperties(mPhysicalDevice, format, &properties);
            if (!(properties.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT))
            {
                throw std::runtime_error("texture image format does not support linear blitting");
            }

            VkCommandBuffer commandBuffer = beginSingleTimeCommands();

            VkImageMemoryBarrier barrier = {  };
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.image = image;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;
            barrier.subresourceRange.levelCount = 1;

            int32_t mipWidth = width;
            int32_t mipHeight = height;
            for (uint32_t i = 1; i < mipLevels; ++i)
            {
                // Last blitted level to transfer source
                barrier.subresourceRange.baseMipLevel = i - 1;
                barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

                // Blit
                VkImageBlit blit = {  };
                blit.srcOffsets[0] = { 0, 0, 0 };
                blit.srcOffsets[1] = { mipWidth, mipHeight, 1 };
                blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                blit.srcSubresource.mipLevel = i - 1;
                blit.srcSubresource.baseArrayLayer = 0;
                blit.srcSubresource.layerCount = 1;

                mipWidth = std::max(mipWidth >> 1, 1);
                mipHeight = std::max(mipHeight >> 1, 1);
                blit.dstOffsets[0] = { 0, 0, 0 };
                blit.dstOffsets[1] = { mipWidth, mipHeight, 1 };
                blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                blit.dstSubresource.mipLevel = i;
                blit.dstSubresource.baseArrayLayer = 0;
                blit.dstSubresource.layerCount = 1;
                vkCmdBlitImage(commandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

                barrier.subresourceRange.baseMipLevel = i - 1;
                barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
            }

            barrier.subresourceRange.baseMipLevel = mipLevels - 1;
            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

            endSingleTimeCommands(commandBuffer);
        }

        void createTextureImage()
        {
            int width;
            int height;
            int channels;
            stbi_uc* pixels = stbi_load(mTexturePath.c_str(), &width, &height, &channels, STBI_rgb_alpha);
            if (!pixels)
            {
                throw std::runtime_error("failed to load texture image!");
            }

            VkDeviceSize imageSize = width * height * 4;
            mMipLevels = static_cast<int>(std::floor(std::log2(std::max(width, height)))) + 1;
            std::cout << "Generating " << mMipLevels << " mip levels" << std::endl;

            VkBuffer stagingBuffer;
            VkDeviceMemory stagingBufferMemory;
            createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

            void *data;
            vkMapMemory(mDevice, stagingBufferMemory, 0, imageSize, 0, &data);
            memcpy(data, pixels, imageSize);
            vkUnmapMemory(mDevice, stagingBufferMemory);

            stbi_image_free(pixels);
            createImage(width, height, mMipLevels, VK_SAMPLE_COUNT_1_BIT, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, mTextureImage, mTextureImageMemory);
            transitionImageLayout(mTextureImage, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, mMipLevels);
            copyBufferToImage(stagingBuffer, mTextureImage, width, height);
            generateMipmaps(mTextureImage, VK_FORMAT_R8G8B8A8_UNORM, width, height, mMipLevels);

            vkDestroyBuffer(mDevice, stagingBuffer, nullptr);
            vkFreeMemory(mDevice, stagingBufferMemory, nullptr);
        }

        void createTextureImageView()
        {
            mTextureImageView = createImageView(mTextureImage, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_ASPECT_COLOR_BIT, mMipLevels);
        }

        void createTextureSampler()
        {
            VkSamplerCreateInfo createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
            createInfo.magFilter = VK_FILTER_LINEAR;
            createInfo.minFilter = VK_FILTER_LINEAR;
            createInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            createInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            createInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            createInfo.anisotropyEnable = VK_TRUE;
            createInfo.maxAnisotropy = 16;
            createInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
            createInfo.unnormalizedCoordinates = VK_FALSE;
            createInfo.compareEnable = VK_FALSE;
            createInfo.compareOp = VK_COMPARE_OP_ALWAYS;
            createInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
            createInfo.mipLodBias = 0.0f;
            createInfo.minLod = 0;
            createInfo.maxLod = static_cast<float>(mMipLevels);
            if (vkCreateSampler(mDevice, &createInfo, nullptr, &mTextureSampler) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create texture sampler!");
            }
        }

        void loadModel()
        {
            std::string err;
            std::string warn;
            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> materials;
            if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, mModelPath.c_str()))
            {
                throw std::runtime_error(err + warn);
            }

            std::unordered_map<uint64_t, int> uniqueIndices;
            for (const auto &shape : shapes)
            {
                for (const auto &index : shape.mesh.indices)
                {
                    Vertex vertex = {  };
                    vertex.colour = { 1.0f, 1.0f, 1.0f };
                    vertex.pos = 
                    {
                        attrib.vertices[(index.vertex_index * 3)],
                        attrib.vertices[(index.vertex_index * 3) + 1],
                        attrib.vertices[(index.vertex_index * 3) + 2]
                    };

                    vertex.texCoord = 
                    {
                        attrib.texcoords[(index.texcoord_index * 2)],
                        1.0f - attrib.texcoords[(index.texcoord_index * 2) + 1]
                    };

                    const auto eit = uniqueIndices.emplace((static_cast<uint64_t>(index.vertex_index) << 32) + index.texcoord_index, mVertices.size());
                    if (eit.second)
                    {
                        mVertices.push_back(vertex);
                    }

                    mIndices.push_back(eit.first->second);
                }
            }

            std::cout << "Loaded " << mVertices.size() << " vertices with " << mIndices.size() << " usages" << std::endl;
            mInstanceData.resize(mDrawInstances);
            {
                float rot_matrix[9];
                quaternion_t rot(point_t(0.0f, 0.0f, 1.0f), 1.5f);
                rot.rotation_matrix(rot_matrix);

                auto &instance = mInstanceData[0];
                instance.col0[0] = rot_matrix[0];
                instance.col0[1] = rot_matrix[1];
                instance.col0[2] = rot_matrix[2];
                instance.col0[3] = 0.0f;

                instance.col1[0] = rot_matrix[3];
                instance.col1[1] = rot_matrix[4];
                instance.col1[2] = rot_matrix[5];
                instance.col1[3] = 0.0f;

                instance.col2[0] = rot_matrix[6];
                instance.col2[1] = rot_matrix[7];
                instance.col2[2] = rot_matrix[8];
                instance.col2[3] = 0.0f;

                instance.col3[0] = 0.0f;
                instance.col3[1] = 0.0f;
                instance.col3[2] = 0.0f;
                instance.col3[3] = 1.0f;
            }
            {
                float rot_matrix[9];
                quaternion_t rot(point_t(0.0f, 0.0f, 1.0f), 2.5f);
                rot.rotation_matrix(rot_matrix);

                auto &instance = mInstanceData[1];
                instance.col0[0] = rot_matrix[0];
                instance.col0[1] = rot_matrix[1];
                instance.col0[2] = rot_matrix[2];
                instance.col0[3] = 0.0f;

                instance.col1[0] = rot_matrix[3];
                instance.col1[1] = rot_matrix[4];
                instance.col1[2] = rot_matrix[5];
                instance.col1[3] = 0.0f;

                instance.col2[0] = rot_matrix[6];
                instance.col2[1] = rot_matrix[7];
                instance.col2[2] = rot_matrix[8];
                instance.col2[3] = 0.0f;

                instance.col3[0] = 4.0f;
                instance.col3[1] = 0.0f;
                instance.col3[2] = 0.0f;
                instance.col3[3] = 1.0f;
            }
            {
                float rot_matrix[9];
                quaternion_t rot(point_t(0.0f, 0.0f, 1.0f), 0.7f);
                rot.rotation_matrix(rot_matrix);

                auto &instance = mInstanceData[2];
                instance.col0[0] = rot_matrix[0];
                instance.col0[1] = rot_matrix[1];
                instance.col0[2] = rot_matrix[2];
                instance.col0[3] = 0.0f;

                instance.col1[0] = rot_matrix[3];
                instance.col1[1] = rot_matrix[4];
                instance.col1[2] = rot_matrix[5];
                instance.col1[3] = 0.0f;

                instance.col2[0] = rot_matrix[6];
                instance.col2[1] = rot_matrix[7];
                instance.col2[2] = rot_matrix[8];
                instance.col2[3] = 0.0f;

                instance.col3[0] = 1.0f;
                instance.col3[1] = 3.0f;
                instance.col3[2] = 0.0f;
                instance.col3[3] = 1.0f;
            }
            {
                float rot_matrix[9];
                quaternion_t rot(point_t(0.0f, 0.0f, 1.0f), -1.25f);
                rot.rotation_matrix(rot_matrix);

                auto &instance = mInstanceData[3];
                instance.col0[0] = rot_matrix[0];
                instance.col0[1] = rot_matrix[1];
                instance.col0[2] = rot_matrix[2];
                instance.col0[3] = 0.0f;

                instance.col1[0] = rot_matrix[3];
                instance.col1[1] = rot_matrix[4];
                instance.col1[2] = rot_matrix[5];
                instance.col1[3] = 0.0f;

                instance.col2[0] = rot_matrix[6];
                instance.col2[1] = rot_matrix[7];
                instance.col2[2] = rot_matrix[8];
                instance.col2[3] = 0.0f;

                instance.col3[0] = -3.0f;
                instance.col3[1] = -4.0f;
                instance.col3[2] = 0.0f;
                instance.col3[3] = 1.0f;
            }
            {
                float rot_matrix[9];
                quaternion_t rot(point_t(0.0f, 0.0f, 1.0f), 0.5f);
                rot.rotation_matrix(rot_matrix);

                auto &instance = mInstanceData[4];
                instance.col0[0] = rot_matrix[0];
                instance.col0[1] = rot_matrix[1];
                instance.col0[2] = rot_matrix[2];
                instance.col0[3] = 0.0f;

                instance.col1[0] = rot_matrix[3];
                instance.col1[1] = rot_matrix[4];
                instance.col1[2] = rot_matrix[5];
                instance.col1[3] = 0.0f;

                instance.col2[0] = rot_matrix[6];
                instance.col2[1] = rot_matrix[7];
                instance.col2[2] = rot_matrix[8];
                instance.col2[3] = 0.0f;

                instance.col3[0] =  7.0f;
                instance.col3[1] = -3.0f;
                instance.col3[2] = 0.0f;
                instance.col3[3] = 1.0f;
            }
        }

        void createVertexBuffer()
        {
            VkBuffer stagingBuffer;
            VkDeviceMemory stagingBufferMemory;
            VkDeviceSize bufferSize = mVertices.size() * sizeof(Vertex);
            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

            void *data;
            vkMapMemory(mDevice, stagingBufferMemory, 0, bufferSize, 0, &data);
            memcpy(data, mVertices.data(), static_cast<size_t>(bufferSize));
            vkUnmapMemory(mDevice, stagingBufferMemory);

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, mVertexBuffer, mVertexBufferMemory);
            copyBuffer(stagingBuffer, mVertexBuffer, bufferSize);

            vkDestroyBuffer(mDevice, stagingBuffer, nullptr);
            vkFreeMemory(mDevice, stagingBufferMemory, nullptr);
        }

        void createIndexBuffer()
        {
            VkBuffer stagingBuffer;
            VkDeviceMemory stagingBufferMemory;
            VkDeviceSize bufferSize = mIndices.size() * sizeof(uint32_t);
            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

            void *data;
            vkMapMemory(mDevice, stagingBufferMemory, 0, bufferSize, 0, &data);
            memcpy(data, mIndices.data(), static_cast<size_t>(bufferSize));
            vkUnmapMemory(mDevice, stagingBufferMemory);

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, mIndexBuffer, mIndexBufferMemory);
            copyBuffer(stagingBuffer, mIndexBuffer, bufferSize);

            vkDestroyBuffer(mDevice, stagingBuffer, nullptr);
            vkFreeMemory(mDevice, stagingBufferMemory, nullptr);
        }

        void createInstanceDataBuffer()
        {
            VkBuffer stagingBuffer;
            VkDeviceMemory stagingBufferMemory;
            VkDeviceSize bufferSize = mInstanceData.size() * sizeof(InstanceData);
            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

            void *data;
            vkMapMemory(mDevice, stagingBufferMemory, 0, bufferSize, 0, &data);
            memcpy(data, mInstanceData.data(), static_cast<size_t>(bufferSize));
            vkUnmapMemory(mDevice, stagingBufferMemory);

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, mInstanceDataBuffer, mInstanceDataBufferMemory);
            copyBuffer(stagingBuffer, mInstanceDataBuffer, bufferSize);

            vkDestroyBuffer(mDevice, stagingBuffer, nullptr);
            vkFreeMemory(mDevice, stagingBufferMemory, nullptr);
        }

        void createUniformBuffers()
        {
            mUniformBuffers.resize(mSwapchainImages.size());
            mUniformBuffersMemory.resize(mSwapchainImages.size());
            VkDeviceSize bufferSize = sizeof(UniformBufferObject);
            for (size_t i = 0; i < mSwapchainImages.size(); ++i)
            {
                createBuffer(bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, mUniformBuffers[i], mUniformBuffersMemory[i]);
            }
        }

        void createDescriptorPool()
        {
            std::array<VkDescriptorPoolSize, 2> poolSizes = {  };
            poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            poolSizes[0].descriptorCount = mSwapchainImages.size();
            poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            poolSizes[1].descriptorCount = mSwapchainImages.size();

            VkDescriptorPoolCreateInfo createInfo = {  };
            createInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            createInfo.poolSizeCount = poolSizes.size();
            createInfo.pPoolSizes = poolSizes.data();
            createInfo.maxSets = mSwapchainImages.size();
            if (vkCreateDescriptorPool(mDevice, &createInfo, nullptr, &mDescriptorPool) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create descriptor pool!");
            }
        }

        void createDescriptorSets()
        {
            std::vector<VkDescriptorSetLayout> layouts(mSwapchainImages.size(), mDescriptorSetLayout);
            VkDescriptorSetAllocateInfo allocInfo = {  };
            allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            allocInfo.descriptorPool = mDescriptorPool;
            allocInfo.descriptorSetCount = mSwapchainImages.size();
            allocInfo.pSetLayouts = layouts.data();

            mDescriptorSets.resize(mSwapchainImages.size());
            if (vkAllocateDescriptorSets(mDevice, &allocInfo, mDescriptorSets.data()) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to allocate descriptor sets!");
            }

            for (size_t i = 0; i < mSwapchainImages.size(); ++i)
            {
                std::vector<VkDescriptorBufferInfo> bufferInfo;
                bufferInfo.resize(1);
                bufferInfo[0].buffer = mUniformBuffers[i];
                bufferInfo[0].offset = 0;
                bufferInfo[0].range = sizeof(UniformBufferObject);

                VkDescriptorImageInfo imageInfo = {  };
                imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                imageInfo.imageView = mTextureImageView;
                imageInfo.sampler = mTextureSampler;

                std::array<VkWriteDescriptorSet, 2> descriptorWrites = {  };
                descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                descriptorWrites[0].dstSet = mDescriptorSets[i];
                descriptorWrites[0].dstBinding = 0;
                descriptorWrites[0].dstArrayElement = 0;
                descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                descriptorWrites[0].descriptorCount = bufferInfo.size();
                descriptorWrites[0].pBufferInfo = bufferInfo.data();
                descriptorWrites[0].pImageInfo = nullptr;
                descriptorWrites[0].pTexelBufferView = nullptr;

                descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                descriptorWrites[1].dstSet = mDescriptorSets[i];
                descriptorWrites[1].dstBinding = 1;
                descriptorWrites[1].dstArrayElement = 0;
                descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                descriptorWrites[1].descriptorCount = 1;
                descriptorWrites[1].pImageInfo = &imageInfo;
                descriptorWrites[1].pTexelBufferView = nullptr;
                vkUpdateDescriptorSets(mDevice, descriptorWrites.size(), descriptorWrites.data(), 0, nullptr);
            }
        }

        void createCommandPool()
        {
            auto queueFamilyIndices = findQueueFamilies(mPhysicalDevice);

            VkCommandPoolCreateInfo commandPoolInfo = {  };
            commandPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            commandPoolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();
            commandPoolInfo.flags = 0;
            if (vkCreateCommandPool(mDevice, &commandPoolInfo, nullptr, &mCommandPool) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create command pool!");
            }

            VkCommandPoolCreateInfo tempCommandPoolInfo = {  };
            tempCommandPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            tempCommandPoolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();
            tempCommandPoolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
            if (vkCreateCommandPool(mDevice, &tempCommandPoolInfo, nullptr, &mTempCommandPool) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create temporary command pool!");
            }
        }

        void createCommandBuffers()
        {
            // Allocate command bufers
            mCommandBuffers.resize(mSwapchainFramebuffers.size());

            VkCommandBufferAllocateInfo allocInfo = {  };
            allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            allocInfo.commandPool = mCommandPool;
            allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            allocInfo.commandBufferCount = static_cast<uint32_t>(mCommandBuffers.size());
            if (vkAllocateCommandBuffers(mDevice, &allocInfo, mCommandBuffers.data()) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to allocate command buffers!");
            }

            // Record command buffers
            for (size_t i = 0; i < mCommandBuffers.size(); ++i)
            {
                // Begin record
                VkCommandBufferBeginInfo beginInfo = {  };
                beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
                beginInfo.pInheritanceInfo = nullptr; // Optional

                if (vkBeginCommandBuffer(mCommandBuffers[i], &beginInfo) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to begin recording command buffer!");
                }

                VkRenderPassBeginInfo renderPassInfo = {  };
                renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
                renderPassInfo.renderPass = mRenderPass;
                renderPassInfo.framebuffer = mSwapchainFramebuffers[i];
                renderPassInfo.renderArea.offset = {0, 0};
                renderPassInfo.renderArea.extent = mSwapchainSupport.chosenExtent;

                std::array<VkClearValue, 2> clearColors = {  };
                clearColors[0] = { 0.0f, 0.0f, 0.0f, 1.0f };
                clearColors[1] = { 1.0f, 0 };
                renderPassInfo.clearValueCount = clearColors.size();
                renderPassInfo.pClearValues = clearColors.data();
                vkCmdBeginRenderPass(mCommandBuffers[i], &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
                vkCmdBindPipeline(mCommandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, mGraphicsPipeline);

                VkViewport viewport = {  };
                viewport.x = 0.0f;
                viewport.y = 0.0f;
                viewport.width = static_cast<float>(mSwapchainSupport.chosenExtent.width);
                viewport.height = static_cast<float>(mSwapchainSupport.chosenExtent.height);
                viewport.minDepth = 0.0f;
                viewport.maxDepth = 1.0f;
                vkCmdSetViewport(mCommandBuffers[i], 0, 1, &viewport);

                VkRect2D scissor = {  };
                scissor.offset = { 0, 0 };
                scissor.extent = mSwapchainSupport.chosenExtent;
                vkCmdSetScissor(mCommandBuffers[i], 0, 1, &scissor);

                VkDeviceSize offsets[] = { 0, 0 };
                VkBuffer vertexBuffers[] = { mVertexBuffer, mInstanceDataBuffer };
                vkCmdBindVertexBuffers(mCommandBuffers[i], 0, 2, vertexBuffers, offsets);
         
                vkCmdBindIndexBuffer(mCommandBuffers[i], mIndexBuffer, 0, VK_INDEX_TYPE_UINT32);

                vkCmdBindDescriptorSets(mCommandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSets[i], 0, nullptr);

                vkCmdDrawIndexed(mCommandBuffers[i], static_cast<uint32_t>(mIndices.size()), mDrawInstances, 0, 0, 0);
                vkCmdEndRenderPass(mCommandBuffers[i]);
                if (vkEndCommandBuffer(mCommandBuffers[i]) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to record command buffer");
                }
            }
        }

        void createSyncObjects()
        {
            mImageAvailableSemaphore.resize(mMaxFramesInflight);
            mRenderFinishedSemaphore.resize(mMaxFramesInflight);
            mInflightFences.resize(mMaxFramesInflight);

            VkSemaphoreCreateInfo semaphoreInfo = {  };
            semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

            VkFenceCreateInfo fenceInfo = {  };
            fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
            fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            for (int i = 0; i < mMaxFramesInflight; ++i)
            {
                if ((vkCreateSemaphore(mDevice, &semaphoreInfo, nullptr, &mImageAvailableSemaphore[i]) != VK_SUCCESS) ||
                    (vkCreateSemaphore(mDevice, &semaphoreInfo, nullptr, &mRenderFinishedSemaphore[i]) != VK_SUCCESS) ||
                    (vkCreateFence(mDevice, &fenceInfo, nullptr, &mInflightFences[i]) != VK_SUCCESS))
                {
                    throw std::runtime_error("failed to create semaphores!");
                }
            }
        }

        void mainLoop()
        {
            int do_next = 0;
            mCurrentFrame = 0;
            const auto start_time = std::chrono::system_clock::now();
            while (do_next != 1)
            {
                do_next = mSdlEventHandler.process_events();
                if (do_next == 0)
                {
                    const auto now = std::chrono::system_clock::now();
                    const float delta_time = std::chrono::duration<float, std::chrono::seconds::period>(now - start_time).count();
                    mSdlEventHandler.process_down_keys(delta_time);
                    drawFrame();
                }
            }

            vkDeviceWaitIdle(mDevice);
        }

        void drawFrame()
        {
            // Wait for a free inflight frame
            vkWaitForFences(mDevice, 1, &mInflightFences[mCurrentFrame], VK_TRUE, std::numeric_limits<uint64_t>::max());

            // Get swap image
            uint32_t imageIndex;
            const auto acquireResult = vkAcquireNextImageKHR(mDevice, mSwapChain, std::numeric_limits<uint64_t>::max(), mImageAvailableSemaphore[mCurrentFrame], VK_NULL_HANDLE, &imageIndex);
            if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR)
            {
                std::cout << "swap chain out of date on image acquire" << std::endl;
                recreateSwapChain();
                return;
            }
            else if ((acquireResult != VK_SUCCESS) && (acquireResult != VK_SUBOPTIMAL_KHR))
            {
                throw std::runtime_error("failed to acquire swap chain image!");
            }

            // Send globals
            updateUniformBuffers(imageIndex);

            // Submit render command to queue
            VkSubmitInfo submitInfo = {  };
            submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

            VkSemaphore waitSemaphores[] = { mImageAvailableSemaphore[mCurrentFrame] };
            VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
            submitInfo.waitSemaphoreCount = 1;
            submitInfo.pWaitSemaphores = waitSemaphores;
            submitInfo.pWaitDstStageMask = waitStages;

            submitInfo.commandBufferCount = 1;
            submitInfo.pCommandBuffers = &mCommandBuffers[imageIndex];

            VkSemaphore signalSemaphores[] = { mRenderFinishedSemaphore[mCurrentFrame] };
            submitInfo.signalSemaphoreCount = 1;
            submitInfo.pSignalSemaphores = signalSemaphores;

            vkResetFences(mDevice, 1, &mInflightFences[mCurrentFrame]);
            if (vkQueueSubmit(mGraphicsQueue, 1, &submitInfo, mInflightFences[mCurrentFrame]) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to submit draw command buffer!");
            }

            // Return image to swapchain
            VkPresentInfoKHR presentInfo = {  };
            presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
            presentInfo.waitSemaphoreCount = 1;
            presentInfo.pWaitSemaphores = signalSemaphores;

            VkSwapchainKHR swapChains[] = { mSwapChain };
            presentInfo.swapchainCount = 1;
            presentInfo.pSwapchains = swapChains;
            presentInfo.pImageIndices = &imageIndex;
            presentInfo.pResults = nullptr;
            const auto presentResult = vkQueuePresentKHR(mPresentQueue, &presentInfo);
            if ((presentResult == VK_ERROR_OUT_OF_DATE_KHR) || (presentResult == VK_SUBOPTIMAL_KHR) || mFramebufferResized)
            {
                std::cout << "swap chain out of date on image present" << std::endl;
                mFramebufferResized = false;
                recreateSwapChain();
                return;
            }
            else if (presentResult != VK_SUCCESS)
            {
                throw std::runtime_error("failed to acquire swap chain image!");
            }

            mCurrentFrame = (mCurrentFrame + 1) % mMaxFramesInflight;
        }

        void updateUniformBuffers(const uint32_t imageIndex)
        {
            UniformBufferObject ubo = {  };
            /* Perspective */
            {
                const float z_near = 0.1f;
                const float z_far = 1000.0f;
                const float cot_fov = 1.0f / tan(M_PI * 0.25f * 0.5f);
                const float aspect_inv = mSwapchainSupport.chosenExtent.height / static_cast<float>(mSwapchainSupport.chosenExtent.width);
                ubo.proj[0][0] = cot_fov * aspect_inv;
                ubo.proj[0][1] = 0.0f;
                ubo.proj[0][2] = 0.0f;
                ubo.proj[0][3] = 0.0f;

                ubo.proj[1][0] = 0.0f;
                ubo.proj[1][1] = -cot_fov;
                ubo.proj[1][2] = 0.0f;
                ubo.proj[1][3] = 0.0f;

                ubo.proj[2][0] = 0.0f;
                ubo.proj[2][1] = 0.0f;
                ubo.proj[2][2] = (z_near + z_far) / (z_near - z_far);
                ubo.proj[2][3] = -1.0f;

                ubo.proj[3][0] = 0.0f;
                ubo.proj[3][1] = 0.0f;
                ubo.proj[3][2] = (2.0f * z_near * z_far) / (z_near - z_far);
                ubo.proj[3][3] = 0.0f;
            }

            ubo.view = glm::lookAt(glm::vec3(mCamera.camera_position().x, mCamera.camera_position().y, mCamera.camera_position().z), glm::vec3(mCamera.camera_position().x + mCamera.z_axis().x, mCamera.camera_position().y + mCamera.z_axis().y, mCamera.camera_position().z + mCamera.z_axis().z), glm::vec3(mCamera.y_axis().x, mCamera.y_axis().y, mCamera.y_axis().z));

            void *data;
            const int bufferSize = sizeof(UniformBufferObject);
            vkMapMemory(mDevice, mUniformBuffersMemory[imageIndex], 0, bufferSize, 0, &data);
            memcpy(data, &ubo, bufferSize);
            vkUnmapMemory(mDevice, mUniformBuffersMemory[imageIndex]);
        }

        void updatePushConstants()
        {
            static auto startTime = std::chrono::system_clock::now();
            const auto now = std::chrono::system_clock::now();
            const float time = std::chrono::duration<float, std::chrono::seconds::period>(now - startTime).count();
            
            glm::mat4 model = glm::rotate(glm::mat4(1.0f), time * glm::radians(20.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            glm::mat4 view = glm::lookAt(glm::vec3(2.0f, 2.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            glm::mat4 proj = glm::perspective(glm::radians(45.0f), mSwapchainSupport.chosenExtent.width / static_cast<float>(mSwapchainSupport.chosenExtent.height), 0.1f, 10.0f);
            proj[1][1] *= -1.0f;
            mPushConstants.transform = proj * view * model;

            // vkCmdPushConstants(mCommandBuffers[i], mPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push_constants), &mPushConstants);
        }

        void cleanupSwapChain()
        {
            // Clean up colour attachment
            vkDestroyImageView(mDevice, mColourImageView, nullptr);
            vkDestroyImage(mDevice, mColourImage, nullptr);
            vkFreeMemory(mDevice, mColourImageMemory, nullptr);

            // Clean up depth stencil attachment
            vkDestroyImageView(mDevice, mDepthImageView, nullptr);
            vkDestroyImage(mDevice, mDepthImage, nullptr);
            vkFreeMemory(mDevice, mDepthImageMemory, nullptr);

            // Clean up swapchain frame buffers
            for (auto &swapchainFramebuffer : mSwapchainFramebuffers)
            {
                vkDestroyFramebuffer(mDevice, swapchainFramebuffer, nullptr);
            }

            // Free command buffers
            vkFreeCommandBuffers(mDevice, mCommandPool, static_cast<uint32_t>(mCommandBuffers.size()), mCommandBuffers.data());

            // Clean up render pass
            vkDestroyRenderPass(mDevice, mRenderPass, nullptr);

            // Clean up swapchain image views
            for (auto &swapchainImageView : mSwapchainImageViews)
            {
                vkDestroyImageView(mDevice, swapchainImageView, nullptr);
            }

            // Clean up swapchain
            vkDestroySwapchainKHR(mDevice, mSwapChain, nullptr);
        }

        void cleanup()
        {
            // Clean up swapchain
            cleanupSwapChain();

            // Clean up semaphores
            for (int i = 0; i < mMaxFramesInflight; ++i)
            {
                vkDestroySemaphore(mDevice, mImageAvailableSemaphore[i], nullptr);
                vkDestroySemaphore(mDevice, mRenderFinishedSemaphore[i], nullptr);
                vkDestroyFence(mDevice, mInflightFences[i], nullptr);
            }

            // Clean up command pool
            vkDestroyCommandPool(mDevice, mCommandPool, nullptr);
            vkDestroyCommandPool(mDevice, mTempCommandPool, nullptr);

            // Clean up vertex and index buffer
            vkDestroyBuffer(mDevice, mVertexBuffer, nullptr);
            vkDestroyBuffer(mDevice, mIndexBuffer, nullptr);
            vkDestroyBuffer(mDevice, mInstanceDataBuffer, nullptr);

            // Clean up vertex and index buffer memory
            vkFreeMemory(mDevice, mVertexBufferMemory, nullptr);
            vkFreeMemory(mDevice, mIndexBufferMemory, nullptr);
            vkFreeMemory(mDevice, mInstanceDataBufferMemory, nullptr);

            // Clean up texture sampler
            vkDestroySampler(mDevice, mTextureSampler, nullptr);

            // Clean up texture image
            vkDestroyImageView(mDevice, mTextureImageView, nullptr);
            vkDestroyImage(mDevice, mTextureImage, nullptr);
            vkFreeMemory(mDevice, mTextureImageMemory, nullptr);

            // Clean up descriptor pool
            vkDestroyDescriptorPool(mDevice, mDescriptorPool, nullptr);

            // Clean up descriptor set layouts
            vkDestroyDescriptorSetLayout(mDevice, mDescriptorSetLayout, nullptr);

            // Clean up uniform buffers
            for (size_t i = 0; i < mSwapchainImages.size(); ++i)
            {
                vkDestroyBuffer(mDevice, mUniformBuffers[i], nullptr);
                vkFreeMemory(mDevice, mUniformBuffersMemory[i], nullptr);
            }

            // Clean up pipeline
            vkDestroyPipeline(mDevice, mGraphicsPipeline, nullptr);

            // Clean up pipeline layout
            vkDestroyPipelineLayout(mDevice, mPipelineLayout, nullptr);

            // Clean up logical device
            vkDestroyDevice(mDevice, nullptr);

#ifndef NDEBUG
            // Clean up validation layer messenger
            DestroyDebugUtilsMessengerEXT(mInstance, mCallback, nullptr);
#endif
            // Destroy surface
            vkDestroySurfaceKHR(mInstance, mSurface, nullptr);

            // Clean up vulkan
            vkDestroyInstance(mInstance, nullptr);

            // SDL clean up
            sdl_clean_up(mSdlWindow, nullptr, nullptr, nullptr);
        }


        base_camera                     mCamera;
        SDL_Window *                    mSdlWindow;
        sdl_event_handler               mSdlEventHandler;
        VkInstance                      mInstance;
        VkDebugUtilsMessengerEXT        mCallback;
        VkSurfaceKHR                    mSurface;
        VkSampleCountFlagBits           mMsaaSamples;
        VkPhysicalDevice                mPhysicalDevice;
        VkDevice                        mDevice;
        VkQueue                         mGraphicsQueue;
        VkQueue                         mPresentQueue;
        SwapChainSupportDetails         mSwapchainSupport;
        VkSwapchainKHR                  mSwapChain;
        std::vector<VkImage>            mSwapchainImages;
        std::vector<VkImageView>        mSwapchainImageViews;
        std::vector<VkFramebuffer>      mSwapchainFramebuffers;
        VkRenderPass                    mRenderPass;
        VkDescriptorSetLayout           mDescriptorSetLayout;
        VkPipelineLayout                mPipelineLayout;
        VkPipeline                      mGraphicsPipeline;
        VkCommandPool                   mCommandPool;
        VkCommandPool                   mTempCommandPool;
        VkBuffer                        mVertexBuffer;
        VkBuffer                        mIndexBuffer;
        VkDeviceMemory                  mVertexBufferMemory;
        VkDeviceMemory                  mIndexBufferMemory;
        VkBuffer                        mInstanceDataBuffer;
        VkDeviceMemory                  mInstanceDataBufferMemory;
        VkImage                         mTextureImage;
        VkImageView                     mTextureImageView;
        VkDeviceMemory                  mTextureImageMemory;
        VkSampler                       mTextureSampler;
        VkImage                         mColourImage;
        VkImageView                     mColourImageView;
        VkDeviceMemory                  mColourImageMemory;
        VkImage                         mDepthImage;
        VkImageView                     mDepthImageView;
        VkDeviceMemory                  mDepthImageMemory;
        push_constants                  mPushConstants;
        std::vector<VkBuffer>           mUniformBuffers;
        std::vector<VkDeviceMemory>     mUniformBuffersMemory;
        VkDescriptorPool                mDescriptorPool;
        std::vector<VkDescriptorSet>    mDescriptorSets;
        std::vector<VkCommandBuffer>    mCommandBuffers;
        std::vector<VkSemaphore>        mImageAvailableSemaphore;
        std::vector<VkSemaphore>        mRenderFinishedSemaphore;
        std::vector<VkFence>            mInflightFences;
        std::vector<Vertex>             mVertices;
        std::vector<uint32_t>           mIndices;
        std::vector<InstanceData>       mInstanceData;
        const std::vector<const char*>  mValidationLayers = { "VK_LAYER_LUNARG_standard_validation" };
        const std::vector<const char *> mDeviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
        const std::string               mModelPath = "models/chalet.obj";
        const std::string               mTexturePath = "textures/chalet.jpg";
        const uint32_t                  mWidth = 800;
        const uint32_t                  mHeight = 600;
        const uint32_t                  mMaxFramesInflight = 2;
        const int                       mDrawInstances = 5;
        int                             mMipLevels;
        int                             mCurrentFrame;
        bool                            mFramebufferResized;
};
}
