#include "base/PreDefine.h"
#include <vulkan/vulkan.hpp>
#include <GLFW/glfw3.h>
#include <optional>
#include <glm/glm.hpp>

#include <opencv2/opencv.hpp>


#define VK_CHECK_RESULT(f)																				\
{																										\
	VkResult res = (f);																					\
	if (res != VK_SUCCESS)																				\
	{																									\
		std::cout << "Fatal : VkResult is \"" << "FAIL" << "\" in " << __FILE__ << " at line " << __LINE__ << "\n"; \
		assert(res == VK_SUCCESS);																		\
	}																									\
}


namespace LYJ_VK {
	class Instance
	{
	public:
		Instance() {};
		~Instance() {};

		uint32_t GetMemoryTypeIndex(const uint32_t memType, const uint32_t usage) {

		}

	private:
		VkInstance m_instance = VK_NULL_HANDLE;
		VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
		std::vector<const char*> m_requireExtensions;
		VkDevice m_device = VK_NULL_HANDLE;
		VkPhysicalDeviceMemoryProperties m_memProperties;
		VkPhysicalDeviceFeatures m_devFeature;
		VkPhysicalDeviceProperties m_devProperties;
	};

	struct Buffer
	{
		VkDeviceMemory devMemory;
		VkBuffer devBuffer;
		VkDescriptorSet descriptor;
		void* devMap2Host;
	};
	static bool CreateBuffer(VkDevice device, const uint32_t size, const VkBufferUsageFlags usage, VkBuffer* buffer) {
		VkBufferCreateInfo bufferCreateInfo{};
		bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferCreateInfo.size = size;
		bufferCreateInfo.usage = usage;
		VK_CHECK_RESULT(vkCreateBuffer(device, &bufferCreateInfo, nullptr, buffer));

		VkMemoryRequirements memReqs;
		vkGetBufferMemoryRequirements(device, *buffer, &memReqs);

		VkMemoryAllocateInfo memAlloc{};
		memAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		memAlloc.allocationSize = memReqs.size;
	}
}



NSP_SLAM_LYJ_BEGIN

//#define NDEBUG

struct QueueFamilyIndices {
	std::optional<uint32_t> graphicsFamily;
	std::optional<uint32_t> presentFamily;

	bool isComplete() {
		return graphicsFamily.has_value() && presentFamily.has_value();
	}

	std::optional<uint32_t> computeFamily;
	bool isCompleteCompute() {
		return computeFamily.has_value();
	}
};
struct SwapChainSupportDetails
{
	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;
};
struct ShaderData
{
	glm::mat4 projectionMatrix;
	glm::mat4 moduleMatrix;
	glm::mat4 viewMatrix;
};
//struct UniformBuffer
//{
//	VkDeviceMemory memory = nullptr;
//	VkBuffer buffer = nullptr;
//	VkDescriptorSet descriptorSet = nullptr;
//	uint8_t* mapped{ nullptr };
//};
//struct Texture
//{
//	VkSampler sampler{ VK_NULL_HANDLE };
//	VkImage image{ VK_NULL_HANDLE };
//	VkImageLayout imageLayout;
//	VkDeviceMemory deviceMemory{ VK_NULL_HANDLE };
//	VkImageView view{ VK_NULL_HANDLE };
//	uint32_t width{ 0 };
//	uint32_t height{ 0 };
//	uint32_t mipLevels{ 0 };
//};
struct Vertex
{
	float pos[3];
	float uv[2];
	float normal[3];
};
namespace vks {
	struct Buffer
	{
		VkBuffer buffer = VK_NULL_HANDLE;
		VkDeviceMemory memory = VK_NULL_HANDLE;
		VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
		void* mapped = nullptr;
		VkDescriptorBufferInfo bufferInfo{};
		VkDeviceSize size = 0;
		VkDeviceSize alignment = 0;
		VkBufferUsageFlags usageFlags;
		VkMemoryPropertyFlags memoryPropertyFlags;

		VkDevice device;
		VkResult map(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
			return vkMapMemory(device, memory, offset, size, 0, &mapped);
		}
		void unmap() {
			if (mapped) {
				vkUnmapMemory(device, memory);
				mapped = nullptr;
			}
		}
		VkResult bind(VkDeviceSize offset = 0) {
			return vkBindBufferMemory(device, buffer, memory, offset);
		}
		void setupDescriptor(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
			bufferInfo.offset = offset;
			bufferInfo.buffer = buffer;
			bufferInfo.range = size;
		}
		void copyTo(void* data, VkDeviceSize size) {
			assert(mapped);
			memcpy(mapped, data, size);
		}
		//对device可见
		VkResult flush(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
			VkMappedMemoryRange mappedRange = {};
			mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
			mappedRange.memory = memory;
			mappedRange.offset = offset;
			mappedRange.size = size;
			return vkFlushMappedMemoryRanges(device, 1, &mappedRange);
		}
		//对host可见
		VkResult invalidate(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
			VkMappedMemoryRange mappedRange = {};
			mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
			mappedRange.memory = memory;
			mappedRange.offset = offset;
			mappedRange.size = size;
			return vkInvalidateMappedMemoryRanges(device, 1, &mappedRange);
		}
		void destroy() {
			if (buffer) {
				vkDestroyBuffer(device, buffer, nullptr);
			}
			if (memory) {
				vkFreeMemory(device, memory, nullptr);
			}
		}
	};
}

class VulkanImp
{
public:
	VulkanImp();
	~VulkanImp();

	void run();

private:
	void compute();
	void init();
	void mainLoop();
	void drawFrame();
	void cleanup();

	bool createVKInstance();
	bool createVKPhysicalDevice(bool _onlyCompute = false);
	bool createVKDevice();
	bool createVKQueue();
	bool createVKCommandPool();
	bool createVKDescriptorPool();
	bool createVKDescriptorSetLayout();
	bool createVKPipelineLayout();
	bool createVKDescriptorSets(bool _bCompute = false);
	bool createVKPipelineCahce();
	bool createVKFenceAndSemaphore();

private:
	bool m_graphics = true;

	VkInstance m_instance;
	VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;

	GLFWwindow* m_windows = nullptr;
	const uint32_t m_width = 1600;
	const uint32_t m_height = 1200;
	VkSurfaceKHR m_surface;
	SwapChainSupportDetails m_details;
	uint32_t m_imageCnt;

	VkDevice m_device = VK_NULL_HANDLE;
	QueueFamilyIndices m_queueIndices;
	VkPhysicalDeviceProperties m_deviceProperties;
	VkPhysicalDeviceFeatures m_deviceFeatures;
	VkPhysicalDeviceMemoryProperties m_deviceMemoryProperties;
	VkQueue m_graphicsQueue;
	VkQueue m_presentQueue;
	VkQueue m_computeQueue;

	VkSwapchainKHR m_swapChain;
	std::vector<VkImage> m_swapChainImages;
	VkFormat m_swapChainImageFormat;
	VkExtent2D m_swapChainExtent;
	std::vector<VkImageView> m_swapChainImageViews;

	VkCommandPool m_graphicsCommandPool;
	VkCommandPool m_computeCommandPool;

	VkDescriptorPool m_descPool;
	VkDescriptorSetLayout m_descriptorSetLayout;

	VkPipelineLayout m_layout = VK_NULL_HANDLE;
	VkPipeline m_pipelineCompute = VK_NULL_HANDLE;
	VkPipeline m_pipeline = VK_NULL_HANDLE;
	VkPipelineCache m_pipelineCache = VK_NULL_HANDLE;

	std::vector<vks::Buffer> m_uniformBuffers;
	vks::Buffer m_deviceBuffer;
	vks::Buffer m_hostBuffer;
	vks::Buffer m_deviceBuffer2;
	vks::Buffer m_hostBuffer2;
	vks::Buffer m_vertexBuffer;
	vks::Buffer m_vertexBufferTmp;
	vks::Buffer m_indexBuffer;
	vks::Buffer m_indexBufferTmp;

	std::vector<VkCommandBuffer> m_commandBuffers;
	VkCommandBuffer m_cmdBufferCompute = VK_NULL_HANDLE;
	VkCommandBuffer m_cmdBuffer = VK_NULL_HANDLE;

	//texture2d
	VkImage m_texture2D;
	VkImageView m_texture2DView;
	VkDeviceMemory m_texture2DMemory;
	VkSampler m_sampler;
	VkImageLayout m_textureLayout;
	uint32_t m_indexCount{ 0 };
	VkDescriptorSet m_descriptorSet{ VK_NULL_HANDLE };


	VkRenderPass m_renderPass;
	std::vector<VkFramebuffer> m_swapChainFramebuffers;


	VkFence m_inFlightFence;
	VkSemaphore m_imageAvailableSemaphore;
	VkSemaphore m_renderFinishedSemaphore;
	VkClearValue m_clearColor{ 1.f, 0.f, 0.f, 1.f };


	//validation
	const std::vector<const char*> m_validationLayers = { "VK_LAYER_KHRONOS_validation" };
	const std::vector<const char*> m_deviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
#ifdef NDEBUG
	const bool m_enableValidationLayers = false;
#else
	const bool m_enableValidationLayers = true;
#endif // NDEBUG


	int m_cnt = 0;
};







NSP_SLAM_LYJ_END