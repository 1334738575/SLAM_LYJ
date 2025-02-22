#include "vulkanImp.h"


NSP_SLAM_LYJ_BEGIN

//static std::string shaderPath = "D:/SLAM_LYJ/src/vulkanImp/shader/";
//static std::string imagePath = "D:/SLAM_LYJ/other/";
static std::string shaderPath = LYJOPT->sysHomePath + "src/vulkanImp/shader/";
static std::string imagePath = LYJOPT->sysHomePath + "other/";


static uint32_t getMemoryTypeIndex(uint32_t _typeBits, VkMemoryPropertyFlags _properties, VkPhysicalDeviceMemoryProperties& _deviceMemoryProperties)
{
	for (uint32_t i = 0; i < _deviceMemoryProperties.memoryTypeCount; ++i) {
		if ((_typeBits & 1) == 1) {
			if ((_deviceMemoryProperties.memoryTypes[i].propertyFlags & _properties) == _properties)
			{
				return i;
			}
		}
		_typeBits >>= 1;
	}
	return 0;
}
static VkShaderModule createShaderModule(VkDevice& _device, const std::vector<char>& _code)
{
	VkShaderModuleCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.codeSize = _code.size();
	createInfo.pCode = reinterpret_cast<const uint32_t*>(_code.data());
	VkShaderModule shaderModule;
	if (vkCreateShaderModule(_device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
		throw std::runtime_error("failed to create shader module");
	return shaderModule;
}
static std::vector<char> readFile(const std::string& _filename) {
	std::ifstream file(_filename, std::ios::ate | std::ios::binary); //from file end
	if (!file.is_open())
		throw std::runtime_error("failed to open file");
	size_t fileSize = (size_t)file.tellg();
	std::vector<char> buffer(fileSize);
	file.seekg(0);//to file begin
	file.read(buffer.data(), fileSize);
	file.close();
	return buffer;
}
static void createBuffer(VkDevice& _device, vks::Buffer& _buffer, VkPhysicalDeviceMemoryProperties& _deviceMemoryProperties, void* _data = nullptr)
{
	VkBufferCreateInfo bufferCreateInfo{};
	bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferCreateInfo.usage = _buffer.usageFlags;
	bufferCreateInfo.size = _buffer.size;
	//VK_SHARING_MODE_EXCLUSIVE = 0, 资源被队列访问时独占
	//VK_SHARING_MODE_CONCURRENT = 1, 允许多个队列访问
	bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	VK_CHECK_RESULT(vkCreateBuffer(_device, &bufferCreateInfo, nullptr, &_buffer.buffer));
	VkMemoryRequirements memoryRequires;
	VkMemoryAllocateInfo memoryAllocateInfo{};
	memoryAllocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	vkGetBufferMemoryRequirements(_device, _buffer.buffer, &memoryRequires);
	memoryAllocateInfo.allocationSize = memoryRequires.size;
	memoryAllocateInfo.memoryTypeIndex = getMemoryTypeIndex(memoryRequires.memoryTypeBits, _buffer.memoryPropertyFlags, _deviceMemoryProperties);
	VK_CHECK_RESULT(vkAllocateMemory(_device, &memoryAllocateInfo, nullptr, &_buffer.memory));
	VK_CHECK_RESULT(vkMapMemory(_device, _buffer.memory, 0, _buffer.size, 0, &_buffer.mapped));
	if (_data != nullptr) {
		memcpy(_buffer.mapped, _data, _buffer.size);
		vkUnmapMemory(_device, _buffer.memory);
	}
	VK_CHECK_RESULT(vkBindBufferMemory(_device, _buffer.buffer, _buffer.memory, 0));
	return;
}
static VkCommandBuffer createCommandBuffer(VkDevice& _device, VkCommandPool& _commandPool, bool _begin)
{
	VkCommandBufferAllocateInfo cmdAllocInfo{};
	cmdAllocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	cmdAllocInfo.commandPool = _commandPool;
	cmdAllocInfo.commandBufferCount = 1;
	cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	VkCommandBuffer vkCmdBuffer;
	VK_CHECK_RESULT(vkAllocateCommandBuffers(_device, &cmdAllocInfo, &vkCmdBuffer));
	if (_begin) {
		VkCommandBufferBeginInfo cmdBeginInfo{};
		cmdBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		VK_CHECK_RESULT(vkBeginCommandBuffer(vkCmdBuffer, &cmdBeginInfo));
	}
	return vkCmdBuffer;
}
//只支持简单的提交，不涉及先后顺序
static void flushCommand(VkDevice& _device, VkCommandBuffer& _cmdBuffer, VkCommandPool& _commandPool, VkQueue& _queue, VkFence& _fence, bool _destory)
{
	vkResetFences(_device, 1, &_fence);
	VK_CHECK_RESULT(vkEndCommandBuffer(_cmdBuffer));
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &_cmdBuffer;
	VK_CHECK_RESULT(vkQueueSubmit(_queue, 1, &submitInfo, _fence));
	VK_CHECK_RESULT(vkWaitForFences(_device, 1, &_fence, VK_TRUE, UINT64_MAX));
	vkFreeCommandBuffers(_device, _commandPool, 1, &_cmdBuffer);
}
static void copyBuffer(VkDevice& _device, VkCommandPool& _commandPool, VkFence& _fence, vks::Buffer* _src, vks::Buffer* _dst, VkQueue _queue, VkBufferCopy* _copyRegion)
{
	assert(dst->size <= src->size);
	assert(src->buffer);
	VkCommandBuffer copyCmd = createCommandBuffer(_device, _commandPool, true);
	VkBufferCopy bufferCopy{};
	if (_copyRegion == nullptr) {
		bufferCopy.size = _src->size;
	}
	else {
		bufferCopy = *_copyRegion;
	}
	vkCmdCopyBuffer(copyCmd, _src->buffer, _dst->buffer, 1, &bufferCopy);
	flushCommand(_device, copyCmd, _commandPool, _queue, _fence, true);
	return;
}



VulkanImp::VulkanImp()
{
}
VulkanImp::~VulkanImp()
{
}

void VulkanImp::run() {
	init();
	compute();
	mainLoop();
	cleanup();
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
	VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
	VkDebugUtilsMessageTypeFlagsEXT messageType,
	const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
	void* pUserData) {
	std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
	return VK_FALSE;
}

void VulkanImp::compute() {
	//createVKInstance();
	//createVKPhysicalDevice();
	//createVKDevice();
	//createVKQueue();
	//createVKFenceAndSemaphore();
	//createVKCommandPool();

	const int computeSize = 32;
	std::vector<uint32_t> computeInput(computeSize);
	std::vector<uint32_t> computeOutput(computeSize);
	uint32_t n = 0;
	std::generate(computeInput.begin(), computeInput.end(), [&n] {return n++; });
	const VkDeviceSize bufferSize = computeSize * sizeof(uint32_t);
	{
		m_hostBuffer.size = bufferSize;
		m_hostBuffer.usageFlags = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
		m_hostBuffer.memoryPropertyFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
		createBuffer(m_device, m_hostBuffer, m_deviceMemoryProperties, computeInput.data());
		VkMappedMemoryRange mappedRange;
		mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
		mappedRange.memory = m_hostBuffer.memory;
		mappedRange.offset = 0;
		mappedRange.size = VK_WHOLE_SIZE;
		vkFlushMappedMemoryRanges(m_device, 1, &mappedRange);
		m_deviceBuffer.size = bufferSize;
		m_deviceBuffer.usageFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		m_deviceBuffer.memoryPropertyFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		createBuffer(m_device, m_deviceBuffer, m_deviceMemoryProperties);
		copyBuffer(m_device, m_computeCommandPool, m_inFlightFence, &m_hostBuffer, &m_deviceBuffer, m_computeQueue, nullptr);
	}
	const int computeSize2 = 32;
	std::vector<uint32_t> computeInput2(computeSize2);
	std::vector<uint32_t> computeOutput2(computeSize2);
	uint32_t n2 = 5;
	std::generate(computeInput2.begin(), computeInput2.end(), [&n2] {return n2++; });
	const VkDeviceSize bufferSize2 = computeSize2 * sizeof(uint32_t);
	{
		m_hostBuffer2.size = bufferSize2;
		m_hostBuffer2.usageFlags = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
		m_hostBuffer2.memoryPropertyFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
		createBuffer(m_device, m_hostBuffer2, m_deviceMemoryProperties, computeInput2.data());
		VkMappedMemoryRange mappedRange;
		mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
		mappedRange.memory = m_hostBuffer2.memory;
		mappedRange.offset = 0;
		mappedRange.size = VK_WHOLE_SIZE;
		vkFlushMappedMemoryRanges(m_device, 1, &mappedRange);
		m_deviceBuffer2.size = bufferSize2;
		m_deviceBuffer2.usageFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		m_deviceBuffer2.memoryPropertyFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		createBuffer(m_device, m_deviceBuffer2, m_deviceMemoryProperties);
		copyBuffer(m_device, m_computeCommandPool, m_inFlightFence, &m_hostBuffer2, &m_deviceBuffer2, m_computeQueue, nullptr);
	}

	//createVKDescriptorPool();
	//createVKDescriptorSetLayout();
	//createVKPipelineLayout();
	createVKDescriptorSets(true);
	createVKPipelineCahce();
	{
		VkComputePipelineCreateInfo computePipelineCreateInfo{};
		computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
		computePipelineCreateInfo.layout = m_layout;
		computePipelineCreateInfo.flags = 0;
		struct SpecializationData
		{
			uint32_t BUFFER_ELEMENT_COUNT = static_cast<uint32_t>(computeSize);
		} specializationData;
		VkSpecializationMapEntry specializationMapEntry{};
		specializationMapEntry.constantID = 0;
		specializationMapEntry.offset = 0;
		specializationMapEntry.size = sizeof(uint32_t);
		VkSpecializationInfo specializationInfo{};
		specializationInfo.dataSize = sizeof(SpecializationData);
		specializationInfo.mapEntryCount = 1;
		specializationInfo.pData = &specializationData;
		specializationInfo.pMapEntries = &specializationMapEntry;
		auto computeShaderCode = readFile(shaderPath + "compute/headless.comp.spv");
		VkShaderModule computeShaderModule = createShaderModule(m_device, computeShaderCode);
		VkPipelineShaderStageCreateInfo pipelineShaderStageCreateInfo{};
		pipelineShaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		pipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
		pipelineShaderStageCreateInfo.module = computeShaderModule;
		pipelineShaderStageCreateInfo.pSpecializationInfo = &specializationInfo;
		pipelineShaderStageCreateInfo.pName = "main";
		assert(pipelineShaderStageCreateInfo.module != VK_NULL_HANDLE);
		computePipelineCreateInfo.stage = pipelineShaderStageCreateInfo;
		VK_CHECK_RESULT(vkCreateComputePipelines(m_device, m_pipelineCache, 1, &computePipelineCreateInfo, nullptr, &m_pipelineCompute));
	}

	{
		m_cmdBufferCompute = createCommandBuffer(m_device, m_computeCommandPool, true);

		std::vector< VkBufferMemoryBarrier> bufferMemoryBarriers(2);
		bufferMemoryBarriers[0] = VkBufferMemoryBarrier{};
		bufferMemoryBarriers[1] = VkBufferMemoryBarrier{};
		VkBufferMemoryBarrier& bufferMemoryBarrier = bufferMemoryBarriers[0];
		VkBufferMemoryBarrier& bufferMemoryBarrier2 = bufferMemoryBarriers[1];

		bufferMemoryBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
		bufferMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		bufferMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		bufferMemoryBarrier.buffer = m_deviceBuffer.buffer;
		bufferMemoryBarrier.size = VK_WHOLE_SIZE;
		bufferMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
		bufferMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		bufferMemoryBarrier2.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
		bufferMemoryBarrier2.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		bufferMemoryBarrier2.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		bufferMemoryBarrier2.buffer = m_deviceBuffer2.buffer;
		bufferMemoryBarrier2.size = VK_WHOLE_SIZE;
		bufferMemoryBarrier2.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
		bufferMemoryBarrier2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		vkCmdPipelineBarrier(m_cmdBufferCompute,
			VK_PIPELINE_STAGE_HOST_BIT,
			VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
			0, 0, nullptr, 2, bufferMemoryBarriers.data(), 0, nullptr);
		vkCmdBindPipeline(m_cmdBufferCompute, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineCompute);
		std::vector< VkDescriptorSet> descriptorsets;
		descriptorsets.push_back(m_deviceBuffer.descriptorSet);
		descriptorsets.push_back(m_deviceBuffer2.descriptorSet);
		//指令，管道阶段，管道布局，管道布局的起始描述符集，描述符集数量，描述符集数据（要和创建时的描述符集结构相同）
		vkCmdBindDescriptorSets(m_cmdBufferCompute, VK_PIPELINE_BIND_POINT_COMPUTE, m_layout, 0, 2, descriptorsets.data(), 0, nullptr);
		//vkCmdBindDescriptorSets(m_cmdBufferCompute, VK_PIPELINE_BIND_POINT_COMPUTE, m_layout, 0, 1, &m_deviceBuffer.descriptorSet, 0, nullptr);
		//vkCmdBindDescriptorSets(m_cmdBufferCompute, VK_PIPELINE_BIND_POINT_COMPUTE, m_layout, 1, 1, &m_deviceBuffer2.descriptorSet, 0, nullptr);
		vkCmdDispatch(m_cmdBufferCompute, computeSize, 1, 1); //运行线程数，类似cuda中xyz
		bufferMemoryBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		bufferMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
		bufferMemoryBarrier.buffer = m_deviceBuffer.buffer;
		bufferMemoryBarrier.size = VK_WHOLE_SIZE;
		bufferMemoryBarrier2.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		bufferMemoryBarrier2.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
		bufferMemoryBarrier2.buffer = m_deviceBuffer2.buffer;
		bufferMemoryBarrier2.size = VK_WHOLE_SIZE;
		vkCmdPipelineBarrier(m_cmdBufferCompute,
			VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			0, 0, nullptr, 2, bufferMemoryBarriers.data(), 0, nullptr);
		VkBufferCopy bufferCopy{};
		bufferCopy.size = bufferSize;
		vkCmdCopyBuffer(m_cmdBufferCompute, m_deviceBuffer.buffer, m_hostBuffer.buffer, 1, &bufferCopy);
		bufferMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		bufferMemoryBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
		bufferMemoryBarrier.buffer = m_hostBuffer.buffer;
		bufferMemoryBarrier.size = VK_WHOLE_SIZE;
		vkCmdPipelineBarrier(m_cmdBufferCompute,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			VK_PIPELINE_STAGE_HOST_BIT,
			0, 0, nullptr, 1, &bufferMemoryBarrier, 0, nullptr);


		VK_CHECK_RESULT(vkEndCommandBuffer(m_cmdBufferCompute));
		vkResetFences(m_device, 1, &m_inFlightFence);
		const VkPipelineStageFlags waitStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
		VkSubmitInfo computeSubmitInfo{};
		computeSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		computeSubmitInfo.pWaitDstStageMask = &waitStageMask;
		computeSubmitInfo.commandBufferCount = 1;
		computeSubmitInfo.pCommandBuffers = &m_cmdBufferCompute;
		VK_CHECK_RESULT(vkQueueSubmit(m_computeQueue, 1, &computeSubmitInfo, m_inFlightFence));
		VK_CHECK_RESULT(vkWaitForFences(m_device, 1, &m_inFlightFence, VK_TRUE, UINT64_MAX));

		// Make device writes visible to the host
		void* mapped;
		vkMapMemory(m_device, m_hostBuffer.memory, 0, VK_WHOLE_SIZE, 0, &mapped);
		VkMappedMemoryRange mappedRange{};
		mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
		mappedRange.memory = m_hostBuffer.memory;
		mappedRange.offset = 0;
		mappedRange.size = VK_WHOLE_SIZE;
		vkInvalidateMappedMemoryRanges(m_device, 1, &mappedRange);

		// Copy to output
		memcpy(computeOutput.data(), mapped, bufferSize);
		vkUnmapMemory(m_device, m_hostBuffer.memory);
	}
	vkQueueWaitIdle(m_computeQueue);

	// Output buffer contents
	std::cout << "Compute input:\n";
	for (auto v : computeInput) {
		std::cout << v << " ";
	}
	std::cout << std::endl;
	std::cout << "Compute output:\n";
	for (auto v : computeOutput) {
		std::cout << v << " ";
	}
	std::cout << std::endl;
}

void VulkanImp::init() {
	auto funcCreateWinows = [&]() {
		glfwInit();
		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
		glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
		m_windows = glfwCreateWindow(m_width, m_height, "Vulkan", nullptr, nullptr);
		};
	funcCreateWinows();

	//ini vk
	createVKInstance();

	//extensions
	auto funcRequiredExtensions = [&]()->std::vector<const char*> {
		uint32_t glfwExtCnt = 0;
		const char** glfwExtensions;
		glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtCnt);
		std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtCnt);
		if (m_enableValidationLayers) {
			extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
		}
		return extensions;
		};
	auto funcInitVkExt = [&]()->bool {
		uint32_t extensionCnt = 0;
		vkEnumerateInstanceExtensionProperties(nullptr, &extensionCnt, nullptr);
		std::vector<VkExtensionProperties> extensions(extensionCnt);
		vkEnumerateInstanceExtensionProperties(nullptr, &extensionCnt, extensions.data());
		std::cout << "available extensions:\n";
		for (const auto& extension : extensions) {
			std::cout << '\t' << extension.extensionName << '\n';
		}
		return true;
		};
	//funcInitVkExt();
	//layers
	auto funcCheckValid = [&]()->bool {
		uint32_t layerCnt = 0;
		vkEnumerateInstanceLayerProperties(&layerCnt, nullptr);
		std::vector<VkLayerProperties> availableLayers(layerCnt);
		vkEnumerateInstanceLayerProperties(&layerCnt, availableLayers.data());
		bool layerFound = false;
		for (const char* layerName : m_validationLayers) {
			for (const auto& layerProperty : availableLayers) {
				if (strcmp(layerName, layerProperty.layerName) == 0) {
					layerFound = true;
					break;
				}
			}
		}
		return layerFound;
		};
	if (m_enableValidationLayers && !funcCheckValid()) {
		std::cout << "can't find valid layer!\n";
	}

	//surface
	auto createSurface = [&]()->bool {
		if (glfwCreateWindowSurface(m_instance, m_windows, nullptr, &m_surface) != VK_SUCCESS)
			return false;
		return true;
		};
	createSurface();

	//physical device
	createVKPhysicalDevice();
	//logical device
	createVKDevice();
	createVKQueue();
	createVKFenceAndSemaphore();

	//swap chain
	auto funcChooseSwapSurfaceFormat = [&](const std::vector<VkSurfaceFormatKHR>& availableFormats)-> VkSurfaceFormatKHR {
		for (const auto& availableFormat : availableFormats) {
			if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
			{
				return availableFormat;
			}
		}
		return availableFormats[0];
		};
	auto funcChooseSwapPresentMode = [&](const std::vector<VkPresentModeKHR>& availablePresentModes)->VkPresentModeKHR {
		for (const auto& availablePresentMode : availablePresentModes) {
			if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR)
				return availablePresentMode;
		}
		return VK_PRESENT_MODE_FIFO_KHR;
		};
	auto funcChooseSwapExtent = [&](const VkSurfaceCapabilitiesKHR& capabilities)->VkExtent2D {
		if (capabilities.currentExtent.width != UINT32_MAX) {
			return capabilities.currentExtent;
		}
		else {
			int width, height;
			glfwGetFramebufferSize(m_windows, &width, &height);
			VkExtent2D actualExtent = {
				static_cast<uint32_t>(width),
				static_cast<uint32_t>(height)
			};
			actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
			actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
			return actualExtent;
		}
		};
	auto funcCreateSwapChain = [&]() {
		VkSurfaceFormatKHR surfaceFormat = funcChooseSwapSurfaceFormat(m_details.formats);
		VkPresentModeKHR presentMode = funcChooseSwapPresentMode(m_details.presentModes);
		VkExtent2D extent = funcChooseSwapExtent(m_details.capabilities);
		m_imageCnt = m_details.capabilities.minImageCount + 1;
		if (m_details.capabilities.maxImageCount > 0 && m_imageCnt > m_details.capabilities.maxImageCount) {
			m_imageCnt = m_details.capabilities.maxImageCount;
		}
		m_imageCnt = 2;
		VkSwapchainCreateInfoKHR createInfo{};
		createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
		createInfo.surface = m_surface;
		createInfo.minImageCount = m_imageCnt;
		createInfo.imageFormat = surfaceFormat.format;
		createInfo.imageColorSpace = surfaceFormat.colorSpace;
		createInfo.imageExtent = extent;
		createInfo.imageArrayLayers = 1;
		createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
		uint32_t queueFamilyIndices[] = { m_queueIndices.graphicsFamily.value(), m_queueIndices.presentFamily.value() };
		if (m_queueIndices.graphicsFamily != m_queueIndices.presentFamily) {
			createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
			createInfo.queueFamilyIndexCount = 2;
			createInfo.pQueueFamilyIndices = queueFamilyIndices;
		}
		else {
			createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
			createInfo.queueFamilyIndexCount = 0;
			createInfo.pQueueFamilyIndices = nullptr;
		}
		createInfo.preTransform = m_details.capabilities.currentTransform; //最终显示图像变换（旋转，镜像）
		createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR; //图像合成效果，透明度
		createInfo.presentMode = presentMode;
		createInfo.clipped = VK_TRUE; //裁剪超出窗口外的部分
		createInfo.oldSwapchain = VK_NULL_HANDLE; //不为空时，将在旧交换链基础上创建新的
		if (vkCreateSwapchainKHR(m_device, &createInfo, nullptr, &m_swapChain) != VK_SUCCESS)
			throw std::runtime_error("failed to create swap chain");
		vkGetSwapchainImagesKHR(m_device, m_swapChain, &m_imageCnt, nullptr);
		m_swapChainImages.resize(m_imageCnt);
		vkGetSwapchainImagesKHR(m_device, m_swapChain, &m_imageCnt, m_swapChainImages.data());
		m_swapChainImageFormat = surfaceFormat.format;
		m_swapChainExtent = extent;
		};
	funcCreateSwapChain();

	//command pool
	createVKCommandPool();

	//return;

	//image view
	auto funcCreateImageViews = [&]() {
		m_swapChainImageViews.resize(m_swapChainImages.size());
		for (size_t i = 0; i < m_swapChainImages.size(); ++i) {
			VkImageViewCreateInfo createInfo{};
			createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
			createInfo.image = m_swapChainImages[i];
			createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
			createInfo.format = m_swapChainImageFormat;
			createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
			createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
			createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
			createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
			createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			createInfo.subresourceRange.baseMipLevel = 0;
			createInfo.subresourceRange.levelCount = 1;
			createInfo.subresourceRange.baseArrayLayer = 0;
			createInfo.subresourceRange.layerCount = 1;
			if (vkCreateImageView(m_device, &createInfo, nullptr, &m_swapChainImageViews[i]) != VK_SUCCESS)
				throw std::runtime_error("failed to create image views");
		}
		};
	funcCreateImageViews();

	//render pass
	auto funcCreateRenderPass = [&]() {
		VkAttachmentDescription colorAttachment{};
		colorAttachment.format = m_swapChainImageFormat;
		colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
		colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

		VkAttachmentReference colorAttachmentRef{};
		colorAttachmentRef.attachment = 0;
		colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

		VkSubpassDescription subpass{};
		subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpass.colorAttachmentCount = 1;
		subpass.pColorAttachments = &colorAttachmentRef;

		VkSubpassDependency dependency{};
		dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
		dependency.dstSubpass = 0;
		dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependency.srcAccessMask = 0;
		dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

		VkRenderPassCreateInfo renderPassInfo{};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.attachmentCount = 1;
		renderPassInfo.pAttachments = &colorAttachment;
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = 1;
		renderPassInfo.pDependencies = &dependency;

		if (vkCreateRenderPass(m_device, &renderPassInfo, nullptr, &m_renderPass) != VK_SUCCESS)
			throw std::runtime_error("failed to create render pass");
		};
	funcCreateRenderPass();

	//texture2d
	auto funcCreateTextureImage = [&]() {
		//std::string imgName = imagePath + "IMG_9179[1](1).png";
		//cv::Mat image1 = cv::imread(imgName, cv::IMREAD_UNCHANGED);
		//cv::Mat image = image1.clone();
		//if (image.channels() == 3)
		//	cv::cvtColor(image, image, cv::COLOR_BGR2BGRA);

		std::string imgName = imagePath + "IMG_9179[1](1).png";
		cv::Mat image1 = cv::imread(imgName, 0);
		cv::Mat image2 = image1.clone();
		if (image1.empty() || image2.empty()) {
			throw std::runtime_error("failed to load texture image!");
		}
		cv::Mat image = cv::Mat::zeros(image1.rows, image1.cols + image2.cols, image1.type());
		cv::Rect rect1(0, 0, image1.cols, image1.rows);
		cv::Rect rect2(image1.cols, 0, image2.cols, image2.rows);
		image1.copyTo(image(rect1));
		image2.copyTo(image(rect2));
		if (image.channels() == 1)
			cv::cvtColor(image, image, cv::COLOR_GRAY2RGBA);

		//std::string imgName = imagePath + "img1.png";
		//cv::Mat image = cv::imread(imgName, cv::IMREAD_UNCHANGED);
		//if (image.empty()) {
		//	throw std::runtime_error("failed to load texture image!");
		//}
		//if (image.channels() == 3)
		//	cv::cvtColor(image, image, cv::COLOR_BGR2BGRA);

#if false
		const int width = image.cols;
		const int height = image.rows;
		VkDeviceSize imageSize = width * height * 4;
		VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;

		VkBuffer stagingBuffer;
		VkDeviceMemory stagingMemory;

		VkBufferCreateInfo bufferCreateInfo{};
		bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferCreateInfo.size = imageSize;
		// This buffer is used as a transfer source for the buffer copy
		bufferCreateInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		VK_CHECK_RESULT(vkCreateBuffer(m_device, &bufferCreateInfo, nullptr, &stagingBuffer));

		VkMemoryAllocateInfo memAllocInfo{};
		memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		VkMemoryRequirements memReqs = {};
		// Get memory requirements for the staging buffer (alignment, memory type bits)
		vkGetBufferMemoryRequirements(m_device, stagingBuffer, &memReqs);
		memAllocInfo.allocationSize = memReqs.size;
		// Get memory type index for a host visible buffer
		memAllocInfo.memoryTypeIndex = getMemoryTypeIndex(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, m_deviceMemoryProperties);
		VK_CHECK_RESULT(vkAllocateMemory(m_device, &memAllocInfo, nullptr, &stagingMemory));
		VK_CHECK_RESULT(vkBindBufferMemory(m_device, stagingBuffer, stagingMemory, 0));

		// Copy texture data into host local staging buffer
		uint8_t* data;
		VK_CHECK_RESULT(vkMapMemory(m_device, stagingMemory, 0, memReqs.size, 0, (void**)&data));
		memcpy(data, image.data, imageSize);
		vkUnmapMemory(m_device, stagingMemory);

		// Setup buffer copy regions for each mip level
		std::vector<VkBufferImageCopy> bufferCopyRegions;
		uint32_t offset = 0;

		uint32_t minl = 1;
		for (uint32_t i = 0; i < minl; i++) {
			// Setup a buffer image copy structure for the current mip level
			VkBufferImageCopy bufferCopyRegion = {};
			bufferCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			bufferCopyRegion.imageSubresource.mipLevel = i;
			bufferCopyRegion.imageSubresource.baseArrayLayer = 0;
			bufferCopyRegion.imageSubresource.layerCount = 1;
			bufferCopyRegion.imageExtent.width = static_cast<uint32_t>(width) >> i;
			bufferCopyRegion.imageExtent.height = static_cast<uint32_t>(height) >> i;
			bufferCopyRegion.imageExtent.depth = 1;
			bufferCopyRegion.bufferOffset = offset;
			bufferCopyRegions.push_back(bufferCopyRegion);
		}

		// Create optimal tiled target image on the device
		VkImageCreateInfo imageCreateInfo{};
		imageCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
		imageCreateInfo.format = format;
		imageCreateInfo.mipLevels = minl;
		imageCreateInfo.arrayLayers = 1;
		imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
		imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		// Set initial layout of the image to undefined
		imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		imageCreateInfo.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
		imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		VK_CHECK_RESULT(vkCreateImage(m_device, &imageCreateInfo, nullptr, &m_texture2D));

		vkGetImageMemoryRequirements(m_device, m_texture2D, &memReqs);
		memAllocInfo.allocationSize = memReqs.size;
		memAllocInfo.memoryTypeIndex = getMemoryTypeIndex(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m_deviceMemoryProperties);
		VK_CHECK_RESULT(vkAllocateMemory(m_device, &memAllocInfo, nullptr, &m_texture2DMemory));
		VK_CHECK_RESULT(vkBindImageMemory(m_device, m_texture2D, m_texture2DMemory, 0));

		VkCommandBufferAllocateInfo cmdBufAllocateInfo{};
		cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		cmdBufAllocateInfo.commandPool = m_graphicsCommandPool;
		cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		cmdBufAllocateInfo.commandBufferCount = 1;
		VkCommandBuffer copyCmd;
		VK_CHECK_RESULT(vkAllocateCommandBuffers(m_device, &cmdBufAllocateInfo, &copyCmd));
		VkCommandBufferBeginInfo cmdBufBeginInfo{};
		cmdBufBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		VK_CHECK_RESULT(vkBeginCommandBuffer(copyCmd, &cmdBufBeginInfo));

		// Image memory barriers for the texture image

		// The sub resource range describes the regions of the image that will be transitioned using the memory barriers below
		VkImageSubresourceRange subResourceRange = {};
		// Image only contains color data
		subResourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		// Start at first mip level
		subResourceRange.baseMipLevel = 0;
		// We will transition on all mip levels
		subResourceRange.levelCount = minl;
		// The 2D texture only has one layer
		subResourceRange.layerCount = 1;

		// Transition the texture image layout to transfer target, so we can safely copy our buffer data to it.
		VkImageMemoryBarrier imageMemoryBarrier{};
		imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageMemoryBarrier.image = m_texture2D;
		imageMemoryBarrier.subresourceRange = subResourceRange;
		imageMemoryBarrier.srcAccessMask = 0;
		imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

		// Insert a memory dependency at the proper pipeline stages that will execute the image layout transition
		// Source pipeline stage is host write/read execution (VK_PIPELINE_STAGE_HOST_BIT)
		// Destination pipeline stage is copy command execution (VK_PIPELINE_STAGE_TRANSFER_BIT)
		vkCmdPipelineBarrier(
			copyCmd,
			VK_PIPELINE_STAGE_HOST_BIT,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &imageMemoryBarrier);

		// Copy mip levels from staging buffer
		vkCmdCopyBufferToImage(
			copyCmd,
			stagingBuffer,
			m_texture2D,
			VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
			static_cast<uint32_t>(bufferCopyRegions.size()),
			bufferCopyRegions.data());

		// Once the data has been uploaded we transfer to the texture image to the shader read layout, so it can be sampled from
		imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

		// Insert a memory dependency at the proper pipeline stages that will execute the image layout transition
		// Source pipeline stage is copy command execution (VK_PIPELINE_STAGE_TRANSFER_BIT)
		// Destination pipeline stage fragment shader access (VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT)
		vkCmdPipelineBarrier(
			copyCmd,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &imageMemoryBarrier);

		// Store current layout for later reuse
		m_textureLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

		VK_CHECK_RESULT(vkEndCommandBuffer(copyCmd));
		VkSubmitInfo submitInfo{};
		submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &copyCmd;
		VkFenceCreateInfo fenceInfo{};
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		fenceInfo.flags = 0;
		VkFence fence;
		VK_CHECK_RESULT(vkCreateFence(m_device, &fenceInfo, nullptr, &fence));
		VK_CHECK_RESULT(vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, fence));
		VK_CHECK_RESULT(vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX));

		// Clean up staging resources
		vkFreeMemory(m_device, stagingMemory, nullptr);
		vkDestroyBuffer(m_device, stagingBuffer, nullptr);

#else
		const int width = image.cols;
		const int height = image.rows;
		VkDeviceSize imageSize = width * height * 4;

		VkImageCreateInfo imageInfo{};
		imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageInfo.imageType = VK_IMAGE_TYPE_2D;
		imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
		imageInfo.mipLevels = 1; //纹理分辨率层级，一次减半
		imageInfo.arrayLayers = 1;
		imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageInfo.tiling = VK_IMAGE_TILING_LINEAR;
		imageInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT;
		imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		imageInfo.initialLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
		imageInfo.extent.width = width;
		imageInfo.extent.height = height;
		imageInfo.extent.depth = 1;
		VK_CHECK_RESULT(vkCreateImage(m_device, &imageInfo, nullptr, &m_texture2D));

		VkMemoryRequirements memRequirements{};
		vkGetImageMemoryRequirements(m_device, m_texture2D, &memRequirements);

		VkMemoryAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocInfo.allocationSize = memRequirements.size;
		allocInfo.memoryTypeIndex = getMemoryTypeIndex(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, m_deviceMemoryProperties);
		VK_CHECK_RESULT(vkAllocateMemory(m_device, &allocInfo, nullptr, &m_texture2DMemory));
		VK_CHECK_RESULT(vkBindImageMemory(m_device, m_texture2D, m_texture2DMemory, 0));

		void* data;
		vkMapMemory(m_device, m_texture2DMemory, 0, memRequirements.size, 0, &data);
		memcpy(data, image.data, memRequirements.size);
		vkUnmapMemory(m_device, m_texture2DMemory);
		m_textureLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

		VkImageSubresourceRange subResourceRange{};
		subResourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		subResourceRange.baseMipLevel = 0;
		subResourceRange.baseArrayLayer = 0;
		subResourceRange.levelCount = 1;
		subResourceRange.layerCount = 1;

		VkCommandBuffer copyCmd = createCommandBuffer(m_device, m_graphicsCommandPool, true);

		VkImageMemoryBarrier imageMemoryBarrier{};
		imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageMemoryBarrier.image = m_texture2D;
		imageMemoryBarrier.subresourceRange = subResourceRange;
		imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
		imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
		imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		vkCmdPipelineBarrier(
			copyCmd,
			VK_PIPELINE_STAGE_HOST_BIT,
			VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &imageMemoryBarrier
		);

		flushCommand(m_device, copyCmd, m_graphicsCommandPool, m_graphicsQueue, m_inFlightFence, true);
#endif


		VkSamplerCreateInfo samplerCreateInfo{};
		samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samplerCreateInfo.maxAnisotropy = 1.f;
		samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
		samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
		samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
		samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.mipLodBias = 0.f;
		samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
		samplerCreateInfo.minLod = 0.f;
		samplerCreateInfo.maxLod = 0.f;
		samplerCreateInfo.maxAnisotropy = 1.0;
		samplerCreateInfo.anisotropyEnable = VK_FALSE;
		samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
		VK_CHECK_RESULT(vkCreateSampler(m_device, &samplerCreateInfo, nullptr, &m_sampler));

		VkImageViewCreateInfo viewCreateInfo{};
		viewCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
		viewCreateInfo.subresourceRange = subResourceRange;
		viewCreateInfo.image = m_texture2D;
		VK_CHECK_RESULT(vkCreateImageView(m_device, &viewCreateInfo, nullptr, &m_texture2DView));
		};
	funcCreateTextureImage();

	//vertices and indices buffer
	const float edgeRatio = 0.5;
	const float textureEdgeRatio = 1.0;
	auto funcGenerateQuad = [&]() {
		std::vector<Vertex> vertices =
		{
			{{edgeRatio, edgeRatio, 0.f}, {textureEdgeRatio, textureEdgeRatio}, {0.f, 0.f, 1.f}},
			{{-edgeRatio, edgeRatio, 0.f}, {0.f, textureEdgeRatio}, {0.f, 0.f, 1.f}},
			{{-edgeRatio, -edgeRatio, 0.f}, {0.f, 0.f}, {0.f, 0.f, 1.f}},
			{{edgeRatio, -edgeRatio, 0.f}, {textureEdgeRatio, 0.f}, {0.f, 0.f, 1.f}}
		};
		std::vector<uint32_t> indices = { 0,1,2, 2,3,0 };
		m_indexCount = static_cast<uint32_t>(indices.size());
		m_vertexBufferTmp.size = vertices.size() * sizeof(Vertex);
		m_vertexBufferTmp.usageFlags = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		m_vertexBufferTmp.memoryPropertyFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		createBuffer(m_device, m_vertexBufferTmp, m_deviceMemoryProperties, vertices.data());
		m_indexBufferTmp.size = indices.size() * sizeof(uint32_t);
		m_indexBufferTmp.usageFlags = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		m_indexBufferTmp.memoryPropertyFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		createBuffer(m_device, m_indexBufferTmp, m_deviceMemoryProperties, indices.data());
		m_vertexBuffer.size = vertices.size() * sizeof(Vertex);
		m_vertexBuffer.usageFlags = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
		m_vertexBuffer.memoryPropertyFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		createBuffer(m_device, m_vertexBuffer, m_deviceMemoryProperties);
		m_indexBuffer.size = indices.size() * sizeof(uint32_t);
		m_indexBuffer.usageFlags = VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
		m_indexBuffer.memoryPropertyFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		createBuffer(m_device, m_indexBuffer, m_deviceMemoryProperties);
		copyBuffer(m_device, m_graphicsCommandPool, m_inFlightFence, &m_vertexBufferTmp, &m_vertexBuffer, m_graphicsQueue, nullptr);
		copyBuffer(m_device, m_graphicsCommandPool, m_inFlightFence, &m_indexBufferTmp, &m_indexBuffer, m_graphicsQueue, nullptr);
		};
	funcGenerateQuad();

	//frame buffer
	auto funcCreateFramebuffers = [&]() {
		m_swapChainFramebuffers.resize(m_swapChainImageViews.size());
		for (size_t i = 0; i < m_swapChainImageViews.size(); ++i) {
			VkImageView attachments[] = { m_swapChainImageViews[i] };
			VkFramebufferCreateInfo framebufferInfo{};
			framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
			framebufferInfo.renderPass = m_renderPass;
			framebufferInfo.attachmentCount = 1;
			framebufferInfo.pAttachments = attachments;
			framebufferInfo.width = m_swapChainExtent.width;
			framebufferInfo.height = m_swapChainExtent.height;
			framebufferInfo.layers = 1;
			if (vkCreateFramebuffer(m_device, &framebufferInfo, nullptr, &m_swapChainFramebuffers[i]) != VK_SUCCESS) {
				throw std::runtime_error("failed to create framebuffer");
			}
		}
		};
	funcCreateFramebuffers();

	//uniform buffer
	auto funcCreateUniformBuffers = [&]() {
		m_uniformBuffers.resize(m_imageCnt);
		for (uint32_t i = 0; i < m_uniformBuffers.size(); ++i) {
			m_uniformBuffers[i].size = sizeof(ShaderData);
			m_uniformBuffers[i].usageFlags = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
			m_uniformBuffers[i].memoryPropertyFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
			createBuffer(m_device, m_uniformBuffers[i], m_deviceMemoryProperties);
		}
		};
	funcCreateUniformBuffers();

	//return;

	//descriptor set layout
	createVKDescriptorSetLayout();

	//layout
	createVKPipelineLayout();

	////descriptor pool多个描述符组成描述符集，描述符集存在描述符池
	createVKDescriptorPool();

	//descriptor set
	createVKDescriptorSets();

	//pipeline
	auto funcCreateGraphicsPipline = [&]() {
		//auto vertShaderCode = readFile(shaderPath + "triangle/triangle.vert.spv");
		//auto fragShaderCode = readFile(shaderPath + "triangle/triangle.frag.spv");
		auto vertShaderCode = readFile(shaderPath + "texture/texture.vert.spv");
		auto fragShaderCode = readFile(shaderPath + "texture/texture.frag.spv");
		VkShaderModule vertShaderModule = createShaderModule(m_device, vertShaderCode);
		VkShaderModule fragShaderModule = createShaderModule(m_device, fragShaderCode);
		VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
		vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
		vertShaderStageInfo.module = vertShaderModule;
		vertShaderStageInfo.pName = "main";
		VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
		fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
		fragShaderStageInfo.module = fragShaderModule;
		fragShaderStageInfo.pName = "main";
		VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

		VkGraphicsPipelineCreateInfo pipelineCreateInfo{};
		pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;

		pipelineCreateInfo.basePipelineIndex = -1;

		pipelineCreateInfo.layout = m_layout;

		pipelineCreateInfo.renderPass = m_renderPass;

		pipelineCreateInfo.stageCount = 2;
		pipelineCreateInfo.pStages = shaderStages;

		std::vector<VkVertexInputBindingDescription> vertexInputBindings(1);
		vertexInputBindings[0].binding = 0;
		vertexInputBindings[0].stride = sizeof(Vertex);
		vertexInputBindings[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
		std::vector<VkVertexInputAttributeDescription> vertexInputAttributes(3);
		vertexInputAttributes[0].location = 0;
		vertexInputAttributes[0].binding = 0;
		vertexInputAttributes[0].format = VK_FORMAT_R32G32B32_SFLOAT;
		vertexInputAttributes[0].offset = offsetof(Vertex, pos);
		vertexInputAttributes[1].location = 1;
		vertexInputAttributes[1].binding = 0;
		vertexInputAttributes[1].format = VK_FORMAT_R32G32_SFLOAT;
		vertexInputAttributes[1].offset = offsetof(Vertex, uv);
		vertexInputAttributes[2].location = 2;
		vertexInputAttributes[2].binding = 0;
		vertexInputAttributes[2].format = VK_FORMAT_R32G32B32_SFLOAT;
		vertexInputAttributes[2].offset = offsetof(Vertex, normal);
		VkPipelineVertexInputStateCreateInfo vertexInputStateCreateInfo{};
		vertexInputStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		vertexInputStateCreateInfo.vertexBindingDescriptionCount = (uint32_t)vertexInputBindings.size();
		vertexInputStateCreateInfo.pVertexBindingDescriptions = vertexInputBindings.data();
		vertexInputStateCreateInfo.vertexAttributeDescriptionCount = (uint32_t)vertexInputAttributes.size();
		vertexInputStateCreateInfo.pVertexAttributeDescriptions = vertexInputAttributes.data();
		pipelineCreateInfo.pVertexInputState = &vertexInputStateCreateInfo;

		VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCreateInfo{};
		inputAssemblyStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		inputAssemblyStateCreateInfo.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;//VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
		pipelineCreateInfo.pInputAssemblyState = &inputAssemblyStateCreateInfo;

		VkPipelineTessellationStateCreateInfo tessellationStateCreateInfo{}; //图元细分阶段
		tessellationStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO;
		pipelineCreateInfo.pTessellationState = &tessellationStateCreateInfo;

		std::vector<VkViewport> viewports;
		VkViewport vp{ 0.f, 0.f, float(m_swapChainExtent.width), float(m_swapChainExtent.height), 0.f, 1.f };
		viewports.push_back(vp);
		std::vector<VkRect2D> scissors;
		VkOffset2D of2d{ 0,0 };
		VkRect2D sr{ of2d, m_swapChainExtent };
		scissors.push_back(sr);
		VkPipelineViewportStateCreateInfo viewportStateCreateInfo{};
		viewportStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
		viewportStateCreateInfo.viewportCount = (uint32_t)viewports.size();
		viewportStateCreateInfo.pViewports = viewports.data();
		viewportStateCreateInfo.scissorCount = (uint32_t)scissors.size();
		viewportStateCreateInfo.pScissors = scissors.data();
		pipelineCreateInfo.pViewportState = &viewportStateCreateInfo;

		VkPipelineRasterizationStateCreateInfo rasterizationStateCreateInfo{};
		rasterizationStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
		pipelineCreateInfo.pRasterizationState = &rasterizationStateCreateInfo;

		VkPipelineMultisampleStateCreateInfo multistampleStateCreateInfo{};
		multistampleStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
		multistampleStateCreateInfo.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
		pipelineCreateInfo.pMultisampleState = &multistampleStateCreateInfo;

		VkPipelineDepthStencilStateCreateInfo depthStencilStateCreateInfo{};
		depthStencilStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
		pipelineCreateInfo.pDepthStencilState = &depthStencilStateCreateInfo;

		std::vector<VkPipelineColorBlendAttachmentState> colorBlendAttachmentStates;
		VkPipelineColorBlendAttachmentState tmp{};
		tmp.colorWriteMask = 0b111;
		colorBlendAttachmentStates.push_back(tmp);
		VkPipelineColorBlendStateCreateInfo colorBlendStateCreateInfo{};
		colorBlendStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
		colorBlendStateCreateInfo.attachmentCount = (uint32_t)colorBlendAttachmentStates.size();
		colorBlendStateCreateInfo.pAttachments = colorBlendAttachmentStates.data();
		pipelineCreateInfo.pColorBlendState = &colorBlendStateCreateInfo;

		std::vector<VkDynamicState> dynamicStates;
		//dynamicStates.push_back(VK_DYNAMIC_STATE_VIEWPORT);
		//dynamicStates.push_back(VK_DYNAMIC_STATE_SCISSOR);
		VkPipelineDynamicStateCreateInfo dynamicStateCreateInfo{};
		dynamicStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
		dynamicStateCreateInfo.dynamicStateCount = (uint32_t)dynamicStates.size();
		dynamicStateCreateInfo.pDynamicStates = dynamicStates.data();
		pipelineCreateInfo.pDynamicState = &dynamicStateCreateInfo;

		if (vkCreateGraphicsPipelines(m_device, nullptr, 1, &pipelineCreateInfo, nullptr, &m_pipeline) != VK_SUCCESS) {
			throw std::runtime_error("failed to create pipeline");
		}

		vkDestroyShaderModule(m_device, fragShaderModule, nullptr);
		vkDestroyShaderModule(m_device, vertShaderModule, nullptr);
		};
	funcCreateGraphicsPipline();

	//command buffers
	auto funcCreateCommandBuffers = [&]() {
		m_commandBuffers.resize(m_swapChainFramebuffers.size());
		VkCommandBufferAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		allocInfo.commandPool = m_graphicsCommandPool;
		allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		allocInfo.commandBufferCount = (uint32_t)m_commandBuffers.size();
		if (vkAllocateCommandBuffers(m_device, &allocInfo, m_commandBuffers.data()) != VK_SUCCESS) {
			throw std::runtime_error("failed to create command buffers");
		}
		VkCommandBufferAllocateInfo allocInfo2{};
		allocInfo2.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		allocInfo2.commandPool = m_graphicsCommandPool;
		allocInfo2.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		allocInfo2.commandBufferCount = (uint32_t)1;
		if (vkAllocateCommandBuffers(m_device, &allocInfo2, &m_cmdBuffer) != VK_SUCCESS) {
			throw std::runtime_error("failed to create command buffers");
		}

		for (size_t i = 0; i < m_commandBuffers.size(); ++i) {
			VkCommandBufferBeginInfo beginInfo{};
			beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			if (vkBeginCommandBuffer(m_commandBuffers[i], &beginInfo) != VK_SUCCESS) {
				throw std::runtime_error("failed to begin recording command buffer");
			}

			VkRenderPassBeginInfo renderPassInfo{};
			renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassInfo.renderPass = m_renderPass;
			renderPassInfo.framebuffer = m_swapChainFramebuffers[i];
			renderPassInfo.renderArea.offset = { 0,0 };
			renderPassInfo.renderArea.extent = m_swapChainExtent;
			VkClearValue clearColor = { 0.0f, 0.0f, 0.0f, 1.0f };
			renderPassInfo.clearValueCount = 1;
			renderPassInfo.pClearValues = &clearColor;
			vkCmdBeginRenderPass(m_commandBuffers[i], &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
			vkCmdEndRenderPass(m_commandBuffers[i]);

			if (vkEndCommandBuffer(m_commandBuffers[i]) != VK_SUCCESS) {
				throw std::runtime_error("failed to record command buffer");
			}
		}
		};
	funcCreateCommandBuffers();
	return;
}
void VulkanImp::mainLoop() {
	while (!glfwWindowShouldClose(m_windows)) {
		drawFrame();

		glfwPollEvents();
		vkWaitForFences(m_device, 1, &m_inFlightFence, VK_FALSE, UINT64_MAX);
		vkResetFences(m_device, 1, &m_inFlightFence);
	}
	vkDeviceWaitIdle(m_device);
	glfwTerminate();
}
void VulkanImp::drawFrame() {
	uint32_t imageIndex;
	vkAcquireNextImageKHR(m_device, m_swapChain, UINT64_MAX, m_imageAvailableSemaphore, VK_NULL_HANDLE, &imageIndex);

	ShaderData shaderData{};
	float xof = (m_cnt++ / 100) * 0.1f;
	xof = 0.0f;
	shaderData.projectionMatrix = glm::mat4( //列为主
		1.0f, 0.0f, 0.0f, 0.0f, //第一列
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		xof, 0.0f, 0.0f, 1.0f
	);
	shaderData.viewMatrix = glm::mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
	//shaderData.moduleMatrix = glm::mat4(1.0f);
	shaderData.moduleMatrix = glm::mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
	memcpy(m_uniformBuffers[imageIndex].mapped, &shaderData, sizeof(ShaderData));

	VkCommandBufferBeginInfo cmdBufferBeginInfo{};
	cmdBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	cmdBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	if (vkBeginCommandBuffer(m_cmdBuffer, &cmdBufferBeginInfo) != VK_SUCCESS) {
		throw std::runtime_error("failed to begin recording command buffer");
	}
	VkRenderPassBeginInfo renderPassBeginInfo{};
	renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassBeginInfo.renderPass = m_renderPass;
	renderPassBeginInfo.framebuffer = m_swapChainFramebuffers[imageIndex];
	if (imageIndex % 2 == 0 || true) {
		renderPassBeginInfo.renderArea.offset.x = 0;
		renderPassBeginInfo.renderArea.offset.y = 0;
		renderPassBeginInfo.renderArea.extent.height = m_swapChainExtent.height;
		renderPassBeginInfo.renderArea.extent.width = m_swapChainExtent.width;
	}
	else {
		renderPassBeginInfo.renderArea.offset.x = m_swapChainExtent.width / 2;
		renderPassBeginInfo.renderArea.offset.y = 0;
		renderPassBeginInfo.renderArea.extent.height = m_swapChainExtent.height;
		renderPassBeginInfo.renderArea.extent.width = m_swapChainExtent.width / 2;
	}
	renderPassBeginInfo.clearValueCount = 1;
	renderPassBeginInfo.pClearValues = &m_clearColor;
	vkCmdBeginRenderPass(m_cmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
	vkCmdBindDescriptorSets(m_cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_layout, 0, 1, &m_uniformBuffers[imageIndex].descriptorSet, 0, nullptr);
	vkCmdBindPipeline(m_cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline);

	//vkCmdDraw(m_cmdBuffer, 3, 1, 0, 0);
	VkDeviceSize offsets[1] = { 0 };
	vkCmdBindVertexBuffers(m_cmdBuffer, 0, 1, &m_vertexBuffer.buffer, offsets);
	vkCmdBindIndexBuffer(m_cmdBuffer, m_indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
	vkCmdDrawIndexed(m_cmdBuffer, m_indexCount, 1, 0, 0, 0);

	vkCmdEndRenderPass(m_cmdBuffer);
	if (vkEndCommandBuffer(m_cmdBuffer) != VK_SUCCESS) {
		throw std::runtime_error("failed to record command buffer");
	}

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	VkSemaphore waitSemaphores[] = { m_imageAvailableSemaphore };
	VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
	submitInfo.waitSemaphoreCount = 1;
	submitInfo.pWaitSemaphores = waitSemaphores;
	submitInfo.pWaitDstStageMask = waitStages;
	submitInfo.commandBufferCount = 1;
	//submitInfo.pCommandBuffers = &m_commandBuffers[imageIndex];
	submitInfo.pCommandBuffers = &m_cmdBuffer;
	VkSemaphore signalSemaphores[] = { m_renderFinishedSemaphore };
	submitInfo.signalSemaphoreCount = 1;
	submitInfo.pSignalSemaphores = signalSemaphores;
	if (vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_inFlightFence) != VK_SUCCESS) {
		throw std::runtime_error("failed to submit draw command buffer");
	}

	VkPresentInfoKHR presentInfo{};
	presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
	presentInfo.waitSemaphoreCount = 1;
	presentInfo.pWaitSemaphores = signalSemaphores;
	VkSwapchainKHR swapChains[] = { m_swapChain };
	presentInfo.swapchainCount = 1;
	presentInfo.pSwapchains = swapChains;
	presentInfo.pImageIndices = &imageIndex;
	if (vkQueuePresentKHR(m_presentQueue, &presentInfo) != VK_SUCCESS) {
		throw std::runtime_error("failed to present image");
	}
}
void VulkanImp::cleanup() {
	vkDestroyFence(m_device, m_inFlightFence, nullptr);
	vkDestroySemaphore(m_device, m_imageAvailableSemaphore, nullptr);
	vkDestroySemaphore(m_device, m_renderFinishedSemaphore, nullptr);
	vkDestroyPipelineLayout(m_device, m_layout, nullptr);
	vkDestroyPipeline(m_device, m_pipeline, nullptr);
	for (auto uniformBuffer : m_uniformBuffers) {
		vkDestroyBuffer(m_device, uniformBuffer.buffer, nullptr);
		vkFreeMemory(m_device, uniformBuffer.memory, nullptr);
	}
	vkDestroyBuffer(m_device, m_deviceBuffer.buffer, nullptr);
	vkFreeMemory(m_device, m_deviceBuffer.memory, nullptr);
	vkDestroyBuffer(m_device, m_hostBuffer.buffer, nullptr);
	vkFreeMemory(m_device, m_hostBuffer.memory, nullptr);
	vkDestroyBuffer(m_device, m_vertexBuffer.buffer, nullptr);
	vkFreeMemory(m_device, m_vertexBuffer.memory, nullptr);
	vkDestroyBuffer(m_device, m_vertexBufferTmp.buffer, nullptr);
	vkFreeMemory(m_device, m_vertexBufferTmp.memory, nullptr);
	vkDestroyBuffer(m_device, m_indexBuffer.buffer, nullptr);
	vkFreeMemory(m_device, m_indexBuffer.memory, nullptr);
	vkDestroyBuffer(m_device, m_indexBufferTmp.buffer, nullptr);
	vkFreeMemory(m_device, m_indexBufferTmp.memory, nullptr);
	for (auto framebuffer : m_swapChainFramebuffers) {
		vkDestroyFramebuffer(m_device, framebuffer, nullptr);
	}
	vkFreeCommandBuffers(m_device, m_computeCommandPool, 1, &m_cmdBufferCompute);
	vkFreeCommandBuffers(m_device, m_graphicsCommandPool, 1, &m_cmdBuffer);
	vkDestroyCommandPool(m_device, m_computeCommandPool, nullptr);
	vkDestroyCommandPool(m_device, m_graphicsCommandPool, nullptr);
	for (auto imageView : m_swapChainImageViews)
		vkDestroyImageView(m_device, imageView, nullptr);
	vkDestroySwapchainKHR(m_device, m_swapChain, nullptr);
	vkDestroyDevice(m_device, nullptr);
	vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
	vkDestroyInstance(m_instance, nullptr);
	glfwDestroyWindow(m_windows);
	glfwTerminate();
}




bool VulkanImp::createVKInstance()
{
	VkApplicationInfo appInfo{};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pApplicationName = "Vulkan compute";
	appInfo.pApplicationName = "Hello Triangle";
	appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.pEngineName = "No Engine";
	appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.apiVersion = VK_API_VERSION_1_0;

	VkInstanceCreateInfo instanceCreateInfo{};
	instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	instanceCreateInfo.pApplicationInfo = &appInfo;

	uint32_t glfwExtCnt = 0;
	const char** glfwExts;
	glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCnt);
	instanceCreateInfo.enabledExtensionCount = glfwExtCnt;
	instanceCreateInfo.ppEnabledExtensionNames = glfwExts;

	//no validation
	std::vector<const char*> validationLayers;
	if (false) {
		uint32_t layerCount = 0;
		vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
		std::vector<VkLayerProperties> instanceLayers(layerCount);
		vkEnumerateInstanceLayerProperties(&layerCount, instanceLayers.data());
		validationLayers.push_back("VK_LAYER_KHRONOS_validation");
	}
	instanceCreateInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
	instanceCreateInfo.ppEnabledLayerNames = validationLayers.data();

	VK_CHECK_RESULT(vkCreateInstance(&instanceCreateInfo, nullptr, &m_instance));

	return true;
}
bool VulkanImp::createVKPhysicalDevice(bool _onlyCompute) {
	//physical device
	//确定图形、显示、计算队列
	auto funcFindQueueFamilies = [&](VkPhysicalDevice device)->QueueFamilyIndices {
		QueueFamilyIndices indices;
		uint32_t queueFamilyCnt = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCnt, nullptr);
		std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCnt);
		vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCnt, queueFamilies.data());
		int i = 0;
		for (const auto& queueFamily : queueFamilies) {
			if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT)
				indices.graphicsFamily = i;
			VkBool32 presentSupport = false;
			vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_surface, &presentSupport);
			if (presentSupport)
				indices.presentFamily = i;
			if (queueFamily.queueFlags & VK_QUEUE_COMPUTE_BIT)
				indices.computeFamily = i;
			++i;
		}

		return indices;
		};
	auto funcDeviceExtensionSupport = [&](VkPhysicalDevice device) {
		uint32_t extensionCnt = 0;
		vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCnt, nullptr);
		std::vector<VkExtensionProperties> availableExtensions(extensionCnt);
		vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCnt, availableExtensions.data());
		std::set<std::string> requiredExtensions(m_deviceExtensions.begin(), m_deviceExtensions.end());
		for (const auto& extension : availableExtensions) {
			requiredExtensions.erase(extension.extensionName);
		}
		return requiredExtensions.empty();
		};
	auto funcQuerySwapChainSupport = [&](VkPhysicalDevice device) {
		SwapChainSupportDetails details;
		vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, m_surface, &details.capabilities);
		uint32_t formatCnt;
		vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &formatCnt, nullptr);
		if (formatCnt != 0) {
			details.formats.resize(formatCnt);
			vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &formatCnt, details.formats.data());
		}
		uint32_t presentModeCnt;
		vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface, &presentModeCnt, nullptr);
		if (presentModeCnt != 0) {
			details.presentModes.resize(presentModeCnt);
			vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface, &presentModeCnt, nullptr);
		}
		return details;
		};
	auto funcDeviceSuitable = [&](VkPhysicalDevice device) {
		vkGetPhysicalDeviceProperties(device, &m_deviceProperties);
		vkGetPhysicalDeviceFeatures(device, &m_deviceFeatures);
		vkGetPhysicalDeviceMemoryProperties(device, &m_deviceMemoryProperties);

		QueueFamilyIndices indices = funcFindQueueFamilies(device);

		bool extensionsSupported = funcDeviceExtensionSupport(device);

		if (_onlyCompute)
			return extensionsSupported && indices.isCompleteCompute();

		bool swapChainAdequate = false;
		if (extensionsSupported) {
			m_details = funcQuerySwapChainSupport(device);
			swapChainAdequate = !m_details.formats.empty() && !m_details.presentModes.empty();
		}

		m_queueIndices = indices;
		return m_deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU && m_deviceFeatures.geometryShader\
			&& indices.isComplete() && extensionsSupported && swapChainAdequate \
			&& indices.isCompleteCompute();
		};
	auto funcPickPhysicalDevice = [&]()->bool {
		uint32_t deviceCnt = 0;
		vkEnumeratePhysicalDevices(m_instance, &deviceCnt, nullptr);
		if (deviceCnt == 0) {
			std::cout << "failed to find GPUs" << std::endl;
			return false;
		}
		std::vector<VkPhysicalDevice> devices(deviceCnt);
		vkEnumeratePhysicalDevices(m_instance, &deviceCnt, devices.data());
		for (const auto& device : devices) {
			if (funcDeviceSuitable(device)) {
				m_physicalDevice = device;
				return true;
			}
		}
		return false;
		};
	funcPickPhysicalDevice();
	if (m_physicalDevice == VK_NULL_HANDLE) {
		throw std::runtime_error("failed to find suitable GPU");
	}
	return true;
}
bool VulkanImp::createVKDevice()
{
	std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
	std::set<uint32_t> uniqueQueueFailies = { m_queueIndices.graphicsFamily.value(), m_queueIndices.presentFamily.value(), m_queueIndices.computeFamily.value() };
	float queuePriority = 1.0f;
	for (uint32_t queueFamily : uniqueQueueFailies) {
		VkDeviceQueueCreateInfo queueCreateInfo{};
		queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreateInfo.queueFamilyIndex = queueFamily;
		queueCreateInfo.queueCount = 1;
		queueCreateInfo.pQueuePriorities = &queuePriority;
		queueCreateInfos.push_back(queueCreateInfo);
	}
	VkDeviceCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
	createInfo.pQueueCreateInfos = queueCreateInfos.data();
	createInfo.pEnabledFeatures = &m_deviceFeatures;
	createInfo.enabledExtensionCount = static_cast<uint32_t>(m_deviceExtensions.size());
	createInfo.ppEnabledExtensionNames = m_deviceExtensions.data();
	createInfo.enabledLayerCount = 0;
	if (vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device) != VK_SUCCESS)
		throw std::runtime_error("failed to create logical device");
	return true;
}
bool VulkanImp::createVKQueue()
{
	if (m_queueIndices.isComplete()) {
		vkGetDeviceQueue(m_device, m_queueIndices.graphicsFamily.value(), 0, &m_graphicsQueue);
		vkGetDeviceQueue(m_device, m_queueIndices.presentFamily.value(), 0, &m_presentQueue);
	}
	if (m_queueIndices.isCompleteCompute())
		vkGetDeviceQueue(m_device, m_queueIndices.computeFamily.value(), 0, &m_computeQueue);
	return true;
}
bool VulkanImp::createVKCommandPool()
{
	VkCommandPoolCreateInfo cmdPoolCreateInfo{};
	cmdPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	cmdPoolCreateInfo.queueFamilyIndex = m_queueIndices.computeFamily.value();
	cmdPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	VK_CHECK_RESULT(vkCreateCommandPool(m_device, &cmdPoolCreateInfo, nullptr, &m_computeCommandPool));
	cmdPoolCreateInfo.queueFamilyIndex = m_queueIndices.graphicsFamily.value();
	if (vkCreateCommandPool(m_device, &cmdPoolCreateInfo, nullptr, &m_graphicsCommandPool) != VK_SUCCESS) {
		throw std::runtime_error("failed to create command pool");
	}
	return true;
}
bool VulkanImp::createVKDescriptorPool()
{
	std::vector<VkDescriptorPoolSize> descriptorTypeCounts(4); //描述符定义，类型+数量
	descriptorTypeCounts[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	descriptorTypeCounts[0].descriptorCount = (uint32_t)m_swapChainImages.size() * 2;
	descriptorTypeCounts[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	descriptorTypeCounts[1].descriptorCount = 2;
	descriptorTypeCounts[2].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorTypeCounts[2].descriptorCount = 4;
	descriptorTypeCounts[3].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorTypeCounts[3].descriptorCount = 2;
	VkDescriptorPoolCreateInfo descriptorPoolInfo{};
	descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	descriptorPoolInfo.pNext = nullptr;
	descriptorPoolInfo.poolSizeCount = (uint32_t)descriptorTypeCounts.size();
	descriptorPoolInfo.pPoolSizes = descriptorTypeCounts.data();
	descriptorPoolInfo.maxSets = (uint32_t)m_swapChainImages.size() + (uint32_t)descriptorTypeCounts.size(); //最大描述符集数量定义，总和不能超出上面VkDescriptorPoolSize的总和
	VK_CHECK_RESULT(vkCreateDescriptorPool(m_device, &descriptorPoolInfo, nullptr, &m_descPool));
	return true;
}
bool VulkanImp::createVKDescriptorSetLayout()
{
	std::vector<VkDescriptorSetLayoutBinding> layoutBinding(4); //创建描述符集的绑定点，可以有多个相同的类型（大小可以不同？），对应shader中传入的数组
	layoutBinding[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	layoutBinding[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	layoutBinding[0].descriptorCount = 1;
	layoutBinding[0].pImmutableSamplers = nullptr;
	layoutBinding[0].binding = 0;
	layoutBinding[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	layoutBinding[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
	layoutBinding[1].descriptorCount = 1;
	layoutBinding[1].pImmutableSamplers = nullptr;
	layoutBinding[1].binding = 1;
	layoutBinding[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	layoutBinding[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	layoutBinding[2].descriptorCount = 2;
	layoutBinding[2].binding = 2;
	layoutBinding[2].pImmutableSamplers = nullptr;
	layoutBinding[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	layoutBinding[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	layoutBinding[3].descriptorCount = 1;
	layoutBinding[3].binding = 3;
	layoutBinding[3].pImmutableSamplers = nullptr;
	VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo{}; //创建描述符集布局，可以有多个绑定点，对应shader中的binding
	descriptorLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	descriptorLayoutInfo.pNext = nullptr;
	descriptorLayoutInfo.bindingCount = (uint32_t)layoutBinding.size();
	descriptorLayoutInfo.pBindings = layoutBinding.data();
	VK_CHECK_RESULT(vkCreateDescriptorSetLayout(m_device, &descriptorLayoutInfo, nullptr, &m_descriptorSetLayout));
	return true;
}
bool VulkanImp::createVKPipelineLayout()
{
	std::vector<VkDescriptorSetLayout> layouts = { m_descriptorSetLayout  ,m_descriptorSetLayout };
	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo{};//创建多个描述符集布局，布局可以不同，顺序对应shader中的set
	pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutCreateInfo.pSetLayouts = layouts.data();
	pipelineLayoutCreateInfo.setLayoutCount = (uint32_t)layouts.size();
	VK_CHECK_RESULT(vkCreatePipelineLayout(m_device, &pipelineLayoutCreateInfo, nullptr, &m_layout));
	return true;
}
bool VulkanImp::createVKDescriptorSets(bool _bCompute)
{
	if (_bCompute)
	{
		std::vector<VkDescriptorSetLayout> layouts = { m_descriptorSetLayout  ,m_descriptorSetLayout };
		VkDescriptorSetAllocateInfo descriptorSetAllocateInfo{}; //在描述符池中分配描述符集，最大不能超过池中上限，布局可以不同
		descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorSetAllocateInfo.descriptorPool = m_descPool;
		descriptorSetAllocateInfo.descriptorSetCount = (uint32_t)layouts.size(); //内部包含n个描述符集
		descriptorSetAllocateInfo.pSetLayouts = layouts.data();
		std::vector<VkDescriptorSet> descriptors(2);
		VK_CHECK_RESULT(vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo, descriptors.data()));
		m_deviceBuffer.descriptorSet = descriptors[0];
		//进行描述
		m_deviceBuffer.bufferInfo.buffer = m_deviceBuffer.buffer; //描述的buffer
		m_deviceBuffer.bufferInfo.offset = 0; //偏移
		m_deviceBuffer.bufferInfo.range = VK_WHOLE_SIZE; //范围
		VkWriteDescriptorSet writeDescriptorSet{}; //将描述绑定到对应描述符集中
		writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writeDescriptorSet.descriptorCount = 1; //有多个则在shader中用数组取
		writeDescriptorSet.dstSet = m_deviceBuffer.descriptorSet; //写入的描述符集
		writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writeDescriptorSet.dstBinding = 2; //描述符集中的绑定点
		writeDescriptorSet.pBufferInfo = &m_deviceBuffer.bufferInfo;
		std::vector< VkWriteDescriptorSet> writeDescriptorSets{ writeDescriptorSet };
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, NULL); //更新描述符集

		//VK_CHECK_RESULT(vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo, &m_deviceBuffer2.descriptorSet));
		m_deviceBuffer2.descriptorSet = descriptors[1];
		m_deviceBuffer2.bufferInfo.buffer = m_deviceBuffer2.buffer;
		m_deviceBuffer2.bufferInfo.offset = 0;
		m_deviceBuffer2.bufferInfo.range = VK_WHOLE_SIZE;
		VkWriteDescriptorSet writeDescriptorSet2{};
		writeDescriptorSet2.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writeDescriptorSet2.descriptorCount = 1;
		writeDescriptorSet2.dstSet = m_deviceBuffer2.descriptorSet;
		writeDescriptorSet2.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writeDescriptorSet2.dstBinding = 3;
		writeDescriptorSet2.pBufferInfo = &m_deviceBuffer2.bufferInfo;
		std::vector< VkWriteDescriptorSet> writeDescriptorSets2{ writeDescriptorSet2 };
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writeDescriptorSets2.size()), writeDescriptorSets2.data(), 0, NULL);
		return true;
	}
	for (uint32_t i = 0; i < m_swapChainImages.size(); ++i) {
		VkDescriptorSetAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		allocInfo.descriptorPool = m_descPool;
		allocInfo.descriptorSetCount = 1;
		allocInfo.pSetLayouts = &m_descriptorSetLayout;
		VK_CHECK_RESULT(vkAllocateDescriptorSets(m_device, &allocInfo, &m_uniformBuffers[i].descriptorSet));
		VkDescriptorImageInfo textureDescriptor;
		textureDescriptor.imageView = m_texture2DView;
		textureDescriptor.sampler = m_sampler;
		textureDescriptor.imageLayout = m_textureLayout;
		m_uniformBuffers[i].bufferInfo.buffer = m_uniformBuffers[i].buffer;
		m_uniformBuffers[i].bufferInfo.range = sizeof(ShaderData);
		std::vector<VkWriteDescriptorSet> writeDescriptorSet(2);
		writeDescriptorSet[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writeDescriptorSet[0].dstSet = m_uniformBuffers[i].descriptorSet;
		writeDescriptorSet[0].descriptorCount = 1;
		writeDescriptorSet[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		writeDescriptorSet[0].pBufferInfo = &m_uniformBuffers[i].bufferInfo;
		writeDescriptorSet[0].dstBinding = 0; //与VkDescriptorSetLayoutBinding中的binding相同
		writeDescriptorSet[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writeDescriptorSet[1].dstSet = m_uniformBuffers[i].descriptorSet;
		writeDescriptorSet[1].descriptorCount = 1;
		writeDescriptorSet[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		writeDescriptorSet[1].pImageInfo = &textureDescriptor;
		writeDescriptorSet[1].dstBinding = 1;
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, nullptr);
	}
	return true;
}
bool VulkanImp::createVKPipelineCahce()
{
	VkPipelineCacheCreateInfo pipelineCacheCreateInfo{};
	pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	VK_CHECK_RESULT(vkCreatePipelineCache(m_device, &pipelineCacheCreateInfo, nullptr, &m_pipelineCache));
	return true;
}
bool VulkanImp::createVKFenceAndSemaphore()
{
	VkFenceCreateInfo fenceCreateInfo{};
	fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceCreateInfo.flags = 0;
	if (vkCreateFence(m_device, &fenceCreateInfo, nullptr, &m_inFlightFence) != VK_SUCCESS) {
		throw std::runtime_error("failed to create fence");
	}
	VkSemaphoreCreateInfo semaphoreCreateInfo{};
	semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	if (vkCreateSemaphore(m_device, &semaphoreCreateInfo, nullptr, &m_imageAvailableSemaphore) != VK_SUCCESS) {
		throw std::runtime_error("failed to create semaphore1");
	}
	VkSemaphoreCreateInfo semaphoreCreateInfo2{};
	semaphoreCreateInfo2.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	if (vkCreateSemaphore(m_device, &semaphoreCreateInfo2, nullptr, &m_renderFinishedSemaphore) != VK_SUCCESS) {
		throw std::runtime_error("failed to create semaphore2");
	}
	return true;
}

NSP_SLAM_LYJ_END