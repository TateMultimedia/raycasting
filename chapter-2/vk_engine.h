// vulkan_guide.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <vk_types.h>
#include <vector>
#include <functional>
#include"gltfloading.h"
#include <deque>
#include <unordered_map>
class PipelineBuilder {
public:

	std::vector<VkPipelineShaderStageCreateInfo> _shaderStages;
	VkPipelineVertexInputStateCreateInfo _vertexInputInfo;
	VkPipelineInputAssemblyStateCreateInfo _inputAssembly;
	VkViewport _viewport;
	VkRect2D _scissor;
	VkPipelineRasterizationStateCreateInfo _rasterizer;
	VkPipelineColorBlendAttachmentState _colorBlendAttachment;
	VkPipelineMultisampleStateCreateInfo _multisampling;
	VkPipelineLayout _pipelineLayout;

	VkPipeline build_pipeline(VkDevice device, VkRenderPass pass);
};


struct Material {
	VkDescriptorSet textureSet{ VK_NULL_HANDLE };
	VkPipeline pipeline;
	VkPipelineLayout pipelineLayout;
};


struct DeletionQueue
{
    std::deque<std::function<void()>> deletors;

    void push_function(std::function<void()>&& function) {
        deletors.push_back(function);
    }

    void flush() {
        // reverse iterate the deletion queue to execute all the functions
        for (auto it = deletors.rbegin(); it != deletors.rend(); it++) {
            (*it)(); //call functors
        }

        deletors.clear();
    }
};

struct UploadContext {
	VkFence _uploadFence;
	VkCommandPool _commandPool;
	VkCommandBuffer _commandBuffer;
};
struct Texture {
	AllocatedImage image;
	VkImageView imageView;
};


class VulkanEngine {
public:

	bool _isInitialized{ false };
	int _frameNumber {0};

	VkExtent2D _windowExtent{ 1700 , 900 };

	struct SDL_Window* _window{ nullptr };

	VkInstance _instance;
	VkDebugUtilsMessengerEXT _debug_messenger;
	VkPhysicalDevice _chosenGPU;
	VkDevice _device;

	VkSemaphore _presentSemaphore, _renderSemaphore, _computeSemaphore;
	VkFence _renderFence;

	VkQueue _graphicsQueue;
	VkQueue _computeQueue;
	uint32_t _graphicsQueueFamily;
	uint32_t _computeQueueFamily;

	VkCommandPool _commandPool;
	VkCommandPool _compCommandPool;
	VkCommandBuffer _compCommandBuffer;
	VkCommandBuffer _mainCommandBuffer;

	AllocatedBuffer computeVerticesBuffer;
	VkDescriptorSet computeVerticesDescriptor;
	AllocatedBuffer computeColorBuffer;
	VkDescriptorSet computeColorDescriptor;
	AllocatedBuffer computeLightsBuffer;
	VkDescriptorSet computeLightsDescriptor;
	AllocatedBuffer computeNormalBuffer;
	VkDescriptorSet computeNormalDescriptor;
	AllocatedBuffer computeTrianglesBuffer;
	VkDescriptorSet computeTrianglesDescriptor;
	AllocatedBuffer computeUniformBuffer;
	VkDescriptorSet computeUniformDescriptor;


	AllocatedBuffer computeBVHBuffer[5];
	VkDescriptorSet computeBVHDescriptor[5];
	AllocatedBuffer computeBVHIdxBuffer;
	VkDescriptorSet computeBVHIdxDescriptor;
	VkRenderPass _renderPass;

	VulkanglTFModel *meshes;

	VkSurfaceKHR _surface;
	VkSwapchainKHR _swapchain;
	VkFormat _swachainImageFormat;

	std::vector<VkFramebuffer> _framebuffers;
	std::vector<VkImage> _swapchainImages;
	std::vector<VkImageView> _swapchainImageViews;

	VkPipelineLayout _trianglePipelineLayout;
	VkPipelineLayout _computePipelineLayout;
	VkPipelineCache pipelineCache;

	VkPipeline _redTrianglePipeline;
	VkPipeline _computePipeline;

    DeletionQueue _mainDeletionQueue;

	VmaAllocator _allocator; //vma lib allocator



	VkDescriptorSet _descriptorSet;				// Compute shader bindings

	VkDescriptorSetLayout _TextureSetLayout;
	VkDescriptorSetLayout _ComputeTextureSetLayout;
	VkDescriptorSetLayout _ComputeSetLayout;
	VkDescriptorSetLayout _ComputeVertDataSetLayout;
	VkDescriptorSetLayout _ComputeNormDataSetLayout;
	VkDescriptorSetLayout _ComputeColorDataSetLayout;
	VkDescriptorSetLayout _ComputeLightsDataSetLayout;
	VkDescriptorSetLayout _ComputeBVHDataSetLayout[5];
	VkDescriptorSetLayout _ComputeBVHIdxDataSetLayout;
	VkDescriptorSetLayout _ComputeDataSet2Layout;
	UploadContext _uploadContext;
	UploadContext _uploadComputeContext;
	//initializes everything in the engine
	void init();

	//shuts down the engine
	void cleanup();

	//draw loop
	void draw();

	//run main loop
	void run();
	void init_mesh();

	VkDescriptorPool _descriptorPool;
	std::unordered_map<std::string, Material> _materials;
	std::unordered_map<std::string, Texture> _loadedTextures;
	AllocatedBuffer create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage);
	void immediate_submit(std::function<void(VkCommandBuffer cmd)>&& function);
private:

	void init_vulkan();

	void init_swapchain();

	void init_default_renderpass();

	void init_framebuffers();

	void init_commands();

	void init_sync_structures();

	void init_pipelines();

	//loads a shader module from a spir-v file. Returns false if it errors
	bool load_shader_module(const char* filePath, VkShaderModule* outShaderModule);

	void load_images();
};
