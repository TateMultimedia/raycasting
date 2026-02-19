
#include "vk_engine.h"

#include <SDL.h>
#include <SDL_vulkan.h>

#include <vk_types.h>
#include <vk_initializers.h>
#include <vk_textures.h>

#include "VkBootstrap.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <glm/glm.hpp>

#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"
constexpr bool bUseValidationLayers = true;

struct BVHNode
{
	std::vector<glm::vec3> aabbMin, aabbMax;
	std::vector<int> leftChild, rightChild;
	std::vector<int> firstPrim, primCount;
};
class BVHTree {
public:
	BVHTree(VulkanglTFModel* models, std::vector<uint32_t> indexBuffer,	VulkanglTFModel::Vertex vertices);

	struct Tri { glm::vec3 vertex0, vertex1, vertex2, centroid; };
	void BuildBVH()
	{
		for (int i = 0; i < N; i++) {
			triIdx.push_back(i);
			int index = i * 3;
			Tri t;
			t.vertex0 = vertices.pos[indexBuffer[index]];
			t.vertex1 = vertices.pos[indexBuffer[index + 1]];
			t.vertex2 = vertices.pos[indexBuffer[index + 2]];			
			t.centroid = (t.vertex0 + t.vertex1 + t.vertex2) * 0.3333f;
			triangles.push_back(t);
		}
		// assign all triangles to root node
		bvhNode.leftChild[rootNodeIdx] = bvhNode.rightChild[rootNodeIdx] = 0;
		bvhNode.firstPrim[rootNodeIdx] = 0, bvhNode.primCount[rootNodeIdx] = N;
		UpdateNodeBounds(rootNodeIdx);
		// subdivide recursively
		Subdivide(rootNodeIdx);

	}

	void Subdivide(int nodeIdx)
	{
		// terminate recursion
		if (bvhNode.primCount[nodeIdx] <= 2) return;
		// determine split axis and position
		glm::vec3 extent = bvhNode.aabbMax[nodeIdx] - bvhNode.aabbMin[nodeIdx];
		int axis = 0;
		if (extent.y > extent.x) axis = 1;
		if (extent.z > extent[axis]) axis = 2;
		float splitPos = bvhNode.aabbMin[nodeIdx][axis] + extent[axis] * 0.5f;
		// in-place partition
		int i = bvhNode.firstPrim[nodeIdx];
		int j = i + bvhNode.primCount[nodeIdx] - 1;
		while (i <= j)
		{
			if (triangles[triIdx[i]].centroid[axis] < splitPos)
				i++;
			else
				std::swap(triIdx[i], triIdx[j--]);
		}
		// abort split if one of the sides is empty
		int leftCount = i - bvhNode.firstPrim[nodeIdx];
		if (leftCount == 0 || leftCount == bvhNode.primCount[nodeIdx]) return;
		// create child nodes
		int leftChildIdx = nodesUsed++;
		int rightChildIdx = nodesUsed++;
		bvhNode.leftChild[nodeIdx] = leftChildIdx;
		bvhNode.firstPrim[leftChildIdx] = bvhNode.firstPrim[nodeIdx];
		bvhNode.primCount[leftChildIdx] = leftCount;
		bvhNode.firstPrim[rightChildIdx] = i;
		bvhNode.primCount[rightChildIdx] = bvhNode.primCount[nodeIdx] - leftCount;
		bvhNode.primCount[nodeIdx] = 0;
		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);
		// recurse
		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}

	glm::vec3 fminf(glm::vec3 v1, glm::vec3 v2) {
		glm::vec3 o;
		o.x = v1.x < v2.x ? v1.x : v2.x;
		o.y = v1.y < v2.y ? v1.y : v2.y;
		o.z = v1.z < v2.z ? v1.z : v2.z;
		return o;
	}
	glm::vec3 fmaxf(glm::vec3 v1, glm::vec3 v2) {
		glm::vec3 o;
		o.x = v1.x > v2.x ? v1.x : v2.x;
		o.y = v1.y > v2.y ? v1.y : v2.y;
		o.z = v1.z > v2.z ? v1.z : v2.z;
		return o;
	}
	void UpdateNodeBounds(int nodeIdx)
	{
		bvhNode.aabbMin[nodeIdx] = glm::vec3(1e30f);
		bvhNode.aabbMax[nodeIdx] = glm::vec3(-1e30f);
		for (int first = bvhNode.firstPrim[nodeIdx], i = 0; i < bvhNode.primCount[nodeIdx]; i++)
		{
			int leafTriIdx = triIdx[first + i];
			Tri& leafTri = triangles[leafTriIdx];
			bvhNode.aabbMin[nodeIdx] = fminf(bvhNode.aabbMin[nodeIdx], leafTri.vertex0),
			bvhNode.aabbMin[nodeIdx] = fminf(bvhNode.aabbMin[nodeIdx], leafTri.vertex1),
			bvhNode.aabbMin[nodeIdx] = fminf(bvhNode.aabbMin[nodeIdx], leafTri.vertex2),
			bvhNode.aabbMax[nodeIdx] = fmaxf(bvhNode.aabbMax[nodeIdx], leafTri.vertex0),
			bvhNode.aabbMax[nodeIdx] = fmaxf(bvhNode.aabbMax[nodeIdx], leafTri.vertex1),
			bvhNode.aabbMax[nodeIdx] = fmaxf(bvhNode.aabbMax[nodeIdx], leafTri.vertex2);
		}
	}
	BVHNode bvhNode;
	std::vector <Tri> triangles;
	std::vector <int> triIdx;
	BVHNode* root = nullptr;
	int N;
	int rootNodeIdx = 0, nodesUsed = 1;
	std::pair<glm::vec3, glm::vec3> CalculateTriAABB(int index);
	VulkanglTFModel* models;	
	std::vector<uint32_t> indexBuffer;
	VulkanglTFModel::Vertex vertices;
};
BVHTree::BVHTree(VulkanglTFModel* models, std::vector<uint32_t> indexBuffer, VulkanglTFModel::Vertex vertices) {
	this->models = models;
	this->indexBuffer = indexBuffer;
	this->vertices = vertices;
	N = indexBuffer.size() / 3;
	int size = (N * 2 - 1);
	bvhNode.aabbMin.resize(size);
	bvhNode.aabbMax.resize(size);
	bvhNode.leftChild.resize(size);
	bvhNode.rightChild.resize(size);
	bvhNode.firstPrim.resize(size);
	bvhNode.primCount.resize(size);

	BuildBVH();
}

std::pair<glm::vec3, glm::vec3> BVHTree::CalculateTriAABB(int index) {
	glm::vec3 v[3];
	v[0] = vertices.pos[indexBuffer[index]];
	v[1] = vertices.pos[indexBuffer[index + 1]];
	v[2] = vertices.pos[indexBuffer[index + 2]];
	glm::vec3 AA = glm::vec3(float(INT32_MAX)), BB = glm::vec3(float(INT32_MIN));
	for (int i = 0; i < 3; i++) {
		if (v[i].x < AA.x)AA.x = v[i].x;
		if (v[i].y < AA.y)AA.y = v[i].y;
		if (v[i].z < AA.z)AA.z = v[i].z;
		if (v[i].x > BB.x)BB.x = v[i].x;
		if (v[i].y > BB.y)BB.y = v[i].y;
		if (v[i].z > BB.z)BB.z = v[i].z;
	}
	return std::pair<glm::vec3, glm::vec3>(AA, BB);
}
//we want to immediately abort when there is an error. In normal engines this would give an error message to the user, or perform a dump of state.
using namespace std;
#define VK_CHECK(x)                                                 \
	do                                                              \
	{                                                               \
		VkResult err = x;                                           \
		if (err)                                                    \
		{                                                           \
			std::cout <<"Detected Vulkan error: " << err << std::endl; \
			abort();                                                \
		}                                                           \
	} while (0)

struct ComputeBuffer {
	int numOfTris;
	int numOfLights;
	glm::ivec2 resolution = { 1280,720 };
	glm::mat4 invTrans;
}computeBuffer;

void VulkanEngine::init()
{
	computeBuffer.resolution = glm::ivec2( 1280,720 );
	// We initialize SDL and create a window with it. 
	SDL_Init(SDL_INIT_VIDEO);

	SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_VULKAN);
	
	_window = SDL_CreateWindow(
		"Vulkan Engine",
		SDL_WINDOWPOS_UNDEFINED,
		SDL_WINDOWPOS_UNDEFINED,
		_windowExtent.width,
		_windowExtent.height,
		window_flags
	);

	init_vulkan();

	init_swapchain();

	init_default_renderpass();

	init_framebuffers();

	init_commands();

	init_sync_structures();
	init_pipelines();
	load_images();
	init_mesh();
	std::chrono::milliseconds timespan(1000);
	std::this_thread::sleep_for(timespan);
	//everything went fine
	_isInitialized = true;
}
void VulkanEngine::cleanup()
{	
	if (_isInitialized) {
		
		//make sure the gpu has stopped doing its things
		vkDeviceWaitIdle(_device);

		_mainDeletionQueue.flush();

		vkDestroySurfaceKHR(_instance, _surface, nullptr);

		vkDestroyDevice(_device, nullptr);
		vkb::destroy_debug_utils_messenger(_instance, _debug_messenger);
		vkDestroyInstance(_instance, nullptr);

		SDL_DestroyWindow(_window);
	}
}

void VulkanEngine::draw()
{
	static auto start = chrono::steady_clock::now();
	static int count = 0;
	count++;
	auto end = chrono::steady_clock::now(); 
	float time = chrono::duration_cast<chrono::seconds>(end - start).count();
	if (time >= 5) {
		std::cout << "FPS: " << float(count) / time <<"\n";
		count = 0;
		start = chrono::steady_clock::now();
	}

	//check if window is minimized and skip drawing
	if (SDL_GetWindowFlags(_window) & SDL_WINDOW_MINIMIZED)
		return;

	//request image from the swapchain
	uint32_t swapchainImageIndex;
	VK_CHECK(vkAcquireNextImageKHR(_device, _swapchain, 1000000000000000000, _presentSemaphore, nullptr, &swapchainImageIndex));

	Material* computeMat;
	auto it = _materials.find("compute");
	if (it == _materials.end()) {
		computeMat = nullptr;
	}
	else {
		computeMat = &(*it).second;
	}

	VK_CHECK(vkWaitForFences(_device, 1, &_renderFence, true, 1000000000000000000));
	VK_CHECK(vkResetFences(_device, 1, &_renderFence));
	// Flush the queue if we're rebuilding the command buffer after a pipeline change to ensure it's not currently in use

	VkCommandBufferBeginInfo cmdBufferBeginInfo{};
	cmdBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

	VK_CHECK(vkBeginCommandBuffer(_compCommandBuffer, &cmdBufferBeginInfo));

	vkCmdBindPipeline(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipeline);
	if (computeMat->textureSet != VK_NULL_HANDLE) {
		//texture descriptor
		vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, computeMat->pipelineLayout, 0, 1, &computeMat->textureSet, 0, nullptr);

		vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 3, 1, &computeUniformDescriptor, 0, nullptr);
	}
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 1, 1, &computeVerticesDescriptor, 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 2, 1, &computeTrianglesDescriptor, 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 4, 1, &computeNormalDescriptor, 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 5, 1, &computeColorDescriptor, 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 6, 1, &computeLightsDescriptor, 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 7, 1, &computeBVHDescriptor[0], 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 8, 1, &computeBVHIdxDescriptor, 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 9, 1, &computeBVHDescriptor[1], 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 10, 1, &computeBVHDescriptor[2], 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 11, 1, &computeBVHDescriptor[3], 0, nullptr);
	vkCmdBindDescriptorSets(_compCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, _computePipelineLayout, 12, 1, &computeBVHDescriptor[4], 0, nullptr);

	vkCmdDispatch(_compCommandBuffer, computeBuffer.resolution.x / 16, computeBuffer.resolution.y / 16, 1);

	VK_CHECK(vkEndCommandBuffer(_compCommandBuffer));


	//naming it cmd for shorter writing
	VkCommandBuffer cmd = _mainCommandBuffer;

	//begin the command buffer recording. We will use this command buffer exactly once, so we want to let vulkan know that
	VkCommandBufferBeginInfo cmdBeginInfo = vkinit::command_buffer_begin_info(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

	VK_CHECK(vkBeginCommandBuffer(cmd, &cmdBeginInfo));

	//make a clear-color from frame number. This will flash with a 120 frame period.
	VkClearValue clearValue;
	float flash = abs(sin(_frameNumber / 120.f));
	clearValue.color = { { 0.0f, 0.0f, flash, 1.0f } };

	//start the main renderpass. 
	//We will use the clear color from above, and the framebuffer of the index the swapchain gave us
	VkRenderPassBeginInfo rpInfo = vkinit::renderpass_begin_info(_renderPass, _windowExtent, _framebuffers[swapchainImageIndex]);

	//connect clear values
	rpInfo.clearValueCount = 1;
	rpInfo.pClearValues = &clearValue;

	vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);


	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _redTrianglePipeline);

	Material* texturedMat;
	it = _materials.find("defaultmesh");
	if (it == _materials.end()) {
		texturedMat = nullptr;
	}
	else {
		texturedMat = &(*it).second;
	}
	if (texturedMat->textureSet != VK_NULL_HANDLE) {
		//texture descriptor
		vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, texturedMat->pipelineLayout, 0, 1, &texturedMat->textureSet, 0, nullptr);

	}

	vkCmdDraw(cmd, 6, 1, 0, 0);

	//finalize the render pass
	vkCmdEndRenderPass(cmd);
	//finalize the command buffer (we can no longer add commands, but it can now be executed)
	VK_CHECK(vkEndCommandBuffer(cmd));

	//prepare the submission to the queue. 
	//we want to wait on the _presentSemaphore, as that semaphore is signaled when the swapchain is ready
	//we will signal the _renderSemaphore, to signal that rendering has finished

	VkPipelineStageFlags waitStageMask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	// Submit compute commands


	VkSubmitInfo computeSubmitInfo = vkinit::submit_info(&_compCommandBuffer);
	computeSubmitInfo.commandBufferCount = 1;
	computeSubmitInfo.pCommandBuffers = &_compCommandBuffer;
	computeSubmitInfo.waitSemaphoreCount = 1;
	computeSubmitInfo.pWaitSemaphores = &_presentSemaphore;
	computeSubmitInfo.pWaitDstStageMask = &waitStageMask;
	computeSubmitInfo.signalSemaphoreCount = 1;
	computeSubmitInfo.pSignalSemaphores = &_computeSemaphore;
	vkQueueSubmit(_computeQueue, 1, &computeSubmitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(_computeQueue);
	VkSubmitInfo submit = vkinit::submit_info(&cmd);
	VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	VkPipelineStageFlags graphicsWaitStageMasks[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
	//VkSemaphore graphicsWaitSemaphores[] = { _computeSemaphore, _presentSemaphore };


	submit.waitSemaphoreCount = 1;
	submit.pWaitDstStageMask = graphicsWaitStageMasks;
	submit.pWaitSemaphores = &_computeSemaphore;
	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = &_renderSemaphore;

	//submit command buffer to the queue and execute it.
	// _renderFence will now block until the graphic commands finish execution
	VK_CHECK(vkQueueSubmit(_graphicsQueue, 1, &submit, _renderFence));
	vkQueueWaitIdle(_graphicsQueue);

	//prepare present
	// this will put the image we just rendered to into the visible window.
	// we want to wait on the _renderSemaphore for that, 
	// as its necessary that drawing commands have finished before the image is displayed to the user
	VkPresentInfoKHR presentInfo = vkinit::present_info();

	presentInfo.pSwapchains = &_swapchain;
	presentInfo.swapchainCount = 1;

	presentInfo.pWaitSemaphores = &_renderSemaphore;
	presentInfo.waitSemaphoreCount = 1;

	presentInfo.pImageIndices = &swapchainImageIndex;

	VK_CHECK(vkQueuePresentKHR(_graphicsQueue, &presentInfo));

	//increase the number of frames drawn
	_frameNumber++;
}

void VulkanEngine::run()
{
	SDL_Event e;
	bool bQuit = false;

	//main loop
	while (!bQuit)
	{
		//Handle events on queue
		while (SDL_PollEvent(&e) != 0)
		{
			//close the window when user alt-f4s or clicks the X button			
			if (e.type == SDL_QUIT)
			{
				bQuit = true;
			}
			else if (e.type == SDL_KEYDOWN)
			{
			}
		}

		draw();
	}
}

void VulkanEngine::init_vulkan()
{
	vkb::InstanceBuilder builder;

	//make the vulkan instance, with basic debug features
	auto inst_ret = builder.set_app_name("Example Vulkan Application")
		.request_validation_layers(bUseValidationLayers)
		.use_default_debug_messenger()
		.require_api_version(1, 0, 0)
		.build();

	vkb::Instance vkb_inst = inst_ret.value();

	//grab the instance 
	_instance = vkb_inst.instance;
	_debug_messenger = vkb_inst.debug_messenger;
	SDL_Vulkan_CreateSurface(_window, _instance, &_surface);

	//use vkbootstrap to select a gpu. 
	//We want a gpu that can write to the SDL surface and supports vulkan 1.2
	vkb::PhysicalDeviceSelector selector{ vkb_inst };
	
	selector.add_required_extension("VK_KHR_shader_draw_parameters")
			.add_required_extension("VK_KHR_storage_buffer_storage_class")
			.add_required_extension("VK_EXT_descriptor_indexing");
	vkb::PhysicalDevice physicalDevice = selector
		.set_minimum_version(1, 2)
		.set_surface(_surface)
		.select()
		.value();

	//create the final vulkan device

	vkb::DeviceBuilder deviceBuilder{ physicalDevice };

	vkb::Device vkbDevice = deviceBuilder.build().value();

	// Get the VkDevice handle used in the rest of a vulkan application
	_device = vkbDevice.device;
	_chosenGPU = physicalDevice.physical_device;
	VkPhysicalDeviceProperties prop;
	vkGetPhysicalDeviceProperties(_chosenGPU, &prop);
	std::cout << "" << prop.deviceName <<"\n";
	// use vkbootstrap to get a Graphics queue
	_graphicsQueue = vkbDevice.get_queue(vkb::QueueType::graphics).value();

	_graphicsQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::graphics).value();

	_computeQueue = vkbDevice.get_queue(vkb::QueueType::compute).value();

	_computeQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::compute).value();
}

void VulkanEngine::init_swapchain()
{
	vkb::SwapchainBuilder swapchainBuilder{_chosenGPU,_device,_surface };

	vkb::Swapchain vkbSwapchain = swapchainBuilder
		.use_default_format_selection()
		//use vsync present mode
		.set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
		.set_desired_extent(_windowExtent.width, _windowExtent.height)
		.build()
		.value();

	//store swapchain and its related images
	_swapchain = vkbSwapchain.swapchain;
	_swapchainImages = vkbSwapchain.get_images().value();
	_swapchainImageViews = vkbSwapchain.get_image_views().value();

	_swachainImageFormat = vkbSwapchain.image_format;

	//initialize the memory allocator
	VmaAllocatorCreateInfo allocatorInfo = {};
	allocatorInfo.physicalDevice = _chosenGPU;
	allocatorInfo.device = _device;
	allocatorInfo.instance = _instance;
	vmaCreateAllocator(&allocatorInfo, &_allocator);

	_mainDeletionQueue.push_function([&]() {
		vmaDestroyAllocator(_allocator);
		});
	_mainDeletionQueue.push_function([=]() {
		vkDestroySwapchainKHR(_device, _swapchain, nullptr);
	});
}

void VulkanEngine::init_default_renderpass()
{
	VkAttachmentDescription color_attachment = {};
	color_attachment.format = _swachainImageFormat;
	color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
	color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	color_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

	VkAttachmentReference color_attachment_ref = {};
	color_attachment_ref.attachment = 0;
	color_attachment_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	//we are going to create 1 subpass, which is the minimum you can do
	VkSubpassDescription subpass = {};
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.colorAttachmentCount = 1;
	subpass.pColorAttachments = &color_attachment_ref;

	//1 dependency, which is from "outside" into the subpass. And we can read or write color
	VkSubpassDependency dependency = {};
	dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
	dependency.dstSubpass = 0;
	dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	dependency.srcAccessMask = 0;
	dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;


	VkRenderPassCreateInfo render_pass_info = {};
	render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	render_pass_info.attachmentCount = 1;
	render_pass_info.pAttachments = &color_attachment;
	render_pass_info.subpassCount = 1;
	render_pass_info.pSubpasses = &subpass;
	render_pass_info.dependencyCount = 1;
	render_pass_info.pDependencies = &dependency;

	
	VK_CHECK(vkCreateRenderPass(_device, &render_pass_info, nullptr, &_renderPass));

	_mainDeletionQueue.push_function([=]() {
		vkDestroyRenderPass(_device, _renderPass, nullptr);
	});
}

void VulkanEngine::init_framebuffers()
{
	//create the framebuffers for the swapchain images. This will connect the render-pass to the images for rendering
	VkFramebufferCreateInfo fb_info = vkinit::framebuffer_create_info(_renderPass, _windowExtent);

	const size_t swapchain_imagecount = _swapchainImages.size();
	_framebuffers = std::vector<VkFramebuffer>(swapchain_imagecount);

	for (int i = 0; i < swapchain_imagecount; i++) {

		fb_info.pAttachments = &_swapchainImageViews[i];
		VK_CHECK(vkCreateFramebuffer(_device, &fb_info, nullptr, &_framebuffers[i]));

		_mainDeletionQueue.push_function([=]() {
			vkDestroyFramebuffer(_device, _framebuffers[i], nullptr);
			vkDestroyImageView(_device, _swapchainImageViews[i], nullptr);
		});
	}
}

void VulkanEngine::init_commands()
{
	//create a command pool for commands submitted to the graphics queue.
	//we also want the pool to allow for resetting of individual command buffers
	VkCommandPoolCreateInfo commandPoolInfo = vkinit::command_pool_create_info(_graphicsQueueFamily, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);

	VK_CHECK(vkCreateCommandPool(_device, &commandPoolInfo, nullptr, &_commandPool));

	//allocate the default command buffer that we will use for rendering
	VkCommandBufferAllocateInfo cmdAllocInfo = vkinit::command_buffer_allocate_info(_commandPool, 1);

	VK_CHECK(vkAllocateCommandBuffers(_device, &cmdAllocInfo, &_mainCommandBuffer));

	_mainDeletionQueue.push_function([=]() {
		vkDestroyCommandPool(_device, _commandPool, nullptr);
	});

	VkCommandPoolCreateInfo uploadCommandPoolInfo = vkinit::command_pool_create_info(_graphicsQueueFamily);
	//create pool for upload context
	VK_CHECK(vkCreateCommandPool(_device, &uploadCommandPoolInfo, nullptr, &_uploadContext._commandPool));

	_mainDeletionQueue.push_function([=]() {
		vkDestroyCommandPool(_device, _uploadContext._commandPool, nullptr);
	});

	//allocate the default command buffer that we will use for rendering
	VkCommandBufferAllocateInfo cmdAllocInfo2 = vkinit::command_buffer_allocate_info(_uploadContext._commandPool, 1);

	VK_CHECK(vkAllocateCommandBuffers(_device, &cmdAllocInfo2, &_uploadContext._commandBuffer));


	VkCommandPoolCreateInfo uploadComputeCommandPoolInfo = vkinit::command_pool_create_info(_computeQueueFamily);
	//create pool for upload context
	VK_CHECK(vkCreateCommandPool(_device, &uploadComputeCommandPoolInfo, nullptr, &_uploadComputeContext._commandPool));

	_mainDeletionQueue.push_function([=]() {
		vkDestroyCommandPool(_device, _uploadComputeContext._commandPool, nullptr);
		});

	//allocate the default command buffer that we will use for rendering
	VkCommandBufferAllocateInfo cmdCompAllocInfo2 = vkinit::command_buffer_allocate_info(_uploadComputeContext._commandPool, 1);

	VK_CHECK(vkAllocateCommandBuffers(_device, &cmdCompAllocInfo2, &_uploadComputeContext._commandBuffer));
}

void VulkanEngine::init_sync_structures()
{
	//create syncronization structures
	//one fence to control when the gpu has finished rendering the frame,
	//and 2 semaphores to syncronize rendering with swapchain
	//we want the fence to start signalled so we can wait on it on the first frame
	VkFenceCreateInfo fenceCreateInfo = vkinit::fence_create_info(VK_FENCE_CREATE_SIGNALED_BIT);

	VK_CHECK(vkCreateFence(_device, &fenceCreateInfo, nullptr, &_renderFence));

	//enqueue the destruction of the fence
	_mainDeletionQueue.push_function([=]() {
		vkDestroyFence(_device, _renderFence, nullptr);
		});
	VkSemaphoreCreateInfo semaphoreCreateInfo = vkinit::semaphore_create_info();

	VK_CHECK(vkCreateSemaphore(_device, &semaphoreCreateInfo, nullptr, &_presentSemaphore));
	VK_CHECK(vkCreateSemaphore(_device, &semaphoreCreateInfo, nullptr, &_renderSemaphore));
	
	//enqueue the destruction of semaphores
	_mainDeletionQueue.push_function([=]() {
		vkDestroySemaphore(_device, _presentSemaphore, nullptr);
		vkDestroySemaphore(_device, _renderSemaphore, nullptr);
		});


	// Semaphore for compute & graphics sync
	VK_CHECK(vkCreateSemaphore(_device, &semaphoreCreateInfo, nullptr, &_computeSemaphore));
	VkFenceCreateInfo uploadFenceCreateInfo = vkinit::fence_create_info();

	VK_CHECK(vkCreateFence(_device, &uploadFenceCreateInfo, nullptr, &_uploadContext._uploadFence));
	_mainDeletionQueue.push_function([=]() {
		vkDestroyFence(_device, _uploadContext._uploadFence, nullptr);
		});

	VkFenceCreateInfo uploadComputeFenceCreateInfo = vkinit::fence_create_info();

	VK_CHECK(vkCreateFence(_device, &uploadComputeFenceCreateInfo, nullptr, &_uploadComputeContext._uploadFence));
	_mainDeletionQueue.push_function([=]() {
		vkDestroyFence(_device, _uploadComputeContext._uploadFence, nullptr);
		});
}



void VulkanEngine::load_images()
{
	Texture lostEmpire;
	vkutil::create_image(*this, 1280, 720, lostEmpire.image);

	VkImageViewCreateInfo imageinfo = vkinit::imageview_create_info(VK_FORMAT_B8G8R8A8_UNORM, lostEmpire.image._image, VK_IMAGE_ASPECT_COLOR_BIT);
	vkCreateImageView(_device, &imageinfo, nullptr, &lostEmpire.imageView);

	_mainDeletionQueue.push_function([=]() {
		vkDestroyImageView(_device, lostEmpire.imageView, nullptr);
		});

	_loadedTextures["image"] = lostEmpire;

	Material* texturedMat;
	auto it = _materials.find("defaultmesh");
	if (it == _materials.end()) {
		texturedMat = nullptr;
	}
	else {
		texturedMat = &(*it).second;
	}
	VkDescriptorSetAllocateInfo allocInfo = {};
	allocInfo.pNext = nullptr;
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = _descriptorPool;
	allocInfo.descriptorSetCount = 1;
	allocInfo.pSetLayouts = &_TextureSetLayout;

	vkAllocateDescriptorSets(_device, &allocInfo, &texturedMat->textureSet);

	VkSamplerCreateInfo samplerInfo = vkinit::sampler_create_info(VK_FILTER_LINEAR);

	VkSampler blockySampler;
	vkCreateSampler(_device, &samplerInfo, nullptr, &blockySampler);

	_mainDeletionQueue.push_function([=]() {
		vkDestroySampler(_device, blockySampler, nullptr);
		});

	VkDescriptorImageInfo imageBufferInfo;
	imageBufferInfo.sampler = blockySampler;
	imageBufferInfo.imageView = _loadedTextures["image"].imageView;
	imageBufferInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

	VkWriteDescriptorSet texture = vkinit::write_descriptor_image(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, texturedMat->textureSet, &imageBufferInfo, 0);

	vkUpdateDescriptorSets(_device, 1, &texture, 0, nullptr);

	Material mat;
	mat.pipeline = _computePipeline;
	mat.pipelineLayout = _computePipelineLayout;
	_materials["compute"] = mat;

	Material* computedMat;
	it = _materials.find("compute");
	if (it == _materials.end()) {
		computedMat = nullptr;
	}
	else {
		computedMat = &(*it).second;
	}
	VkDescriptorSetAllocateInfo allocCompInfo = {};
	allocCompInfo.pNext = nullptr;
	allocCompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocCompInfo.descriptorPool = _descriptorPool;
	allocCompInfo.descriptorSetCount = 1;
	allocCompInfo.pSetLayouts = &_ComputeTextureSetLayout;

	vkAllocateDescriptorSets(_device, &allocCompInfo, &computedMat->textureSet);

	VkWriteDescriptorSet compTexture = vkinit::write_descriptor_image(VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, computedMat->textureSet, &imageBufferInfo, 1);

	vkUpdateDescriptorSets(_device, 1, &compTexture, 0, nullptr);
}

void VulkanEngine::immediate_submit(std::function<void(VkCommandBuffer cmd)>&& function)
{
	VkCommandBuffer cmd = _uploadContext._commandBuffer;
	//begin the command buffer recording. We will use this command buffer exactly once, so we want to let vulkan know that
	VkCommandBufferBeginInfo cmdBeginInfo = vkinit::command_buffer_begin_info(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

	VK_CHECK(vkBeginCommandBuffer(cmd, &cmdBeginInfo));


	function(cmd);


	VK_CHECK(vkEndCommandBuffer(cmd));

	VkSubmitInfo submit = vkinit::submit_info(&cmd);


	//submit command buffer to the queue and execute it.
	// _renderFence will now block until the graphic commands finish execution
	VK_CHECK(vkQueueSubmit(_graphicsQueue, 1, &submit, _uploadContext._uploadFence));

	vkWaitForFences(_device, 1, &_uploadContext._uploadFence, true, 1000000000000000000);
	vkResetFences(_device, 1, &_uploadContext._uploadFence);

	vkResetCommandPool(_device, _uploadContext._commandPool, 0);
}

void VulkanEngine::init_pipelines()
{
	//compile colored triangle modules
	VkShaderModule redTriangleFragShader;
	if (!load_shader_module("../../shaders/triangle.frag.spv", &redTriangleFragShader))
	{
		std::cout << "Error when building the triangle fragment shader module" << std::endl;
	}
	else {
		std::cout << "Red Triangle fragment shader succesfully loaded" << std::endl;
	}

	VkShaderModule redTriangleVertShader;
	if (!load_shader_module("../../shaders/triangle.vert.spv", &redTriangleVertShader))
	{
		std::cout << "Error when building the triangle vertex shader module" << std::endl;
	}
	else {
		std::cout << "Red Triangle vertex shader succesfully loaded" << std::endl;
	}


	VkShaderModule computeShader;
	if (!load_shader_module("../../shaders/computed.comp.spv", &computeShader))
	{
		std::cout << "Error when building the computeShader  module" << std::endl;
	}
	else {
		std::cout << "computeShader succesfully loaded" << std::endl;
	}

	//build the pipeline layout that controls the inputs/outputs of the shader
	//we are not using descriptor sets or other systems yet, so no need to use anything other than empty default
	VkPipelineLayoutCreateInfo pipeline_layout_info = vkinit::pipeline_layout_create_info();
	VkPipelineLayoutCreateInfo compute_layout_info = vkinit::pipeline_layout_create_info();

	vkCreatePipelineLayout(_device, &compute_layout_info, nullptr, &_computePipelineLayout);

	VkDescriptorSetLayoutBinding textureBind = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);

	VkDescriptorSetLayoutBinding textureBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 1);
	VkDescriptorSetLayoutBinding computeUniform = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);

	VkDescriptorSetLayoutBinding vertBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	VkDescriptorSetLayoutBinding normalBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	VkDescriptorSetLayoutBinding colorBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	VkDescriptorSetLayoutBinding lightsBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	VkDescriptorSetLayoutBinding BVHBindCompute[5];
	BVHBindCompute[0] = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	BVHBindCompute[1] = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	BVHBindCompute[2] = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	BVHBindCompute[3] = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	BVHBindCompute[4] = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	VkDescriptorSetLayoutBinding BVHIdxBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
	VkDescriptorSetLayoutBinding indexBindCompute = vkinit::descriptorset_layout_binding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);

	//create a descriptor pool that will hold 10 uniform buffers
	std::vector<VkDescriptorPoolSize> sizes =
	{
		{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2 },
		{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 20 },
		{ VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 2},
	};

	VkDescriptorPoolCreateInfo pool_info = {};
	pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	pool_info.flags = 0;
	pool_info.maxSets = 20;
	pool_info.poolSizeCount = (uint32_t)sizes.size();
	pool_info.pPoolSizes = sizes.data();

	vkCreateDescriptorPool(_device, &pool_info, nullptr, &_descriptorPool);

	VkDescriptorSetLayoutCreateInfo set0info = {};
	set0info.bindingCount = 1;
	set0info.flags = 0;
	set0info.pNext = nullptr;
	set0info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set0info.pBindings = &textureBind;

	VkDescriptorSetLayoutCreateInfo set0CompInfo = {};
	set0CompInfo.bindingCount = 1;
	set0CompInfo.flags = 0;
	set0CompInfo.pNext = nullptr;
	set0CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set0CompInfo.pBindings = &textureBindCompute;

	VkDescriptorSetLayoutCreateInfo set3CompInfo = {};
	set3CompInfo.bindingCount = 1;
	set3CompInfo.flags = 0;
	set3CompInfo.pNext = nullptr;
	set3CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set3CompInfo.pBindings = &computeUniform;

	VkDescriptorSetLayoutCreateInfo set1CompInfo = {};
	set1CompInfo.bindingCount = 1;
	set1CompInfo.flags = 0;
	set1CompInfo.pNext = nullptr;
	set1CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set1CompInfo.pBindings = &vertBindCompute;

	VkDescriptorSetLayoutCreateInfo set4CompInfo = {};
	set4CompInfo.bindingCount = 1;
	set4CompInfo.flags = 0;
	set4CompInfo.pNext = nullptr;
	set4CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set4CompInfo.pBindings = &normalBindCompute;

	VkDescriptorSetLayoutCreateInfo set5CompInfo = {};
	set5CompInfo.bindingCount = 1;
	set5CompInfo.flags = 0;
	set5CompInfo.pNext = nullptr;
	set5CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set5CompInfo.pBindings = &colorBindCompute;

	VkDescriptorSetLayoutCreateInfo set6CompInfo = {};
	set6CompInfo.bindingCount = 1;
	set6CompInfo.flags = 0;
	set6CompInfo.pNext = nullptr;
	set6CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set6CompInfo.pBindings = &lightsBindCompute;

	VkDescriptorSetLayoutCreateInfo set7CompInfo[5] = {};
	for (int i = 0; i < 5; i++) {
		set7CompInfo[i].bindingCount = 1;
		set7CompInfo[i].flags = 0;
		set7CompInfo[i].pNext = nullptr;
		set7CompInfo[i].sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		set7CompInfo[i].pBindings = &BVHBindCompute[i];

		VK_CHECK(vkCreateDescriptorSetLayout(_device, &set7CompInfo[i], nullptr, &_ComputeBVHDataSetLayout[i]));
	}

	VkDescriptorSetLayoutCreateInfo set8CompInfo = {};
	set8CompInfo.bindingCount = 1;
	set8CompInfo.flags = 0;
	set8CompInfo.pNext = nullptr;
	set8CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set8CompInfo.pBindings = &BVHIdxBindCompute;

	VkDescriptorSetLayoutCreateInfo set2CompInfo = {};
	set2CompInfo.bindingCount = 1;
	set2CompInfo.flags = 0;
	set2CompInfo.pNext = nullptr;
	set2CompInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	set2CompInfo.pBindings = &indexBindCompute;


	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set0info, nullptr, &_TextureSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set0CompInfo, nullptr, &_ComputeTextureSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set3CompInfo, nullptr, &_ComputeSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set1CompInfo, nullptr, &_ComputeVertDataSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set4CompInfo, nullptr, &_ComputeNormDataSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set5CompInfo, nullptr, &_ComputeColorDataSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set6CompInfo, nullptr, &_ComputeLightsDataSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set8CompInfo, nullptr, &_ComputeBVHIdxDataSetLayout));
	VK_CHECK(vkCreateDescriptorSetLayout(_device, &set2CompInfo, nullptr, &_ComputeDataSet2Layout));

	pipeline_layout_info.setLayoutCount = 1;
	pipeline_layout_info.pSetLayouts = &_TextureSetLayout;

	VkDescriptorSetLayout computeSetLayout[] = { _ComputeTextureSetLayout, _ComputeSetLayout, _ComputeVertDataSetLayout, _ComputeDataSet2Layout, _ComputeNormDataSetLayout, _ComputeColorDataSetLayout, _ComputeLightsDataSetLayout, _ComputeBVHDataSetLayout[0], _ComputeBVHIdxDataSetLayout, _ComputeBVHDataSetLayout[1], _ComputeBVHDataSetLayout[2], _ComputeBVHDataSetLayout[3], _ComputeBVHDataSetLayout[4] };

	compute_layout_info.setLayoutCount = 13;
	compute_layout_info.pSetLayouts = computeSetLayout;

	VK_CHECK(vkCreatePipelineLayout(_device, &pipeline_layout_info, nullptr, &_trianglePipelineLayout));
	VK_CHECK(vkCreatePipelineLayout(_device, &compute_layout_info, nullptr, &_computePipelineLayout));

	VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
	pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	VK_CHECK(vkCreatePipelineCache(_device, &pipelineCacheCreateInfo, nullptr, &pipelineCache));
	VkComputePipelineCreateInfo computePipelineCreateInfo{};
	computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	computePipelineCreateInfo.layout = _computePipelineLayout;
	computePipelineCreateInfo.stage = vkinit::pipeline_shader_stage_create_info(VK_SHADER_STAGE_COMPUTE_BIT, computeShader);
	computePipelineCreateInfo.flags = 0;
	VK_CHECK(vkCreateComputePipelines(_device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &_computePipeline));




	//build the stage-create-info for both vertex and fragment stages. This lets the pipeline know the shader modules per stage
	PipelineBuilder pipelineBuilder;

	//vertex input controls how to read vertices from vertex buffers. We arent using it yet
	pipelineBuilder._vertexInputInfo = vkinit::vertex_input_state_create_info();

	//input assembly is the configuration for drawing triangle lists, strips, or individual points.
	//we are just going to draw triangle list
	pipelineBuilder._inputAssembly = vkinit::input_assembly_create_info(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);

	//build viewport and scissor from the swapchain extents
	pipelineBuilder._viewport.x = 0.0f;
	pipelineBuilder._viewport.y = 0.0f;
	pipelineBuilder._viewport.width = (float)_windowExtent.width;
	pipelineBuilder._viewport.height = (float)_windowExtent.height;
	pipelineBuilder._viewport.minDepth = 0.0f;
	pipelineBuilder._viewport.maxDepth = 1.0f;

	pipelineBuilder._scissor.offset = { 0, 0 };
	pipelineBuilder._scissor.extent = _windowExtent;

	//configure the rasterizer to draw filled triangles
	pipelineBuilder._rasterizer = vkinit::rasterization_state_create_info(VK_POLYGON_MODE_FILL);

	//we dont use multisampling, so just run the default one
	pipelineBuilder._multisampling = vkinit::multisampling_state_create_info();

	//a single blend attachment with no blending and writing to RGBA
	pipelineBuilder._colorBlendAttachment = vkinit::color_blend_attachment_state();

	//use the triangle layout we created
	pipelineBuilder._pipelineLayout = _trianglePipelineLayout;

	//clear the shader stages for the builder
	pipelineBuilder._shaderStages.clear();

	//add the other shaders
	pipelineBuilder._shaderStages.push_back(
		vkinit::pipeline_shader_stage_create_info(VK_SHADER_STAGE_VERTEX_BIT, redTriangleVertShader));

	pipelineBuilder._shaderStages.push_back(
		vkinit::pipeline_shader_stage_create_info(VK_SHADER_STAGE_FRAGMENT_BIT, redTriangleFragShader));

	//build the red triangle pipeline
	_redTrianglePipeline = pipelineBuilder.build_pipeline(_device, _renderPass);


	// Separate command pool as queue family for compute may be different than graphics
	VkCommandPoolCreateInfo cmdPoolInfo = {};
	cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	cmdPoolInfo.queueFamilyIndex = _computeQueueFamily;
	cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	vkCreateCommandPool(_device, &cmdPoolInfo, nullptr, &_compCommandPool);

	// Create a command buffer for compute operations
	VkCommandBufferAllocateInfo commandBufferAllocateInfo{};
	commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	commandBufferAllocateInfo.commandPool = _compCommandPool;
	commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	commandBufferAllocateInfo.commandBufferCount = 1;

	VK_CHECK(vkAllocateCommandBuffers(_device, &commandBufferAllocateInfo, &_compCommandBuffer));

	// Build a single command buffer containing the compute dispatch commands

	computeUniformBuffer = create_buffer(sizeof(ComputeBuffer), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
	
	Material mat;
	mat.pipeline = _redTrianglePipeline;
	mat.pipelineLayout = _trianglePipelineLayout;
	_materials["defaultmesh"] = mat;
	mat.pipeline = _computePipeline;
	mat.pipelineLayout = _computePipelineLayout;
	_materials["compute"] = mat;


	vkDestroyShaderModule(_device, redTriangleVertShader, nullptr);
	vkDestroyShaderModule(_device, redTriangleFragShader, nullptr);

	_mainDeletionQueue.push_function([=]() {
		vkDestroyPipeline(_device, _redTrianglePipeline, nullptr);
		vkDestroyPipelineLayout(_device, _trianglePipelineLayout, nullptr);
		vkDestroyDescriptorSetLayout(_device, _TextureSetLayout, nullptr);
	});
}

void VulkanEngine::init_mesh()
{

		/*View = glm::lookAt(glm::vec3(3439.26, -13143.8, 22457.8),
									 glm::vec3(1103.11, -11695.5, 22531),
									 glm::vec3(0, 0, 1));*/
	glm::mat4 View = glm::lookAt(	glm::vec3(0.0, 0, 0),
									glm::vec3(0.0, 1,0),
									glm::vec3(0, 0, 1));
	float factor = 0.2;
	glm::mat4 Projection = glm::ortho(-(float)computeBuffer.resolution.x / factor, (float)computeBuffer.resolution.x/ factor, (float)computeBuffer.resolution.y/ factor, -(float)computeBuffer.resolution.y / factor, 0.1f, 100000.0f);
	
	//glm::mat4 Projection = glm::perspective(glm::radians(45.f), (float)computeBuffer.resolution.x / computeBuffer.resolution.y, 0.1f, 100.f);

	computeBuffer.invTrans = glm::transpose(glm::inverse(Projection * View));


	meshes = new VulkanglTFModel(_device);

	tinygltf::Model glTFInput;
	tinygltf::TinyGLTF gltfContext;
	std::string error, warning, filename = "../../assets/world2.glb";

#if defined(__ANDROID__)
	// On Android all assets are packed with the apk in a compressed form, so we need to open them using the asset manager
	// We let tinygltf handle this, by passing the asset manager of our app
	tinygltf::asset_manager = androidApp->activity->assetManager;
#endif
	bool fileLoaded = gltfContext.LoadBinaryFromFile(&glTFInput, &error, &warning, filename);

	// Pass some Vulkan resources required for setup and rendering to the glTF model loading class
	meshes->_device = _device;
	meshes->copyQueue = _computeQueue;

	std::vector<uint32_t> indexBuffer;
	VulkanglTFModel::Vertex vertices;
	VulkanglTFModel::Light lights;

	if (fileLoaded) {
		//meshes->loadImages(glTFInput);
		meshes->loadMaterials(glTFInput);
		//meshes->loadTextures(glTFInput);
		const tinygltf::Scene& scene = glTFInput.scenes[0];
		for (size_t i = 0; i < scene.nodes.size(); i++) {
			const tinygltf::Node node = glTFInput.nodes[scene.nodes[i]];
			meshes->loadNode(node, glTFInput, nullptr, indexBuffer, vertices, lights);
		}
	}
	else {
		return;
	}
	BVHTree tree(meshes, indexBuffer, vertices);
	
	computeBuffer.numOfLights = lights.pos.size();
	meshes->indices.count = static_cast<uint32_t>(indexBuffer.size());
	computeBuffer.numOfTris = meshes->indices.count / 3;

	printf("loaded scene with %d poly, AA(%f, %f, %f),	BB(%f, %f, %f)\n", computeBuffer.numOfTris, tree.bvhNode.aabbMin[0].x, tree.bvhNode.aabbMin[0].y, tree.bvhNode.aabbMin[0].z, tree.bvhNode.aabbMax[0].x, tree.bvhNode.aabbMax[0].y, tree.bvhNode.aabbMax[0].z);
	size_t vertexBufferSize = vertices.pos.size() * sizeof(float) * 3*10;
	size_t indexBufferSize = indexBuffer.size() * sizeof(uint32_t);
	const size_t bufferSize = vertexBufferSize;
	//allocate vertex buffer
	VkBufferCreateInfo stagingBufferInfo = {};
	stagingBufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	stagingBufferInfo.pNext = nullptr;
	//this is the total size, in bytes, of the buffer we are allocating
	stagingBufferInfo.size = vertexBufferSize;
	//this buffer is going to be used as a Vertex Buffer
	stagingBufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;


	computeVerticesBuffer = create_buffer(vertexBufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
	computeTrianglesBuffer = create_buffer(indexBufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);


	VkDescriptorSetAllocateInfo computeVertSetAlloc = {};
	computeVertSetAlloc.pNext = nullptr;
	computeVertSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeVertSetAlloc.descriptorPool = _descriptorPool;
	computeVertSetAlloc.descriptorSetCount = 1;
	computeVertSetAlloc.pSetLayouts = &_ComputeVertDataSetLayout;

	VK_CHECK(vkAllocateDescriptorSets(_device, &computeVertSetAlloc, &computeVerticesDescriptor));

	VkDescriptorBufferInfo objectBufferInfo;
	objectBufferInfo.buffer = computeVerticesBuffer._buffer;
	objectBufferInfo.offset = 0;
	objectBufferInfo.range = vertexBufferSize;
	VkWriteDescriptorSet objectWrite = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeVerticesDescriptor, &objectBufferInfo, 0);
	//NORMAL	
	VkDescriptorSetAllocateInfo computeNormSetAlloc = {};
	computeNormSetAlloc.pNext = nullptr;
	computeNormSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeNormSetAlloc.descriptorPool = _descriptorPool;
	computeNormSetAlloc.descriptorSetCount = 1;
	computeNormSetAlloc.pSetLayouts = &_ComputeNormDataSetLayout;

	computeNormalBuffer = create_buffer(vertices.normal.size() * sizeof(glm::vec3), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
	VK_CHECK(vkAllocateDescriptorSets(_device, &computeNormSetAlloc, &computeNormalDescriptor));
	VkDescriptorBufferInfo objectNormalInfo;
	objectNormalInfo.buffer = computeNormalBuffer._buffer;
	objectNormalInfo.offset = 0;
	objectNormalInfo.range = vertices.normal.size() * sizeof(glm::vec3);
	VkWriteDescriptorSet NormalWrite = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeNormalDescriptor, &objectNormalInfo, 0);
	//Color
	VkDescriptorSetAllocateInfo computeColorSetAlloc = {};
	computeColorSetAlloc.pNext = nullptr;
	computeColorSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeColorSetAlloc.descriptorPool = _descriptorPool;
	computeColorSetAlloc.descriptorSetCount = 1;
	computeColorSetAlloc.pSetLayouts = &_ComputeColorDataSetLayout;
	computeColorBuffer = create_buffer(vertices.color.size() * sizeof(float) * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
	VK_CHECK(vkAllocateDescriptorSets(_device, &computeColorSetAlloc, &computeColorDescriptor));
	VkDescriptorBufferInfo objectColorInfo;
	objectColorInfo.buffer = computeColorBuffer._buffer;
	objectColorInfo.offset = 0;
	objectColorInfo.range = vertices.color.size() * sizeof(float) * 4;
	VkWriteDescriptorSet ColorWrite = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeColorDescriptor, &objectColorInfo, 0);

	//Lights
	VkDescriptorSetAllocateInfo computeLightsSetAlloc = {};
	computeLightsSetAlloc.pNext = nullptr;
	computeLightsSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeLightsSetAlloc.descriptorPool = _descriptorPool;
	computeLightsSetAlloc.descriptorSetCount = 1;
	computeLightsSetAlloc.pSetLayouts = &_ComputeLightsDataSetLayout;
	computeLightsBuffer = create_buffer(lights.pos.size() * sizeof(float) * 3, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
	VK_CHECK(vkAllocateDescriptorSets(_device, &computeLightsSetAlloc, &computeLightsDescriptor));
	VkDescriptorBufferInfo objectLightsInfo;
	objectLightsInfo.buffer = computeLightsBuffer._buffer;
	objectLightsInfo.offset = 0;
	objectLightsInfo.range = lights.pos.size() * sizeof(float) * 3;
	VkWriteDescriptorSet LightsWrite = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeLightsDescriptor, &objectLightsInfo, 0);

	size_t sizes[] = { tree.bvhNode.aabbMax.size() * sizeof(float) * 3,
						tree.bvhNode.aabbMin.size() * sizeof(float) * 3,
						tree.bvhNode.firstPrim.size() * sizeof(int),
						tree.bvhNode.primCount.size() * sizeof(int),
						tree.bvhNode.leftChild.size() * sizeof(int)
	};
	//BVH
	VkWriteDescriptorSet BVHWrite[5];
	for (int i = 0; i < 5; i++){
		VkDescriptorSetAllocateInfo computeBVHSetAlloc = {};
		computeBVHSetAlloc.pNext = nullptr;
		computeBVHSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		computeBVHSetAlloc.descriptorPool = _descriptorPool;
		computeBVHSetAlloc.descriptorSetCount = 1;
		computeBVHSetAlloc.pSetLayouts = &_ComputeBVHDataSetLayout[i];
		computeBVHBuffer[i] = create_buffer(sizes[i], VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
		VK_CHECK(vkAllocateDescriptorSets(_device, &computeBVHSetAlloc, &computeBVHDescriptor[i]));
		VkDescriptorBufferInfo objectBVHInfo;
		objectBVHInfo.buffer = computeBVHBuffer[i]._buffer;
		objectBVHInfo.offset = 0;
		objectBVHInfo.range = sizes[i];
		BVHWrite[i] = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeBVHDescriptor[i], &objectBVHInfo, 0);
	}
	//BVHIdx
	VkDescriptorSetAllocateInfo computeBVHIdxSetAlloc = {};
	computeBVHIdxSetAlloc.pNext = nullptr;
	computeBVHIdxSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeBVHIdxSetAlloc.descriptorPool = _descriptorPool;
	computeBVHIdxSetAlloc.descriptorSetCount = 1;
	computeBVHIdxSetAlloc.pSetLayouts = &_ComputeBVHIdxDataSetLayout;
	computeBVHIdxBuffer = create_buffer(tree.triIdx.size() * sizeof(int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
	VK_CHECK(vkAllocateDescriptorSets(_device, &computeBVHIdxSetAlloc, &computeBVHIdxDescriptor));
	VkDescriptorBufferInfo objectBVHIdxInfo;
	objectBVHIdxInfo.buffer = computeBVHIdxBuffer._buffer;
	objectBVHIdxInfo.offset = 0;
	objectBVHIdxInfo.range = tree.triIdx.size() * sizeof(int);
	VkWriteDescriptorSet BVHIdxWrite = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeBVHIdxDescriptor, &objectBVHIdxInfo, 0);


	VkDescriptorSetAllocateInfo computeIndexSetAlloc = {};
	computeIndexSetAlloc.pNext = nullptr;
	computeIndexSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeIndexSetAlloc.descriptorPool = _descriptorPool;
	computeIndexSetAlloc.descriptorSetCount = 1;
	computeIndexSetAlloc.pSetLayouts = &_ComputeDataSet2Layout;

	VK_CHECK(vkAllocateDescriptorSets(_device, &computeIndexSetAlloc, &computeTrianglesDescriptor));

	VkDescriptorBufferInfo indexBufferInfo;
	indexBufferInfo.buffer = computeTrianglesBuffer._buffer;
	indexBufferInfo.offset = 0;
	indexBufferInfo.range = indexBufferSize;
	VkWriteDescriptorSet indexWrite = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeTrianglesDescriptor, &indexBufferInfo, 0);



	_mainDeletionQueue.push_function([=]() {

		vmaDestroyBuffer(_allocator, computeVerticesBuffer._buffer, computeVerticesBuffer._allocation);
		vmaDestroyBuffer(_allocator, computeColorBuffer._buffer, computeColorBuffer._allocation);
		vmaDestroyBuffer(_allocator, computeLightsBuffer._buffer, computeLightsBuffer._allocation);
		vmaDestroyBuffer(_allocator, computeBVHIdxBuffer._buffer, computeBVHIdxBuffer._allocation);
		vmaDestroyBuffer(_allocator, computeNormalBuffer._buffer, computeNormalBuffer._allocation);
		});


	//copy vertex data
	void* data;
	vmaMapMemory(_allocator, computeVerticesBuffer._allocation, &data);
	memcpy(data, vertices.pos.data(), vertices.pos.size() * sizeof(float) * 3);
	vmaUnmapMemory(_allocator, computeVerticesBuffer._allocation);

	vmaMapMemory(_allocator, computeNormalBuffer._allocation, &data);
	memcpy(data, vertices.normal.data(), vertices.normal.size() * sizeof(glm::vec3));
	vmaUnmapMemory(_allocator, computeNormalBuffer._allocation);

	vmaMapMemory(_allocator, computeColorBuffer._allocation, &data);
	memcpy(data, vertices.color.data(), vertices.color.size() * sizeof(float) * 4);
	vmaUnmapMemory(_allocator, computeColorBuffer._allocation);

	vmaMapMemory(_allocator, computeLightsBuffer._allocation, &data);
	memcpy(data, lights.pos.data(), lights.pos.size() * sizeof(float) * 3);
	vmaUnmapMemory(_allocator, computeLightsBuffer._allocation);

	VK_CHECK(vmaMapMemory(_allocator, computeBVHIdxBuffer._allocation, &data));
	memcpy(data, tree.triIdx.data(), tree.triIdx.size() * sizeof(int));
	vmaUnmapMemory(_allocator, computeBVHIdxBuffer._allocation);

	//bvh
	void* treeData[] = { tree.bvhNode.aabbMax.data(),
					 tree.bvhNode.aabbMin.data(),
					 tree.bvhNode.firstPrim.data(),
					 tree.bvhNode.primCount.data(),
					 tree.bvhNode.leftChild.data() };
	for (int i = 0; i < 5; i++) {
		VK_CHECK(vmaMapMemory(_allocator, computeBVHBuffer[i]._allocation, &data));
		memcpy(data, treeData[i], sizes[i]);
		vmaUnmapMemory(_allocator, computeBVHBuffer[i]._allocation);
		vkUpdateDescriptorSets(_device, 1, &BVHWrite[i], 0, nullptr);
	}
	//copy index data
	vmaMapMemory(_allocator, computeTrianglesBuffer._allocation, &data);
	memcpy(data, indexBuffer.data(), indexBufferSize);
	vmaUnmapMemory(_allocator, computeTrianglesBuffer._allocation);

	vkUpdateDescriptorSets(_device, 1, &NormalWrite, 0, nullptr);
	vkUpdateDescriptorSets(_device, 1, &ColorWrite, 0, nullptr);
	vkUpdateDescriptorSets(_device, 1, &LightsWrite, 0, nullptr);
	vkUpdateDescriptorSets(_device, 1, &BVHIdxWrite, 0, nullptr);
	vkUpdateDescriptorSets(_device, 1, &indexWrite, 0, nullptr);
	vkUpdateDescriptorSets(_device, 1, &objectWrite, 0, nullptr);

	VkDescriptorSetAllocateInfo computeSetAlloc = {};
	computeSetAlloc.pNext = nullptr;
	computeSetAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	computeSetAlloc.descriptorPool = _descriptorPool;
	computeSetAlloc.descriptorSetCount = 1;
	computeSetAlloc.pSetLayouts = &_ComputeSetLayout;

	VK_CHECK(vkAllocateDescriptorSets(_device, &computeSetAlloc, &computeUniformDescriptor));
	VkDescriptorBufferInfo BufferInfo;
	BufferInfo.buffer = computeUniformBuffer._buffer;
	BufferInfo.offset = 0;
	BufferInfo.range = sizeof(ComputeBuffer);
	void* d;
	vmaMapMemory(_allocator, computeUniformBuffer._allocation, &d); 
	memcpy(d, &computeBuffer, sizeof(ComputeBuffer));
	vmaUnmapMemory(_allocator, computeUniformBuffer._allocation);
	VkWriteDescriptorSet computeUniformDesc = vkinit::write_descriptor_buffer(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, computeUniformDescriptor, &BufferInfo, 0);

	vkUpdateDescriptorSets(_device, 1, &computeUniformDesc, 0, nullptr);

}
AllocatedBuffer VulkanEngine::create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage)
{
	//allocate vertex buffer
	VkBufferCreateInfo bufferInfo = {};
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.pNext = nullptr;
	bufferInfo.size = allocSize;

	bufferInfo.usage = usage;


	//let the VMA library know that this data should be writeable by CPU, but also readable by GPU
	VmaAllocationCreateInfo vmaallocInfo = {};
	vmaallocInfo.usage = memoryUsage;

	AllocatedBuffer newBuffer;

	//allocate the buffer
	VK_CHECK(vmaCreateBuffer(_allocator, &bufferInfo, &vmaallocInfo,
		&newBuffer._buffer,
		&newBuffer._allocation,
		nullptr));

	return newBuffer;
}
bool VulkanEngine::load_shader_module(const char* filePath, VkShaderModule* outShaderModule)
{
	//open the file. With cursor at the end
	std::ifstream file(filePath, std::ios::ate | std::ios::binary);

	if (!file.is_open()) {
		return false;
	}

	//find what the size of the file is by looking up the location of the cursor
	//because the cursor is at the end, it gives the size directly in bytes
	size_t fileSize = (size_t)file.tellg();

	//spirv expects the buffer to be on uint32, so make sure to reserve a int vector big enough for the entire file
	std::vector<uint32_t> buffer(fileSize / sizeof(uint32_t));

	//put file cursor at beggining
	file.seekg(0);

	//load the entire file into the buffer
	file.read((char*)buffer.data(), fileSize);

	//now that the file is loaded into the buffer, we can close it
	file.close();

	//create a new shader module, using the buffer we loaded
	VkShaderModuleCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.pNext = nullptr;

	//codeSize has to be in bytes, so multply the ints in the buffer by size of int to know the real size of the buffer
	createInfo.codeSize = buffer.size() * sizeof(uint32_t);
	createInfo.pCode = buffer.data();

	//check that the creation goes well.
	VkShaderModule shaderModule;
	if (vkCreateShaderModule(_device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
		return false;
	}
	*outShaderModule = shaderModule;
	return true;
}

VkPipeline PipelineBuilder::build_pipeline(VkDevice device, VkRenderPass pass)
{
	//make viewport state from our stored viewport and scissor.
		//at the moment we wont support multiple viewports or scissors
	VkPipelineViewportStateCreateInfo viewportState = {};
	viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportState.pNext = nullptr;

	viewportState.viewportCount = 1;
	viewportState.pViewports = &_viewport;
	viewportState.scissorCount = 1;
	viewportState.pScissors = &_scissor;

	//setup dummy color blending. We arent using transparent objects yet
	//the blending is just "no blend", but we do write to the color attachment
	VkPipelineColorBlendStateCreateInfo colorBlending = {};
	colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	colorBlending.pNext = nullptr;

	colorBlending.logicOpEnable = VK_FALSE;
	colorBlending.logicOp = VK_LOGIC_OP_COPY;
	colorBlending.attachmentCount = 1;
	colorBlending.pAttachments = &_colorBlendAttachment;

	//build the actual pipeline
	//we now use all of the info structs we have been writing into into this one to create the pipeline
	VkGraphicsPipelineCreateInfo pipelineInfo = {};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	pipelineInfo.pNext = nullptr;

	pipelineInfo.stageCount = (uint32_t)_shaderStages.size();
	pipelineInfo.pStages = _shaderStages.data();
	pipelineInfo.pVertexInputState = &_vertexInputInfo;
	pipelineInfo.pInputAssemblyState = &_inputAssembly;
	pipelineInfo.pViewportState = &viewportState;
	pipelineInfo.pRasterizationState = &_rasterizer;
	pipelineInfo.pMultisampleState = &_multisampling;
	pipelineInfo.pColorBlendState = &colorBlending;
	pipelineInfo.layout = _pipelineLayout;
	pipelineInfo.renderPass = pass;
	pipelineInfo.subpass = 0;
	pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;

	//its easy to error out on create graphics pipeline, so we handle it a bit better than the common VK_CHECK case
	VkPipeline newPipeline;
	if (vkCreateGraphicsPipelines(
		device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &newPipeline) != VK_SUCCESS) {
		std::cout << "failed to create pipline\n";
		return VK_NULL_HANDLE; // failed to create graphics pipeline
	}
	else
	{
		return newPipeline;
	}
}
