#pragma once



#define TINYGLTF_USE_CPP14
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE
#ifdef VK_USE_PLATFORM_ANDROID_KHR
#define TINYGLTF_ANDROID_LOAD_FROM_ASSETS
#endif
#include "tiny_gltf.h"

#include <vk_types.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#define ENABLE_VALIDATION false


class VulkanglTFModel
{
public:
	VkQueue copyQueue;
	struct Vertex {
		std::vector<glm::vec3> pos;
		std::vector<glm::vec3> normal;
		std::vector<glm::vec2> uv;
		std::vector<glm::vec4> color;
	};
	struct Light {
		std::vector<glm::vec3> pos;
	};
	struct {
		VkBuffer buffer;
		VkDeviceMemory memory;
	} vertices;

	struct {
		int count;
		VkBuffer buffer;
		VkDeviceMemory memory;
	} indices;

	struct Node;

	struct Primitive {
		uint32_t firstIndex;
		uint32_t indexCount;
	};

	struct Mesh {
		std::vector<Primitive> primitives;
	};

	struct Node {
		Node* parent;
		std::vector<Node> children;
		Mesh mesh;
		glm::mat4 matrix;
	};

	struct Material {
		glm::vec4 baseColorFactor = glm::vec4(1.0f);
		uint32_t baseColorTextureIndex;
	};

	std::vector<Material> materials;
	std::vector<Node> nodes;	
	glm::mat4 View;
	glm::mat4 Projection;
	VkDevice _device;
	VulkanglTFModel(VkDevice _device);
	void loadglTFFile(std::string filename);
	~VulkanglTFModel();
	void loadMaterials(tinygltf::Model& input);
	void loadNode(const tinygltf::Node& inputNode, const tinygltf::Model& input, VulkanglTFModel::Node* parent, std::vector<uint32_t>& indexBuffer, VulkanglTFModel::Vertex& vertex, VulkanglTFModel::Light& lights);
	void drawNode(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, VulkanglTFModel::Node node);
	void draw(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
};