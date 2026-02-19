//we will be using glsl version 4.5 syntax
#version 450

layout (location = 1) out vec2 texCoord;

void main() 
{
	//const array of positions for the triangle
	const vec3 positions[6] = vec3[6](
		vec3(1.f,1.f, 0.0f),
		vec3(-1.f,1.f, 0.0f),
		vec3(-1.f,-1.f, 0.0f),
		vec3(1.f,1.f, 0.0f),
		vec3(-1.f,-1.f, 0.0f),
		vec3(1.f,-1.f, 0.0f)
	);
	const vec2 uv[6] = vec2[6](
		vec2(1.f,1.f),
		vec2(0.f,1.f),
		vec2(0.f,0.f),
		vec2(1.f,1.f),
		vec2(0.f,0.0f),
		vec2(1.f, 0.0f)
	);

	//output the position of each vertex
	gl_Position = vec4(positions[gl_VertexIndex], 1.0f);
	texCoord = uv[gl_VertexIndex];
}