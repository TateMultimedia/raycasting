//glsl version 4.5
#version 450

//output write
layout (location = 0) out vec4 outFragColor;
layout (location = 1) in vec2 texCoord;
layout(set = 0, binding = 0) uniform sampler2D tex;

void main() 
{
	vec3 color = texture(tex,texCoord).xyz;
	outFragColor = vec4(color,1.0f);
}