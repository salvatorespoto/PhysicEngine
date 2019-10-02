#version 450 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
//layout(location = 3) in vec2 inTexCoord;

out vec3 fragmentPosition;
flat out vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;


void main() {

    gl_Position = projection * view * model * vec4(inPosition.x, inPosition.y, inPosition.z, 1.0f);
	fragmentPosition = vec3(model *  vec4(inPosition.x, inPosition.y, inPosition.z, 1.0f));
	normal = inNormal; 
}