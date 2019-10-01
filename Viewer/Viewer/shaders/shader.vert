#version 450 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inTexCoord;

out vec2 outTexCoord;

uniform mat4 transform;

void main() {
    gl_Position = vec4(inPosition.x, inPosition.y, inPosition.z, 1.0);
	outTexCoord = vec2(inTexCoord.x, inTexCoord.y);
}