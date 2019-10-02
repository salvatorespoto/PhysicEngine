#version 450 core
#extension GL_ARB_separate_shader_objects : enable

in vec3 fragmentPosition;
in vec3 normal;

out vec4 fragmentColor;

uniform float ambientStrength;
uniform vec3 lightPosition;
uniform vec3 lightColor;
uniform vec3 meshColor;


void main() 
{
	// Ambient contribute
	vec3 ambient = ambientStrength * lightColor;
    
	// Diffuse contributr
	vec3 norm = normalize(normal);
	vec3 lightDirection = normalize(lightPosition - fragmentPosition); 
	
	float diff = max(dot(norm, lightDirection), 0.0);
	vec3 diffuse = diff * lightColor;
	
	vec3 result = (ambient + diffuse) * meshColor;
    fragmentColor = vec4(result, 1.0);
}