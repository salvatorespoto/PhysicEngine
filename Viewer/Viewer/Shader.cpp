// Copyright 2019 Salvatore Spoto

#include "Shader.h"


Shader::Shader(const boost::filesystem::path& shaderPath)
{	
	const std::string& shaderFileName = shaderPath.filename().string();

	// Set exceptions condition mask
	std::ifstream shaderFile;
	shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	
	// Open the shader file source and read it 
	std::string shaderCode;
	try
	{
		shaderFile.open(shaderPath.string());
		std::stringstream shaderStream;
		shaderStream << shaderFile.rdbuf();
		shaderFile.close();
		shaderCode = shaderStream.str();
	} 
	catch (std::ifstream::failure e)
	{
		throw std::runtime_error("Cannot read shader source file " + shaderPath.filename().string());
	}
	
	// Get the shader type from the extension
	if (std::string_view::npos != shaderFileName.find(VERTEX_SHADER_EXTENSION)) ShaderType = GL_VERTEX_SHADER;
	else if (std::string_view::npos != shaderFileName.find(FRAGMENT_SHADER_EXTENSION)) ShaderType = GL_FRAGMENT_SHADER;
	else throw std::runtime_error("Shader " + std::string(shaderFileName) + " has an invalid extension ");

	// Compile shader
	Id = glCreateShader(ShaderType);
	const char* cShaderCode = shaderCode.c_str();
	glShaderSource(Id, 1, &cShaderCode, NULL);
	glCompileShader(Id);

	// Check for compilation success
	int success;
	char infoLog[1024];
	glGetShaderiv(Id, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(Id, 1024, NULL, infoLog);
		throw std::runtime_error("Error compiling shader " + std::string(shaderFileName) + ":" + infoLog);
	}
}
