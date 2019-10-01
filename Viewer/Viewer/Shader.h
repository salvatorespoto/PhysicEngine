// Copyright 2019 Salvatore Spoto

#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <boost/filesystem/path.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "include/glad/glad.h"



/** This class load and compile a vertex or fragment shader */
class Shader
{

public:
	 
	/** Source file vertex shader extension */
	static constexpr std::string_view VERTEX_SHADER_EXTENSION = ".vs";
	
	/** Source file fragment shader extension */
	static constexpr std::string_view FRAGMENT_SHADER_EXTENSION = ".fs";

	/** Shader type: GL_VERTEX_SHADER or GL_FRAGMENT_SHADER */
	GLenum ShaderType;

	/** Shader id */
	GLuint Id = 0;
	
	/**
	 * Compile a the shader from source file
	 * 
	 * @param shaderPath the shader file source path
	 */
	Shader(const boost::filesystem::path& shaderPath);
};

#endif
