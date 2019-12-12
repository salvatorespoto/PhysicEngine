// Copyright 2019 Salvatore Spoto

#include "ViewerApp.h"
#include "include/PhysicEngine/Functions.h"
#include "include/PhysicEngine/QuadraticSolver.h"
#include "include/PhysicEngine/Memory.h"

/** 
 * Application entry point 
 */
int main() 
{
	float** S = Memory::CreateMatrix(2, 2);
	S[0][0] = 1;
	S[0][1] = 0;
	S[1][0] = 0;
	S[1][1] = 1;
	
	float** A = Memory::CreateMatrix(2, 1);
	A[0][0] = 1;
	A[0][1] = 1;

	float* b = Memory::CreateArray(1);
	b[0] = 1;
	
	float* c = Memory::CreateArray(2);
	c[0] = 2;
	c[1] = 4;

	float* x = Memory::CreateArray(2);

	bool hasSolution;

	PhysicEngine::QuadraticSolver(2, S, 1, A, b, c, 0, x, hasSolution);

	try 
	{
		ViewerApp::GetInstance().Run();
	}
	catch (const std::exception& e) 
	{
		BOOST_LOG_TRIVIAL(fatal) << e.what() << ".";
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}