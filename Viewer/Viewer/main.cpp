// Copyright 2019 Salvatore Spoto

#include "ViewerApp.h"
#include "include/PhysicEngine/Functions.h"

#include "include/PhysicEngine/LPCSolver.h"


/** 
 * Application entry point 
 */
int main() 
{
	/*
	float M[3][3] = { {0, 0, 1}, {0, 0, 1}, {-1, -1, 0} };
	float q[3] = { -2, -1, 3 };
	float w[3];
	float z[3];
	bool hasSolution;
	LPCSolver<3> cx(w, M, q, z, hasSolution);
	float t[3];
	for (int i = 0; i < 3; i++)
	{
		float sum = 0;
		for (int j = 0; j < 3; j++)
		{
			sum += M[i][j] * z[j];
		}
		t[i] = q[i] + sum;
	}
	*/
	
	/*
	float M[5][5] = { {0, 0, -1, 2, 3}, {0, 0, 1, -1, 1}, {1, -1, 0, 0, 0}, {-2, 1, 0, 0, 0}, {-3, -1, 0, 0, 0} };
	float q[5] = { -1, -1, 2, -1, 3 };
	float w[5];
	float z[5];
	bool hasSolution;
	LPCSolver<5> cx(w, M, q, z, hasSolution);
	
	float t[5];
	for (int i = 0; i < 5; i++)
	{
		float sum = 0;
		for (int j = 0; j < 5; j++)
		{
			sum += M[i][j] * z[j];
		}
		t[i] = q[i] + sum;
	}
	
	*/


	float M[3][3] = { {0, 0, 1}, {0, 0, 1}, {-1, -1, 0} };
	float q[3] = { -1, -1, 2 };
	float w[3];
	float z[3];
	bool hasSolution;
	LPCSolver<3> cx(w, M, q, z, hasSolution);
	float t[3];
	for (int i = 0; i < 3; i++)
	{
		float sum = 0;
		for (int j = 0; j < 3; j++)
		{
			sum += M[i][j] * z[j];
		}
		t[i] = q[i] + sum;
	}
	
	
	return 0;

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