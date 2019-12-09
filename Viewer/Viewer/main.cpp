// Copyright 2019 Salvatore Spoto

#include "ViewerApp.h"
#include "include/PhysicEngine/Functions.h"
#include "include/PhysicEngine/LPCSolver.h"
#include "include/PhysicEngine/Memory.h"

/** 
 * Application entry point 
 */
int main() 
{

	/*
	float** M = Memory::CreateMatrix(3, 3);
	M[0][0] = 0;
	M[0][1] = 0;
	M[0][2] = 1;
	M[1][0] = 0;
	M[1][1] = 0;
	M[1][2] = 1;
	M[2][0] =-1;
	M[2][1] =-1;
	M[2][2] = 0;

	float q[3] = { -1, -1, 2 };
	float w[3];
	float z[3];
	bool hasSolution;
	LPCSolver cx(3, M, q, w, z, hasSolution);
	float t[3];
	for (int i = 0; i < 3; i++)
	{
		float sum = 0;
		for (int j = 0; j < 3; j++)
		{
			sum += M.p[i][j] * z[j];
		}
		t[i] = q[i] + sum;
	}
	*/
	
	/*
	float M[5][5] = { {0, 0, -1, 2, 3}, {0, 0, 1, -1, 1}, {1, -1, 0, 0, 0}, {-2, 1, 0, 0, 0}, {-3, -1, 0, 0, 0} };
	
	float** M = Memory::CreateMatrix(5, 5);
	M[0][0] = 0;
	M[0][1] = 0;
	M[0][2] = -1;
	M[0][3] = 2;
	M[0][4] = 3;
	
	M[1][0] = 0;
	M[1][1] = 0;
	M[1][2] = 1;
	M[1][3] = -1;
	M[1][4] = 1;

	M[2][0] = 1;
	M[2][1] = -1;
	M[2][2] = 0;
	M[2][3] = 0;
	M[2][4] = 0;

	M[3][0] = -2;
	M[3][1] = 1;
	M[3][2] = 0;
	M[3][3] = 0;
	M[3][4] = 0;

	M[4][0] = -3;
	M[4][1] = -1;
	M[4][2] = 0;
	M[4][3] = 0;
	M[4][4] = 0;
	
	float q[5] = { -1, -1, 2, -1, 3 };
	float w[5];
	float z[5];
	bool hasSolution;
	LPCSolver cx(5, M, q, w, z, hasSolution);
	
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
	

	float** M = Memory::CreateMatrix(3, 3);
	float M[3][3] = { {0, 0, 1}, {0, 0, 1}, {-1, -1, 0} };
	M[0][0] = 0;
	M[0][1] = 0;
	M[0][2] = 1;
	M[1][0] = 0;
	M[1][1] = 0;
	M[1][2] = 1;
	M[2][0] = -1;
	M[2][1] = -1;
	M[2][2] = 0;

	float q[3] = { -2, -1, 3 };
	float w[3];
	float z[3];
	bool hasSolution;
	LPCSolver cx(3, M, q, w, z, hasSolution);
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
*/
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