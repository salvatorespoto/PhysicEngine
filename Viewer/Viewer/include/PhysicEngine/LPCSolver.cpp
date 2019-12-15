#include "LPCSolver.h"


bool CheckLCPSolution(int size, float** M, float* q, float* w, float* z)
{
	float* t = new float[size];

	// q + Mz
	for (int i = 0; i < size; i++)
	{
		float sum = 0;
		for (int j = 0; j < size; j++)
		{
			sum += M[i][j] * z[j];
		}
		t[i] = q[i] + sum;
	}

	// Check w == q + Mz
	for (int i = 0; i < size; i++)
	{
		if (w[i] - t[i] > 0.0001f)
		{
			return false;
		}
	}

	// Check w o z = 0
	for (int i = 0; i < size; i++)
	{
		if (w[i] * z[i] > 0.0001f)
		{
			return false;
		}
	}

	return true;
}