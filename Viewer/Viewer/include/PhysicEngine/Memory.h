#pragma once

namespace PhysicEngine
{

	class Memory
	{

	public:

		/**
		 * Allocate a nxm matrix, matrix is in row-major order
		 *
		 */
		static float** CreateMatrix(int n, int m)
		{
			float** matrix;
			matrix = new float*[n];
			for (int i = 0; i < n; i++)
			{
				matrix[i] = new float[m];
			}
			return matrix;
		}


		/**
		 * Allocate a vector of n elements
		 *
		 */
		static float* CreateArray(int n)
		{
			return new float[n];
		}

	};

}