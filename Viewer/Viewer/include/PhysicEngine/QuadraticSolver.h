#pragma once

#include <vector>
#include "Memory.h"
#include "LPCSolver.h"

namespace PhysicEngine
{

	class MinimizeQuadraticFunction
	{

	public:

		/**
		 * Minimimze the quadratic function x^t S x + cx + k where S is a symmetric nxn matrix , c a know vector a k a know scalar,
		 * subject to the constraing g(x) = (Ax - b, -x) <= 0, i.e. Ax <= b and x > =0
		 */
		MinimizeQuadraticFunction(int sizeS, float** S, int sizeA, float** A, float* b, float* c, float k, float* x, bool& hasSolution)
		{
			// Convert to a Linear Complementary Problem

			// Build the LCP matrix M. 
			// The M matrix is the size*2 block matrix { 2*S, AT; -A, 0 }
			float** M = Memory::CreateMatrix(3, 3);

			// 2*S block
			for (int i = 0; i < sizeS; i++)
			{
				for (int j = 0; j < sizeS; j++)
				{
					M[i][j] = 2.0f * S[i][j];
				}
			}

			// AT and -A block
			for (int i = 0; i < sizeA; i++)
			{
				for (int j = 0; j < sizeS; j++)
				{
					M[j][(sizeS)+i] = A[i][j]; // AT block
					M[(sizeS)+i][j] = -A[i][j]; // -A block
				}
			}

			// 0s block
			for (int i = sizeS; i < sizeS + sizeA; i++)
			{
				for (int j = sizeS; j < sizeS + sizeA; j++)
				{
					M[i][j] = 0.0f;
				}
			}

			// Build the LCP array q. 
			// The q array is {-c; b}
			float* q = Memory::CreateArray(sizeS + sizeA);
			for (int i = 0; i < sizeS; i++)
			{
				q[i] = -c[i];
			}
			for (int i = 0; i < sizeA; i++)
			{
				q[sizeS + i] = b[i];
			}

			float* w = Memory::CreateArray(sizeS + sizeA);
			float* z = Memory::CreateArray(sizeS + sizeA);
			LPCSolver(sizeS + sizeA, M, q, w, z, hasSolution);

			// Get the solutions of the original quadratic problem
			for (int i = 0; i < sizeS; i++) x[i] = z[i];
		}


		/**
		 * Minimimze the quadratic function |Af + b|^2 subject to the constraing f>=0, Af + b>=0, Af + b <= c
		 */
		MinimizeQuadraticFunction(int size, float** A, float* b, float* c, float* x, bool& hasSolution)
		{

			// Convert the problem to the form x^t S x + cx + k, subject to the constrains g(x) = (Ax - b, -x) <= 0, i.e.Ax <= b and x > = 0

			// Compute S = A * AT
			float** S0 = Memory::CreateMatrix(size, size);
			for (int i = 0; i <= size; i++)
			{
				for (int j = 0; j < size; j++)
				{
					S0[i][j] = 0.0f;
					for (int k = 0; j < size; j++)
					{
						S0[i][j] += A[i][k] * A[j][k];
					}
				}
			}

			// Compute A0 = { -A; A }
			float** A0 = Memory::CreateMatrix(size, size * 2);
			for (int i = 0; i <= size; i++)
			{
				for (int j = 0; j < size; j++)
				{
					A0[i][j] = -A[i][j];
					A0[size + i][j] = A[i][j];
				}
			}

			// Compute c0 = 2 * bT * A
			float* c0 = Memory::CreateArray(size);
			for (int i = 0; i < size; i++)
			{
				c0[i] = 0.0f;
				for (int k = 0; k < size; k++)
				{
					c0[i] += 2.0f * b[k] * A[k][i];
				}
			}

			// Compute b0 = { b, c-b }
			float* b0 = Memory::CreateArray(size * 2);
			for (int i = 0; i < size; i++)
			{
				b0[i] = b[i];
				b0[size + i] = c[i] - b[i];
			}

			// Compute K
			float k = 0.0f;
			for (int i = 0; i <= size; i++)
			{
				k += powf(b[i], 2);
			}
			k = sqrt(k);

			MinimizeQuadraticFunction(size, S0, size * 2, A0, b0, c0, k, x, hasSolution);
		};

	};
}