#pragma once

#include <vector>
#include "Memory.h"
#include "LPCSolver.h"

namespace PhysicEngine 
{

	class QuadraticSolver
	{

	public:
		/**
		 * Minimimze the quadratic function x^t S x + cx + k where S is a symmetric nxn matrix , c a know vector a k a know scalar,
		 * subject to the constraing g(x) = (Ax - b, -x) <= 0, i.e. Ax <= b and x > =0
		 */
		QuadraticSolver(int sizeS, float** S, int sizeA, float** A, float* b, float* c, float k, float* x, bool& hasSolution)
		{
			// Convert to a Linear Complementary Problem

			// Build the LCP matrix M. 
			// The M matrix is the size*2 block matrix { 2*S, AT; -A, 0 }
			float** M = PhysicEngine::Memory::CreateMatrix(3, 3);
		
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
					M[j][(sizeS) + i] = A[i][j]; // AT block
					M[(sizeS) + i][j] = -A[i][j]; // -A block
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
			float* q = PhysicEngine::Memory::CreateArray(sizeS + sizeA);
			for (int i = 0; i < sizeS; i++)
			{
				q[i] = -c[i];
			}
			for (int i = 0; i < sizeA; i++)
			{
				q[sizeS + i] = b[i];
			}

			float* w = PhysicEngine::Memory::CreateArray(sizeS + sizeA);
			float* z = PhysicEngine::Memory::CreateArray(sizeS + sizeA);
			LPCSolver(sizeS + sizeA, M, q, w, z, hasSolution);
		
			// Get the solutions of the original quadratic problem
			for (int i = 0; i < sizeS; i++) x[i] = z[i];
		}

	};

}