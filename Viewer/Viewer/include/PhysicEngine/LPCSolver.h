#pragma once

#include <string>;
#include <unordered_set>;

/**
 * An entry in the dictionary, i.e. an equation
 */
template <int N>
class TEquation
{

public:
	
	char v;			// The basic variable of this equation, '0' means that nothing has been assigned to this equation yet
	int i;			// The index of the basic variable 	
	float e[N+1];	// The PERTURBATION added to the coefficient to avoid DEGENERANCY, e[0] is the constant term of the equation
	float w[N+1];	// The coefficients of the w variables in the equation, note that w[0] is never used
	float z[N+1];	// The coefficients of the z variables in the equation
	

	/**
	 * Constructor
	 */
	TEquation<N> () 
	{
		i = 0;
		for(int i=0; i<=N; i++) 
		{
			e[i] = 0;
			w[i] = 0;
			z[i] = 0;
		}
	}


	/**
	 * Multiply the equation by a value
	 */
	void MultiplyBy(float x)
	{
		for (int i=0; i<=N; i++)
		{
			e[i] *= x;
			w[i] *= x;
			z[i] *= x;
		}
	}


	/**
	 * Compute the constant value of this equation (i.e. the sum of all e[] elements)
	 */
	float E() 
	{
		float s = 0;
		for (int i = 0; i <= N; i++) s += e[i];
		return s;
	}


	/**
	 * Return the ratio between the constant term E and the variable v[i]
	 */
	float GetEViRatio(const char _v, unsigned int _i)
	{
		if (_v == 'w') return E() / w[_i];
		else return E() / z[_i];
	}


	/**
	 * Solve the equation for the non basic variable _v
	 */
	void SolveFor(const char _v, unsigned int _i)
	{
		switch(_v)
		{
			case 'w':
				// Add the basic variable to non basic
				if (v == 'w') w[i] = -1;
				else z[i] = -1;
				// Handle the coefficient of the entering the Dictionary non basic variable
				MultiplyBy(-1 / w[_i]);
				w[_i] = 0;
				break;

			case 'z':
				if (v == 'w') w[i] = -1;
				else z[i] = -1;
				MultiplyBy(-1 / z[_i]);
				z[_i] = 0;
				break;
		}

		// Update the basic variable
		v = _v;
		i = _i;
	}

	
	/**
	 * Replace a basic variable an equation where it is a non basic variable
	 */
	void ReplaceVariable(const TEquation<N> eq)
	{
		float c = 0;
		if (eq.v == 'w')
		{
			if (w[eq.i] == 0) return;
			c = w[eq.i];
		}

		if (eq.v == 'z')
		{
			if (z[eq.i] == 0) return;
			c = z[eq.i];
		}

		for(int k=0; k<=N; k++) 
		{
			e[k] += c*eq.e[k];
			w[k] += c*eq.w[k];
			z[k] += c*eq.z[k];
		}

		if (eq.v == 'w') w[eq.i] = 0;
		else z[eq.i] = 0;
	}
	
};



/**
 * Implements the Lemke algorithm to solve Linear Complementary Problem w = q + Mz
 */
template <int N>
class LPCSolver 
{
public:

	typedef TEquation<N> Equation;

	/**
	 * Matrix M
	 */
	float (&M)[N][N];

	/**
	 * Vector w
	 */
	float(&w)[N];

	/**
	 * Vector q
	 */
	float(&q)[N];

	/**
	 * Vector z
	 */
	float(&z)[N];

	/**
	 * Vector z
	 */
	float z0[N];

	/**
	 *	The dictionary
	 */
	Equation Dictionary[N];

	/**
	 * The PERTURBATION introduced to avoid DEGENERANCY
	 */
	const float epsilon = 0.2f;

	/**
	 * The precomputed powers epsilon^k for k=0..N
	 */
	float(epsPow)[N];
	
	/**
	 * Constructor
	 */
	LPCSolver(float(&w)[N], float(&M)[N][N], float(&q)[N], float(&z)[N], bool& hasSolution) : M(M), w(w), q(q), z(z)
	{
		
		if (HasTrivialSolution())
		{
			hasSolution = true;
			return;
		}

		ComputePerturbations();

		// Execute Lemke algorithm
		InitDictionary();

		// Lemke algorithm FASE 1: 
		// z0 is a non basic variable, select the equation from the dictionary 
		// that has the smallest C / z[0] ratio and solve it for z[0]. 
		// After FASE 1 we get a FEASIBLE DICTIONARY (all C > 0). The dictionary
		// is also BALANCED becouse for each i, either w[i] or z[i] are non basic.
		// The problem is not solved yet because z[0] is basic.
		Equation* eq = SelectEquation('z', 0);
		char exv = eq->v;
		unsigned int exi = eq->i;
		eq->SolveFor('z', 0);
		
		// Update other equation with the new z[0] value
		ReplaceVariableInDictionary(eq);
		

		// Lemke algorithm FASE 2: 
		//  1. Select the equation for the non basic complementary variable 
		//     of the basic variable that just exited from the Dictionary
		//  2. Solve for the non basic variabile
		//  3. If z[0] is back to non basic the dictionary is terminal and the problem is solved.
		//     The solution is given from the vector w and z where the non basic variable are 0 and 
		//     the basic have the value of the coefficients of the equation associated at the end.
		//  4. Else repeat from step 1

		while(IsZ0BasicVariable()) 
		{
			// Select the equation for the equcomplementary variable
			char cexv = (exv == 'w') ? 'z' : 'w';
			unsigned int cexi = exi;
			eq = SelectEquation(cexv, cexi);
			if(eq == NULL)
			{
				hasSolution = false;
				return;
			}
			
			// Save exiting basic variable
			exv = eq->v;
			exi = eq->i;

			// Solve for the complementary variable and update Dictionary
			eq->SolveFor(cexv, cexi);
			ReplaceVariableInDictionary(eq);
		}

		// Build solutions
		for(int i=0; i<N; i++) 
		{
			w[i] = 0;
			z[i] = 0;
		}
		
		for(Equation eq : Dictionary) 
		{
			if (eq.v == 'w') w[eq.i-1] = eq.e[0];
			if (eq.v == 'z') z[eq.i-1] = eq.e[0];
		}

		hasSolution = true;
		return;
	}


	/**
	 * Precompute the powers epsilon^k
	 *
	 * @return true if the problem has trivial solution
	 */
	void ComputePerturbations()
	{
		for(int i=0; i<=N; i++) 
		{
			epsPow[i] = pow(epsilon, i);
		}
	}

	/**
	 * Check if the LCP problem has trivial solution. 
	 * If found the solution w = q and z = 0 is stored in w and z,
	 *
	 * @return true if the problem has trivial solution
	 */
	bool HasTrivialSolution()
	{
		for (int i = 0; i < N; i++) 
		{
			if (q[i] < 0) return false;
		}

		// Has trivial solution w = q and z = 0
		for (int i = 0; i < N; i++)
		{
			w[i] = q[i];
			z[i] = 0;
		}

		return true;
	}

	
	/** 
	 * Init the dictionary inserting the N equations w_i = M_ji * z_j + z0
	 */
	void InitDictionary() 
	{
		for(int i=1; i<=N; i++) 
		{
			Equation eq;

			// At the beginning, all the BASIC VARIABLES are w_i
			eq.v = 'w';
			eq.i = i;
			eq.w[i] = 0; 
			
			// Set up constant terms and perturbations
			eq.e[0] = q[i - 1];
			eq.e[i] = epsPow[i];
			
			// At the beginning, the z[0] auxiliary variable is added to all equations 
			eq.z[0] = 1;

			// Add the coefficients for the NON BASIC VARIABLES z_j, j > 1 variables
			for (int j = 1; j <= N; j++) 
			{
				// Const coefficients
				eq.z[j] = M[i-1][j-1] * 1;
			}

			// Add the equation to the dictionary
			Dictionary[i-1] = eq;
		}
	}


	/**
	 * Select the equation that has the smaller c / v[i] ratio, where 
	 * 
	 * @param v a char that holds the variable name, 'w' or 'z'
	 * @param i the index of the variable 
	 * @return the selected Equation
	 */
	Equation* SelectEquation(char v, int i)
	{
		float ratio = std::numeric_limits<float>::max();
		Equation* result = NULL;
		
		// Special case for z[0], the equation with the less c / z[0] ratio
		if(v == 'z' && i == 0) 
		{
			for (Equation& eq : Dictionary)
			{
				float cv = eq.GetEViRatio(v, i);
				if (cv < ratio)
				{
					result = &eq;
					ratio = cv;
				}
			}
		}

		// For other variables, select the equation that has negative v[i] coefficient
		// and the smallest C / (-v[i]) ratio
		else
		{
			for (Equation& eq : Dictionary)
			{
				if (v == 'w') if (eq.w[i] >= 0) continue;
				if (v == 'z') if (eq.z[i] >= 0) continue;

				float cv = -eq.GetEViRatio(v, i);
				if (cv < ratio)
				{
					result = &eq;
					ratio = cv;
				}
			}
		}

		return result;
	}


	/**
	 * Replace all the occurences of non basic variables in the Dictionary with the equation eq
	 */
	void ReplaceVariableInDictionary(Equation* eq) 
	{
		for (Equation& neq : Dictionary)
		{
			neq.ReplaceVariable(*eq);
		}
	}
	

	/** 
	 * Check if z[0] is among the basic variables
	 *
	 * @return true if z[0] is a basic variable
	 */
	bool IsZ0BasicVariable()
	{
		for (Equation eq : Dictionary)
		{
			if (eq.v == 'z' && eq.i == 0) return true;
		}
		return false;
	}
};