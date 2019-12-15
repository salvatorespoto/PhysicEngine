#include "collision.h"




void PhysicEngine::DoCollisionResponse(float t, float dt, const std::vector<Contact>& contacts, const std::vector<RigidBody*>& bodies)
{
	int N = contacts.size();
	float** M = Memory::CreateMatrix(N, N);

	ComputeLCPMatrix(contacts, M, N);

	// Compute the contact points relative velocities just before the contact
	std::vector<float> preContactVelocities = ComputePreContactVelocities(contacts);
	
	
	std::vector<float> postContactVelocities = std::vector<float>(N);


	float* x = Memory::CreateArray(N);
	float* impulsesMagnitude = Memory::CreateArray(N);

	bool hasSolution;
	Minimize(N, M, preContactVelocities, postContactVelocities, impulsesMagnitude);
	DoImpulse(contacts, impulsesMagnitude);
	
	float* b = Memory::CreateArray(N);
	ComputingRestingContactVector(contacts, b);

	float* w = Memory::CreateArray(N);
	float* z = Memory::CreateArray(N);

	LPCSolver(N, M, b, w, z, hasSolution);

	if (CheckLCPSolution(N, M, b, w, z)) 
	{
		int pippo = 1;
	}


	DoMotion(t, dt, contacts, z, bodies);
	
}

void PhysicEngine::ProcessCollidingContact(RigidBody& rbA, RigidBody& rbB, const glm::vec3& collidingPoint, const glm::vec3& normal)
{
	// 1. Compute the inpulse force

	// r1, r2: colliding point coordinates relatives to the objects center of mass
	glm::vec3 rA = collidingPoint - rbA.Position;
	glm::vec3 rB = collidingPoint - rbB.Position;

	glm::vec3 kA = glm::cross(rA, normal);
	glm::vec3 kB = glm::cross(rB, normal);
	glm::vec3 uA = rbA.InvertedIntertiaTensor * kA;
	glm::vec3 uB = rbB.InvertedIntertiaTensor * kB;

	// The formula formula to compute the resulting impulsive force is quite complex,
	// so it is breaked down into numerator and denumerator
	float numerator = -(1 + RESTITUTION_COEFFICIENT) * (
		glm::dot(normal, rbA.LinearVelocity - rbB.LinearVelocity) +
		glm::dot(rbA.AngularVelocity, kA) -
		glm::dot(rbB.AngularVelocity, kB));
	float denominator = rbA.InvertedMass + rbB.InvertedMass + glm::dot(kA, uA) + glm::dot(kB, uB);
	float impulseModule = numerator / denominator;

	glm::vec3 impulse = impulseModule * normal;

	// 2. Apply inpulse force to bodies to change linear and angular momentum
	rbA.LinearMomentum += impulse;
	rbB.LinearMomentum -= impulse;
	rbA.AngularMomentum += impulseModule * kA;
	rbB.AngularMomentum -= impulseModule * kB;

	// Compute derived quantities: linear velocity and angular velocity
	rbA.LinearVelocity = rbA.LinearMomentum * rbA.InvertedMass;
	rbB.LinearVelocity = rbB.LinearMomentum * rbB.InvertedMass;
	rbA.AngularVelocity = impulse * uA;
	rbB.AngularVelocity = impulse * uB;
}



std::vector<float> PhysicEngine::ComputePreContactVelocities(const std::vector<Contact>& contacts)
{

	std::vector<float> velocities(contacts.size());	
	int i = 0;
	for (Contact c : contacts) 
	{
		RigidBody A = c.rb0;
		RigidBody B = c.rb1;

		glm::vec3 rA = c.point - A.Position;	// Relative position of colliding point of the object A relative to its center of mass
		glm::vec3 rB = c.point - B.Position;	// Relative position of colliding point of the object A relative to its center of mass

		glm::vec3 vA = A.LinearVelocity + glm::cross(A.AngularVelocity, rA);	// vA relative velocity of the contact point of the object A
		glm::vec3 vB = B.LinearVelocity + glm::cross(B.AngularVelocity, rB);	// vB relative velocity of the contact point of the object B

		// Compute the relative velocity in the contact normal direction between the two objcets contacts point 
		velocities[i] = glm::dot(c.normal, vA - vB);
		i++;
	}

	return velocities;
}


/**
	*
	*
	* @param
	* @param	normal			the collding face normal
	*/
void PhysicEngine::ComputeLCPMatrix(std::vector<Contact> contacts, float** M, int N)
{
	for (int i = 0; i < contacts.size(); i++)
	{
		Contact ci = contacts[i];
		glm::vec3 r0Ni = glm::cross(ci.point - ci.rb0.Position, ci.normal);
		glm::vec3 r1Ni = glm::cross(ci.point - ci.rb1.Position, ci.normal);

		for (int j = 0; j < contacts.size(); j++)
		{
			Contact cj = contacts[j];
			glm::vec3 r0Nj = glm::cross(cj.point - cj.rb0.Position, cj.normal);
			glm::vec3 r1Nj = glm::cross(cj.point - cj.rb1.Position, cj.normal);

			M[i][j] = 0;
			if (&ci.rb0 == &cj.rb0)
			{
				M[i][j] += ci.rb0.InvertedMass * glm::dot(ci.normal, cj.normal);
				M[i][j] += glm::dot(r0Ni, ci.rb0.InvertedIntertiaTensor * r0Nj);
			}
			else if (&ci.rb0 == &cj.rb1)
			{
				M[i][j] -= ci.rb0.InvertedMass * glm::dot(ci.normal, cj.normal);
				M[i][j] -= glm::dot(r0Ni, ci.rb0.InvertedIntertiaTensor * r0Nj);
			}
			if (&ci.rb1 == &cj.rb0)
			{
				M[i][j] -= ci.rb1.InvertedMass * glm::dot(ci.normal, cj.normal);
				M[i][j] -= glm::dot(r1Ni, ci.rb1.InvertedIntertiaTensor * r1Nj);
			}
			else if (&ci.rb1 == &cj.rb1)
			{
				M[i][j] += ci.rb1.InvertedMass * glm::dot(ci.normal, cj.normal);
				M[i][j] += glm::dot(r1Ni, ci.rb1.InvertedIntertiaTensor * r1Nj);
			}
		}
	}
}


void PhysicEngine::DoMotion(float t, float dt, std::vector<Contact> contacts, float *restingMagnitute, std::vector<RigidBody*> bodies)
{
	for (int i = 0; i < contacts.size(); i++)
	{
		Contact c = contacts[i];
		glm::vec3 resting = restingMagnitute[i] * c.normal;
		c.rb0.AddInternalForce(resting);
		c.rb0.AddInternalTorque(glm::cross(c.point - c.rb0.Position, resting));
		c.rb1.AddInternalForce(-resting);
		c.rb1.AddInternalTorque(-glm::cross(c.point - c.rb1.Position, resting));
	}
	/*
	for (RigidBody *rb : bodies) 
	{
		(*rb).UpdateState(t, dt);
	}*/
}

void PhysicEngine::Minimize(int N, float** M, const std::vector<float>& preContactVelocities, std::vector<float>& outPostContactVelocities, float* impulsesMagnitude)
{
	// Build quadratic problem coefficients
	float* b = Memory::CreateArray(N);
	float* c = Memory::CreateArray(N);

	for (int i = 0; i < N; i++)
	{
		if (preContactVelocities[i] < 0) b[i] = 2.0f * preContactVelocities[i];
		else b[i] = 0;

		c[i] = fabsf(preContactVelocities[i]);
	}

	bool hasSolution;
	MinimizeQuadraticFunction(N, M, b, c, impulsesMagnitude, hasSolution);
	if (!hasSolution) 
	{
		int pippo = 1;
	}
	// Compute postImpulseVel
	for (int i = 0; i < N; i++)
	{
		outPostContactVelocities[i] = 0.0f;
		for (int j = 0; j < N; j++)
		{
			outPostContactVelocities[i] += impulsesMagnitude[j] * M[j][i];
		}
		outPostContactVelocities[i] += b[i];
	}
}

void PhysicEngine::DoImpulse(std::vector<Contact> contacts, float *impulsesMagnitude)
{
	// Compute the post impulse linear and angular velocities
	for(int i = 0; i<contacts.size(); i++) 
	{
		Contact c = contacts[i];
		glm::vec3 impulse = impulsesMagnitude[i] * c.normal;
		
		glm::vec3 r0 = c.point - c.rb0.Position;
		glm::vec3 r1 = c.point - c.rb1.Position;
				
		c.rb0.LinearMomentum += impulse;
		c.rb1.LinearMomentum -= impulse;
		c.rb0.AngularMomentum += glm::cross(r0, c.normal);
		c.rb1.AngularMomentum -= glm::cross(r1, c.normal);
			
		c.rb0.LinearVelocity = c.rb0.LinearMomentum * c.rb0.InvertedMass;
		c.rb1.LinearVelocity = c.rb1.LinearMomentum * c.rb1.InvertedMass;
		c.rb0.AngularVelocity = impulse * c.rb0.InvertedIntertiaTensor;
		c.rb1.AngularVelocity = impulse * c.rb1.InvertedIntertiaTensor;
	}
}


void PhysicEngine::ComputingRestingContactVector(std::vector<Contact> contacts, float *b)
{
	for (int i = 0; i < contacts.size(); i++)
	{
		Contact c = contacts[i];
		RigidBody A = c.rb0;
		RigidBody B = c.rb1;

		// Body A terms
		glm::vec3 rAi = c.point - A.Position;
		glm::vec3 wAxrAi = glm::cross(A.AngularMomentum, rAi);
		glm::vec3 At1 = A.InvertedMass * A.ExternalForce;
		glm::vec3 At2 = glm::cross(A.InvertedIntertiaTensor * (A.ExternalTorque + glm::cross(A.AngularMomentum, A.AngularVelocity)), rAi);
		glm::vec3 At3 = glm::cross(A.AngularVelocity, wAxrAi);
		glm::vec3 At4 = A.LinearVelocity + wAxrAi;

		// Body B terms
		glm::vec3 rBi = c.point - B.Position;
		glm::vec3 wBxrBi = glm::cross(B.AngularMomentum, rBi);
		glm::vec3 Bt1 = B.InvertedMass * B.ExternalForce;
		glm::vec3 Bt2 = glm::cross(B.InvertedIntertiaTensor * (B.ExternalTorque + glm::cross(B.AngularMomentum, B.AngularVelocity)), rBi);
		glm::vec3 Bt3 = glm::cross(B.AngularVelocity, wBxrBi);
		glm::vec3 Bt4 = B.LinearVelocity + wBxrBi;

		// compute the derivative of the contact normal
		glm::vec3 nDot;
		if(c.type == Contact::Type::VERTEX_FACE)
		{
			nDot = glm::cross(B.AngularVelocity, c.normal);
		}
		else 
		{
			glm::vec3 EAdot = glm::cross(A.AngularVelocity, c.edgeA[0] - c.edgeA[1]);
			glm::vec3 EBdot = glm::cross(B.AngularVelocity, c.edgeB[0] - c.edgeB[1]);
			glm::vec3 U = glm::cross(c.edgeA[0] - c.edgeA[1], EBdot) + glm::cross(c.edgeB[0] - c.edgeB[1], EAdot);
			nDot = (U - glm::dot(U, c.normal) * c.normal) / glm::length(glm::cross(c.edgeA[0] - c.edgeA[1], c.edgeB[0] - c.edgeB[1]));
		}

		b[i] = glm::dot(c.normal, At1 + At2 + At3 - Bt1 - Bt2 - Bt3) + 2.0f * glm::dot(nDot, At4 - Bt4);

	}
}