#include "collision.h"

namespace PhysicEngine 
{

	void ProcessCollidingContact(RigidBody& rbA, RigidBody& rbB, const glm::vec3& collidingPoint, const glm::vec3& normal)
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

}