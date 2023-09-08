#pragma once
#include "Rigidbody.h"

#define GRAVITY_ACC -9.8
//#define GRAVITY_CONSTANT 6.6743e-11f
// This is way too high, but the visuals are cooler this way ;)
#define GRAVITY_CONSTANT 6.6743e-3f

class RigidBodySystem
{
public:

	RigidBodySystem();
	RigidBodySystem(shared_ptr<bool> gravity);
	RigidBodySystem::RigidBodySystem(shared_ptr<bool> gravity, shared_ptr<bool> nBodyGravity);

	std::vector<shared_ptr<Rigidbody>> rigidbodies;
	weak_ptr<bool> gravity;
	weak_ptr<bool> nBodyGravity;

	void reset() 
	{
		rigidbodies.clear();
	}

	void integrate(float timeStep);
	void calculateNBodyGravity();
	void integratePosition(float timeStep, Rigidbody& rigidbody);
	void integrateVelocity(float timeStep, Rigidbody& rigidbody);
	void integrateOrientation(float timeStep, Rigidbody& rigidbody);
	void integrateAngularMomentum(float timeStep, Rigidbody& rigidbody);
	void updateAngularVelocity(Rigidbody& rigidbody);
	void clearState();
	void checkForCollisions(float bounciness);
	void applyImpulse(Rigidbody& first, Rigidbody& second, float bounciness);

	void draw(DrawingUtilitiesClass* DUC);
};