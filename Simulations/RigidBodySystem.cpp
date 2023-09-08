#include "RigidBodySystem.h"
#include "collisionDetect.h"
#include "MassPoint.h"


RigidBodySystem::RigidBodySystem()
{
}

RigidBodySystem::RigidBodySystem(shared_ptr<bool> gravity)
{
	this->gravity = gravity;
}

RigidBodySystem::RigidBodySystem(shared_ptr<bool> gravity, shared_ptr<bool> nBodyGravity)
{
	this->gravity = gravity;
	this->nBodyGravity = nBodyGravity;
}

void RigidBodySystem::integrate(float timeStep)
{
	if (auto nBodyG = nBodyGravity.lock()) {
		if (*nBodyG) {
			calculateNBodyGravity();
		}
	}

	for (shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		if (rigidbody->isFixed)
			continue;

		integratePosition(timeStep, *rigidbody);
		integrateVelocity(timeStep, *rigidbody);
		integrateOrientation(timeStep, *rigidbody);
		integrateAngularMomentum(timeStep, *rigidbody);
		updateAngularVelocity(*rigidbody);
	}
}

void RigidBodySystem::calculateNBodyGravity()
{
	for (int i = 0; i < rigidbodies.size(); i++) {
		Rigidbody& bodyI = *rigidbodies[i];

		for (int j = i + 1; j < rigidbodies.size(); j++) {
			Rigidbody& bodyJ = *rigidbodies[j];

			Vec3 iToJ = bodyJ.position - bodyI.position;
			float r = normalize(iToJ);
			float fScalar = (GRAVITY_CONSTANT * bodyI.mass * bodyJ.mass) / (r * r);

			bodyI.addForce(iToJ * fScalar);
			bodyJ.addForce(iToJ * -fScalar);
		}
	}
}

void RigidBodySystem::integratePosition(float timeStep, Rigidbody& rigidbody)
{
	rigidbody.position += timeStep * rigidbody.velocity;
}

void RigidBodySystem::integrateVelocity(float timeStep, Rigidbody& rigidbody)
{
	float gravitational_acc = 0;
	if (auto g = gravity.lock()) {
		if (*g) {
			gravitational_acc = GRAVITY_ACC;
		}
	}
	auto a = rigidbody.force * (1.0f / rigidbody.mass) + Vec3(0,gravitational_acc,0);
	rigidbody.velocity += timeStep * a;
}

void RigidBodySystem::integrateOrientation(float timeStep, Rigidbody& rigidbody)
{
	auto _0w = Quat{ rigidbody.w.x, rigidbody.w.y, rigidbody.w.z, 0.0f };
	rigidbody.orientation += (timeStep * 0.5f) * _0w * rigidbody.orientation;
	rigidbody.orientation = rigidbody.orientation.unit();
}

void RigidBodySystem::integrateAngularMomentum(float timeStep, Rigidbody& rigidbody)
{
	rigidbody.L += timeStep * rigidbody.torque;
}

void RigidBodySystem::updateAngularVelocity(Rigidbody& rigidbody)
{
	rigidbody.w = rigidbody.getRotatedInertia() * rigidbody.L;

}

void RigidBodySystem::clearState()
{
	for (shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		rigidbody->force = 0;
		rigidbody->torque = 0;
	}
}

void RigidBodySystem::checkForCollisions(float bounciness)
{
	for (int i = 0; i < rigidbodies.size(); i++)
	{
		shared_ptr<Rigidbody> first = rigidbodies[i];
		for (int j = 0; j < rigidbodies.size(); j++)
		{
			shared_ptr<Rigidbody> second = rigidbodies[j];

			applyImpulse(*first, *second, bounciness);
		}
	}
}

void RigidBodySystem::draw(DrawingUtilitiesClass* DUC) 
{
	for (int i = 0; i < rigidbodies.size(); i++)
	{
		shared_ptr<Rigidbody> rb = rigidbodies[i];
		rb->draw(DUC);
	}
}

void RigidBodySystem::applyImpulse(Rigidbody& first, Rigidbody& second, float bounciness)
{
	CollisionInfo info = checkCollisionSAT(first.getMatrix(), second.getMatrix());

	Vec3 localContactPointFirst = info.collisionPointWorld - first.position;
	Vec3 localContactPointSecond = info.collisionPointWorld - second.position;

	Vec3 firstContactVel = first.velocity + cross(first.w, localContactPointFirst);
	Vec3 secondContactVel = second.velocity + cross(second.w, localContactPointSecond);
	Vec3 relativeVel = firstContactVel - secondContactVel;

	if (info.isValid)
	{
		// [(Ia-1 (xa x n)) x xa]
		Vec3 firstInverse = cross(first.getRotatedInertia() * cross(localContactPointFirst, info.normalWorld), localContactPointFirst);
		// [(Ib-1 (xb x n)) x xb]
		Vec3 secondInverse = cross(second.getRotatedInertia() * cross(localContactPointSecond, info.normalWorld), localContactPointSecond);

		// 1/Ma + 1/Mb + firstInverse dot secondInverse
		float deminator = (1.0f / first.mass) + (1.0f / second.mass) + dot(firstInverse, secondInverse);

		float nominator = dot(-(1 + bounciness) * relativeVel, info.normalWorld);

		float impulse = nominator / deminator;

		// small values somehow break the simulation
		if (impulse < 0.01f)
		{
			return;
		}

		Vec3 Jn = impulse * info.normalWorld;

		if (!first.isFixed)
		{
			first.velocity += Jn / first.mass;
			first.L += cross(localContactPointFirst, Jn);
			first.position += info.depth * info.normalWorld;
		}

		if (!second.isFixed)
		{
			second.velocity -= Jn / second.mass;
			second.L -= cross(localContactPointSecond, Jn);
			second.position -= info.depth * info.normalWorld;
		}

	}
}
