#pragma once
#include "Simulator.h"
#include "MassPoint.h"


class Rigidbody : public Point
{
public:
	Vec3 torque;
	// angular momentum
	Vec3 L;
	// angular velocity
	Vec3 w;
	Mat4d inverse_inertia;
	Vec3 size;
	Quat orientation;

	Rigidbody(Vec3 pos, Vec3 size, float mass, int index, bool isFixed = false) : Point(Vec3(0,0,0), pos, Vec3(0,0,0), isFixed, mass, index)
	{
		this->size = size;
		this->inverse_inertia = preComputeInertia(size).inverse();
	}

	Mat4d getRotatedInertia()
	{
		auto r = orientation.getRotMat();
		auto rT = r;
		rT.transpose();
		return rT * inverse_inertia * r;
	}

	Mat4 getMatrix()
	{
		Mat4 translation;
		translation.initTranslation(position.x, position.y, position.z);

		Mat4 scale;
		scale.initScaling(size.x, size.y, size.z);

		auto r = orientation.getRotMat();
		return scale * r * translation;
	}

	virtual void draw(DrawingUtilitiesClass* DUC) override 
	{
		DUC->drawRigidBody(getMatrix());
	}
private:
	// computation for cuboid shapes
	Mat4d preComputeInertia(Vec3 size)
	{
		Mat4d inertia;
		inertia.initId();
		float f1By12 = (1.0f / 12.0f);

		inertia.value[0][0] = ((size.y * size.y) + (size.z * size.z)) * mass * f1By12;
		inertia.value[1][1] = ((size.x * size.x) + (size.z * size.z)) * mass * f1By12;
		inertia.value[2][2] = ((size.x * size.x) + (size.y * size.y)) * mass * f1By12;

		return inertia;
	}
		
};