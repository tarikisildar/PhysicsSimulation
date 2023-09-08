#pragma once
#include "MassPoint.h"

class Spring
{
public:
	Spring();
	Spring(Point* p1, Point* p2, float initialLength);
	Point* point1;
	Point* point2;
	float initialLength;

	virtual Vec3 computeElasticForces(float stiffness);
	void addForceToEndPoints(Vec3 force);

	void draw(DrawingUtilitiesClass* DUC);
};