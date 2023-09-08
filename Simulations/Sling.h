#pragma once
#include "Spring.h"

// Slings are pretty much the same with Springs, except they don't apply force when their length is smaller than their rest length.
class Sling : public Spring 
{
public:
	Sling();
	Sling(Point* p1, Point* p2, float initialLength);
	virtual Vec3 computeElasticForces(float stiffness) override;
};
