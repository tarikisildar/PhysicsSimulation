#pragma once
#include "Simulator.h"
#include "Case.h"

class Point
{
public:

	Point();
	Point(Vec3 vel, Vec3 pos, Vec3 force, bool isFixed, float mass, int index);

	Vec3 force;
	Vec3 velocity;
	Vec3 position;
	float mass;
	bool isFixed;
	int index;

	void clearForce();

	void addForce(Vec3 f);

	void makeFixed();

	virtual void draw(DrawingUtilitiesClass* DUC);

};