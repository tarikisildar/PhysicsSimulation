#pragma once
#include "Simulator.h"
#include "Spring.h"
#include "Sling.h"
#include "MassPoint.h"
#define GRAVITATIONAL_FORCE 9.71


class MassSpringSystem{

public:

	MassSpringSystem();
	MassSpringSystem(float mass, float stiffness, float damping, int integrator, shared_ptr<bool> gravity);


	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	int addMassPoint(shared_ptr<Point> point);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void addSling(int masspoint1, int masspoint2, float initialLength);
	bool removeSpring(int index);
	void removeAllSprings();
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void simulateTimestep(float timeStep);
	void reset();
	bool isHoldingAPoint();

	void draw(DrawingUtilitiesClass* DUC);

	
	void holdTheFirstPoint();
	void releaseHoldingPoint();
	void unholdHoldingPoint();
	void updateHoldingPoint(Vec3 position);


private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	vector<Spring> springs;
	vector<shared_ptr<Point>> massPoints;
	Point* holdingPoint = nullptr;

	weak_ptr<bool> gravity;

	void clearForces(vector<shared_ptr<Point>>& points);
	void computeForces();

	void integrate(Point& p, float timeStep);
	void integrateEuler(Point& p, float timeStep);
	void integrateMidPoint(Point& p, float timeStep);

	// Makes the point to listen mouse input for moving
	void holdPoint(Point& point);
	// Makes the point affected by physics again
	void unHoldPoint(Point& point);
	// Releases the point from all it's springs
	void releasePoint(Point& point);

};

