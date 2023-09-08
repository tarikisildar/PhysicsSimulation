#pragma once
#ifndef FINALSIMULATOR_h
#define FINALSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
#include "RigidbodySystem.h" 
#include "MassSpringSystem.h" 



class FinalSimulator :public Simulator {
public:
	// Construtors
	FinalSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void onMouseUp(int x, int y);
	void onMouseDown(int x, int y);
	void onKeyDown(UINT nChar);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	shared_ptr<Rigidbody> addRigidBody(Vec3 position, Vec3 size, int mass, bool is_static = false);
	void setOrientationOf(int i, Quat orientation);
	void setOrientationOf(Rigidbody& rigidbody, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);


	void applyImpulse(Rigidbody& first, Rigidbody& second);
	void addWalls();
	void addSlingShot(Vec3 position);



private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	float m_bounciness = 0.05f;
	INT32 simulationSpeed = 1;

	double forceMag = 10;
	double rayDiameter = 10;

	shared_ptr<bool> gravity;
	shared_ptr<bool> nBodyGravity;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	RigidBodySystem rigidbodySystem;
	MassSpringSystem massSpringSystem;
};
#endif