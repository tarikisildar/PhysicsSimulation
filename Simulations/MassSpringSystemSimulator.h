#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "Spring.h"
#include "MassPoint.h"
#include "MassSpringSystem.h"


// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change
#define GRAVITATIONAL_FORCE 9.71


class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
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

	// Specific Functions

	void applyExternalForce(Vec3 force);


	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	int m_iIntegrator;

	MassSpringSystem SpringSystem;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	shared_ptr<bool> gravity;

	void basicSetup();
	void simpleSetup();
	void springShootSetup();
	void complexSetup();
};
#endif