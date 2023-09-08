#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

struct Hit 
{
	Rigidbody* hitObj;
	Vec3 impactPos;
};

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	system.rigidbodies[i]->velocity = velocity;
}


RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	this->gravity = make_shared<bool>();
	system = RigidBodySystem(gravity);

}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "DEMO1,DEMO2,DEMO3,DEMO4";
}


void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Bounciness", TW_TYPE_FLOAT, &m_bounciness, "min=0.0 step=0.1 max=1.0");
	//TwAddVarCB(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, *RigidBodySystemSimulator::SetGravityCallback, RigidBodySystemSimulator::GetGravityCallback, gravity.get(), "");
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	system.clearState();
	system.reset();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	for (int i = 0; i < getNumberOfRigidBodies(); i++)
	{
		Rigidbody& rb = *system.rigidbodies[i];
		DUC->drawRigidBody(rb.getMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	switch (testCase)
	{
	case 0:
		std::cout << "DEMO1\n";
		std::cout << "Linear Velocity: (1 , 1, 0)" << std::endl;
		std::cout << "Angular Velocity: (-2.4 , 4.9, -1.7)" << std::endl;
		break;

	case 1:
		std::cout << "DEMO2\n";
		addRigidBody({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.6f, 0.5f }, 2.0f);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	
	case 2:
		std::cout << "DEMO3\n";
		addRigidBody({ -1.0f, 1.0f, 0.0f }, { 1.5f, 0.6f, 0.5f }, 1.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(0, { -1.0f, 1.0f, 0.0f }, {200.0f,-200.0f,0.0f });
		
		addRigidBody({ 1.0f, 0.0f, 0.2f }, { 1.0f, 0.6f, 0.5f }, 1.0f);
		setOrientationOf(1, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(1, { 1.0f, 0.0f, 0.0f }, {-250.0f,250.0f,0.0f });
		break;
	case 3:
		std::cout << "DEMO4\n";
		addRigidBody({ -1.0f, 1.0f, 0.0f }, { 1.5f, 0.6f, 0.5f }, 1.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(0, { -1.0f, 1.0f, 0.0f }, { 400.0f,-400.0f,0.0f });

		addRigidBody({ 1.0f, 0.0f, 0.2f }, { 1.0f, 0.6f, 0.5f }, 1.0f);
		setOrientationOf(1, Quat((0.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(1, { 1.0f, 0.0f, 0.0f }, { -450.0f,450.0f,0.0f });
		
		addRigidBody({ 0.0f, 2.0f, 0.2f }, { 0.5f, 0.5f, 0.5f }, 1.0f);
		setOrientationOf(2, Quat((0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));
		applyForceOnBody(2, { 1.0f, 0.0f, 0.0f }, { -0.0f,0.0f,0.0f });
		
		addRigidBody({ 0.4f, 2.5f, 0.0f }, { 3.0f, 0.2f, 0.2f }, 1.0f);
		setOrientationOf(3, Quat((0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));
		addWalls();
		//system.gravity = true;
		break;
	}
}

void RigidBodySystemSimulator::addWalls() 
{
	addRigidBody({ -0.0f, -1.2f, 0.0f }, { 5.0f, 0.6f, 3.0f }, 1.0f, true);

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	system.checkForCollisions(m_bounciness);
	system.integrate(timeStep);
	system.clearState();
	
}


void RigidBodySystemSimulator::onClick(int x, int y)
{

}


void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouseUp(int x, int y)
{
}

void RigidBodySystemSimulator::onMouseDown(int x, int y)
{
}

void RigidBodySystemSimulator::onKeyDown(UINT nChar)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return system.rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return system.rigidbodies[i]->position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return system.rigidbodies[i]->velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return system.rigidbodies[i]->w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	Rigidbody &body = *system.rigidbodies[i];
	body.force += force;
	body.torque += cross(loc - body.position, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, bool is_static)
{
	shared_ptr<Rigidbody> r = make_shared<Rigidbody>(Rigidbody(position, size, mass, 0, is_static));
	system.rigidbodies.push_back(r);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	system.rigidbodies[i]->orientation = orientation;
}
