#include "FinalSimulator.h"
#include "collisionDetect.h"

FinalSimulator::FinalSimulator()
{
	gravity = make_shared<bool>(false);
	nBodyGravity = make_shared<bool>(false);

	rigidbodySystem = RigidBodySystem(gravity, nBodyGravity);
	massSpringSystem = MassSpringSystem(10, 100, 1, 1, gravity);
}


struct Hit
{
	Rigidbody* hitObj;
	Vec3 impactPos;
};

void FinalSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidbodySystem.rigidbodies[i]->velocity = velocity;
}


const char* FinalSimulator::getTestCasesStr()
{
	return "DEMO1,DEMO2,DEMO3,DEMO4,DEMO5,DEMO6,DEMO7,DEMO8";
}

void FinalSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Bounciness", TW_TYPE_FLOAT, &m_bounciness, "min=0.0 step=0.01 max=1.0");
	//TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &gravity, ""); // likely won't work
}

void FinalSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	rigidbodySystem.clearState();
	rigidbodySystem.reset();
	massSpringSystem.reset();
}

void FinalSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	rigidbodySystem.draw(DUC);
	massSpringSystem.draw(DUC);
}

void FinalSimulator::notifyCaseChanged(int testCase)
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

		*nBodyGravity = false;
		break;

	case 2:
		std::cout << "DEMO3\n";
		addRigidBody({ -1.0f, 1.0f, 0.0f }, { 1.5f, 0.6f, 0.5f }, 1.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		// applyForceOnBody(0, { -1.0f, 1.0f, 0.0f }, { 200.0f,-200.0f,0.0f });

		addRigidBody({ 1.0f, 0.0f, 0.2f }, { 1.0f, 0.6f, 0.5f }, 1.0f);
		setOrientationOf(1, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		// applyForceOnBody(1, { 1.0f, 0.0f, 0.0f }, { -250.0f,250.0f,0.0f });

		*nBodyGravity = true;
		break;
	case 3:
		std::cout << "DEMO4\n";
		addRigidBody({ -2.0f, 1.0f, 0.0f }, { 0.2f, 0.2f, 0.2f }, 1.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		//applyForceOnBody(0, { -1.0f, 1.0f, 0.0f }, { 400.0f,-400.0f,0.0f });

		addSlingShot({ -1.5f, 0.0f, 0.0f });

		addRigidBody({ 2.0f, -0.58f, 0.0f }, { 0.2f, 1.0f, 0.2f }, 1.0f);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat((-0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));
		
		addRigidBody({ 1.6f, -0.54f, 0.0f }, { 0.2f, 1.05f, 0.2f }, 1.0f);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat((-0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));
		
		addRigidBody({ 1.2f, -0.58f, 0.0f }, { 0.2f, 1.0f, 0.2f }, 1.0f);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat((-0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));

		addRigidBody({ 1.6f, 0.07f, 0.0f }, { 1.0f, 0.2f, 0.2f }, 1.0f);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat((-0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));
		
		addWalls();

		for (shared_ptr<Rigidbody> r : rigidbodySystem.rigidbodies) {
			massSpringSystem.addMassPoint(r);
		}

		//int p3 = massSpringSystem.addMassPoint(Vec3(-0.5, 0.25, 0.0), Vec3(0.0, 0.0, 0.0), false);
		//int p1 = massSpringSystem.addMassPoint(Vec3(-0.3, 0.5, 0.0), Vec3(0.0, 0.0, 0.0), true);
		//int p2 = massSpringSystem.addMassPoint(Vec3(-0.1, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), true);
		this->massSpringSystem.addSling(0, 1, 0.5);
		this->massSpringSystem.addSling(2, 0, 0.5);
		*gravity = true;
		*nBodyGravity = true;
		break;
	case 4:
		std::cout << "DEMO5\n";
		addRigidBody({ -1.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f }, 1.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(0, { -1.0f, 1.0f, 0.0f }, { 0.0f,-40.0f,0.0f });

		addRigidBody({ 1.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f }, 1.0f);
		setOrientationOf(1, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(1, { 1.0f, 0.0f, 0.0f }, { 0.0f,+40.0f,0.0f });

		*gravity = false;
		*nBodyGravity = true;
		break;

	case 5: {
		std::cout << "DEMO6\n";
		addRigidBody({ 0.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f }, 100.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, 0.0f));
		//applyForceOnBody(0, { -1.0f, 1.0f, 0.0f }, { 0.0f,-40.0f,0.0f });

		float orbitForce = 700.0f;

		Vec3 body1Pos{ -1.0f, 0.0f, 0.0f };
		addRigidBody(body1Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(1, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(1, body1Pos, { 0.0f, -orbitForce, 0.0f });

		Vec3 body2Pos{ 1.0f, 0.0f, 0.0f };
		addRigidBody(body2Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(2, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(2, body2Pos, { 0.0f, orbitForce, 0.0f });

		Vec3 body3Pos{ 0.0f, 0.0f, 1.0f };
		addRigidBody(body3Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(3, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(3, body3Pos, { orbitForce, 0.0f, 0.0f });

		Vec3 body4Pos{ 0.0f, 0.0f, -1.0f };
		addRigidBody(body4Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(4, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(4, body4Pos, { -orbitForce, 0.0f, 0.0f });

		Vec3 body5Pos{ 0.0f, 1.0f, 0.0f };
		addRigidBody(body5Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(5, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(5, body5Pos, { 0.0f, 0.0f, orbitForce });

		Vec3 body6Pos{ 0.0f, -1.0f, 0.0f };
		addRigidBody(body6Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(6, Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f) * M_PI));
		applyForceOnBody(6, body6Pos, { 0.0f, 0.0f, -orbitForce });

		*gravity = false;
		*nBodyGravity = true;
		break;
	}
	case 6: {
		std::cout << "DEMO7\n";

		float orbitForce = 70.0f;
		Vec3 body0Pos{ -0.7f, 0.3f, 0.0f };
		addRigidBody(body0Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(0, body0Pos, { orbitForce, 0.0f, 0.0f });

		Vec3 body1Pos{ -0.7f, -0.3f, 0.0f };
		addRigidBody(body1Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(1, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(1, body1Pos, { -orbitForce, 0.0f, 0.0f });

		Vec3 body2Pos{ +0.7f, 0.0f, 0.3f };
		addRigidBody(body2Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(2, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(2, body2Pos, { orbitForce, 0.0f, 0.0f });

		Vec3 body3Pos{ +0.7f, 0.0f, -0.3f };
		addRigidBody(body3Pos, { 0.1f, 0.1f, 0.1f }, 1.0f);
		setOrientationOf(3, Quat(0.0f, 0.0f, (135.0f / 180.0f) * M_PI));
		applyForceOnBody(3, body3Pos, { -orbitForce, 0.0f, 0.0f });

		*gravity = false;
		*nBodyGravity = true;
		break;
	}
	case 7: {
		std::cout << "DEMO8\n";

		Vec3 body0Pos{ -2.0f, 0.0f, 0.0f };
		addRigidBody(body0Pos, { 0.1f, 0.1f, 0.1f }, 10.0f);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat((45.0f / 180.0f), (45.0f / 180.0f), (45.0f / 180.0f)* M_PI));

		addSlingShot({ -2.0f, 0.0f, 0.0f });

		addRigidBody({ 0.0f, 0.0f, 0.0f }, { 0.3f, 0.3f, 0.3f }, 1000.0f, true);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat(0.0f, 0.0f, 0.0f));

		addRigidBody({ 2.0f, 0.0f, 0.0f }, { 0.3f, 0.3f, 0.3f }, 1000.0f, true);
		setOrientationOf(*rigidbodySystem.rigidbodies.back(), Quat(0.0f, 0.0f, 0.0f));

		for (shared_ptr<Rigidbody> r : rigidbodySystem.rigidbodies) {
			massSpringSystem.addMassPoint(r);
		}
		this->massSpringSystem.addSling(0, 1, 0.5);
		this->massSpringSystem.addSling(2, 0, 0.5);
		*gravity = false;
		*nBodyGravity = true;
		break;
	}
	}
}

void FinalSimulator::addWalls()
{
	addRigidBody({ -0.0f, -1.2f, 0.0f }, { 5.0f, 0.6f, 3.0f }, 1.0f, true);

}

void FinalSimulator::addSlingShot(Vec3 position) 
{
	shared_ptr<Rigidbody> right = addRigidBody(position + Vec3(0.0f, 0.0f, 0.4f), {0.2f, 1.0f, 0.1f}, 1.0f, true);
	setOrientationOf(*right, Quat((-45.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));

	shared_ptr<Rigidbody> left = addRigidBody(position + Vec3(0.0f, 0.0f, -0.4f), { 0.2f, 1.0f, 0.1f }, 1.0f, true);
	setOrientationOf(*left, Quat((45.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));

	shared_ptr<Rigidbody> main = addRigidBody(position + Vec3(0.0f, -0.7f, 0.0f), { 0.2f, 0.5f, 0.6f }, 1.0f, true);
	setOrientationOf(*main, Quat((0.0f / 180.0f), (0.0f / 180.0f), (0.0f / 180.0f) * M_PI));
}

void FinalSimulator::externalForcesCalculations(float timeElapsed)
{
}

void FinalSimulator::simulateTimestep(float timeStep)
{
	rigidbodySystem.checkForCollisions(m_bounciness);
	rigidbodySystem.integrate(timeStep);
	rigidbodySystem.clearState();
	massSpringSystem.simulateTimestep(timeStep);
}


void FinalSimulator::onClick(int x, int y)
{
	float sensitivity = 250;
	massSpringSystem.updateHoldingPoint(Vec3((x - m_trackmouse.x) / sensitivity, (m_trackmouse.y - y) / sensitivity, 0));
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}


void FinalSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FinalSimulator::onMouseUp(int x, int y)
{
	massSpringSystem.unholdHoldingPoint();
}

void FinalSimulator::onMouseDown(int x, int y)
{
	massSpringSystem.holdTheFirstPoint();
}

void FinalSimulator::onKeyDown(UINT nChar)
{
	switch (nChar)
	{
		case VK_SPACE:
		{
			massSpringSystem.releaseHoldingPoint();
		}
	}
}

int FinalSimulator::getNumberOfRigidBodies()
{
	return rigidbodySystem.rigidbodies.size();
}

Vec3 FinalSimulator::getPositionOfRigidBody(int i)
{
	return rigidbodySystem.rigidbodies[i]->position;
}

Vec3 FinalSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidbodySystem.rigidbodies[i]->velocity;
}

Vec3 FinalSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidbodySystem.rigidbodies[i]->w;
}

void FinalSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	Rigidbody& body = *rigidbodySystem.rigidbodies[i];
	body.force += force;
	body.torque += cross(loc - body.position, force);
}

shared_ptr<Rigidbody> FinalSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, bool is_static)
{
	shared_ptr<Rigidbody> r = make_shared<Rigidbody>(Rigidbody(position, size, mass, 0, is_static));
	rigidbodySystem.rigidbodies.push_back(r);
	return r;
}

void FinalSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidbodySystem.rigidbodies[i]->orientation = orientation;
}

void FinalSimulator::setOrientationOf(Rigidbody& rigidbody, Quat orientation)
{
	rigidbody.orientation = orientation;
}
