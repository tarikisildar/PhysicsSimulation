#include "MassSpringSystemSimulator.h"
#include "Case.h"


MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    this->m_iTestCase = 0;
    this->m_iIntegrator = EULER;
    this->gravity = make_shared<bool>();

    this->SpringSystem = MassSpringSystem(10, 100, 1, m_iIntegrator, gravity);
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "BasicTest,SlingShot,Setup-1,SpringShoot";
}

const char* getIntegratorNames()
{
    return "Euler,Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", getIntegratorNames());
    //TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &this->m_fDamping, "min=0 max=2 step=0.1");


    switch (m_iTestCase)
    {
    case 0:break;
    case 1:
    case 2:
        TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
        break;
    default:break;
    }

}

void MassSpringSystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

    applyExternalForce(Vec3(0, 0, 0));

    SpringSystem.reset();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    SpringSystem.draw(DUC);
}


void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    this->reset();
    TestCase curentCase = (TestCase)testCase;
    switch (curentCase)
    {
    case Basic:
        basicSetup();
        break;

    case SlingShot:
        simpleSetup();
        break;

    case Setup1:
        complexSetup();
        break;

    case SpringShoot:
        springShootSetup();
        break;

    default:
        break;
    }

}
void MassSpringSystemSimulator::basicSetup()
{
    cout << "After an Euler Step:" << endl;
    cout << "point 0 pos (-0.1, 0, 0), vel (-1, 0.4, 0)" << endl;
    cout << "point 1 pos (0.1, 2, 0), vel (1, -0.4, 0)" << endl;

    cout << "After a Midpoint Step:" << endl;
    cout << "point 0 pos (-0.1, 0.02, 0), vel (-0.98, 0.4004, 0)" << endl;
    cout << "point 1 pos (0.1, 1.98, 0), vel (0.98, -0.4004, 0)" << endl;
}

void MassSpringSystemSimulator::simpleSetup()
{
    int p3 = SpringSystem.addMassPoint(Vec3(-0.5, 0.25, 0.0), Vec3(0.0, 0.0, 0.0), false);
    int p1 = SpringSystem.addMassPoint(Vec3(-0.3, 0.5, 0.0), Vec3(0.0, 0.0, 0.0), true);
    int p2 = SpringSystem.addMassPoint(Vec3(-0.1, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), true);
    this->SpringSystem.addSpring(p1, p3, 1.0);
    this->SpringSystem.addSpring(p2, p3, 1.0);
}

void MassSpringSystemSimulator::springShootSetup()
{
    int p3 = SpringSystem.addMassPoint(Vec3(-0.5, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), false);
    int p1 = SpringSystem.addMassPoint(Vec3(-0.5, -0.5, 0.0), Vec3(0.0, 0.0, 0.0), true);
    this->SpringSystem.addSpring(p1, p3, 1.5);
}

void MassSpringSystemSimulator::complexSetup()
{
    int p1 = SpringSystem.addMassPoint(Vec3(0.0, 0.0, 0.0), Vec3(0, 0.0, 0.0), true);
    int p2 = SpringSystem.addMassPoint(Vec3(0.0, 2.0, 0.0), Vec3(1.0, 0.0, 0.0), false);

    int pBox11 = SpringSystem.addMassPoint(Vec3(1.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), false);
    int pBox12 = SpringSystem.addMassPoint(Vec3(2.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), false);
    int pBox13 = SpringSystem.addMassPoint(Vec3(1.0, 1.0, 0.0), Vec3(0.0, 0.0, 0.0), true);
    int pBox14 = SpringSystem.addMassPoint(Vec3(2.0, 1.0, 0.0), Vec3(0.0, 0.0, 0.0), false);


    int pBox21 = SpringSystem.addMassPoint(Vec3(1.0, 0.0, 1.0), Vec3(0.0, 0.0, 0.0), false);
    int pBox22 = SpringSystem.addMassPoint(Vec3(2.0, 0.0, 1.0), Vec3(0.0, 0.0, 0.0), false);
    int pBox23 = SpringSystem.addMassPoint(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 0.0, 0.0), false);
    int pBox24 = SpringSystem.addMassPoint(Vec3(2.0, 1.0, 1.0), Vec3(0.0, 0.0, 0.0), false);
    int pBoxMid = SpringSystem.addMassPoint(Vec3(1.5, 0.5, 0.5), Vec3(0.0, 0.0, 0.0), false);
    this->SpringSystem.addSpring(p1, p2, 0.2);

    this->SpringSystem.addSpring(pBox11, pBox12, 0.8);
    this->SpringSystem.addSpring(pBox13, pBox14, 0.8);
    this->SpringSystem.addSpring(pBox11, pBox13, 0.8);
    this->SpringSystem.addSpring(pBox12, pBox14, 0.8);

    this->SpringSystem.addSpring(pBox21, pBox22, 0.8);
    this->SpringSystem.addSpring(pBox23, pBox24, 0.8);
    this->SpringSystem.addSpring(pBox21, pBox23, 0.8);
    this->SpringSystem.addSpring(pBox22, pBox24, 0.8);

    this->SpringSystem.addSpring(pBox21, pBox11, 0.8);
    this->SpringSystem.addSpring(pBox23, pBox13, 0.8);
    this->SpringSystem.addSpring(pBox22, pBox12, 0.8);
    this->SpringSystem.addSpring(pBox24, pBox14, 0.8);


    this->SpringSystem.addSpring(pBoxMid, pBox11, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox12, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox13, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox14, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox21, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox22, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox23, 1.0);
    this->SpringSystem.addSpring(pBoxMid, pBox24, 1.0);

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        float inputScale = 0.1f;
        inputWorld = inputWorld * inputScale;
        //applyExternalForce(inputWorld);
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    SpringSystem.simulateTimestep(timeStep);
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
    float sensitivity = 250;
    SpringSystem.updateHoldingPoint(Vec3((x - m_trackmouse.x) / sensitivity, (m_trackmouse.y - y) / sensitivity, 0));

    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = m_trackmouse.x;
    m_oldtrackmouse.y = m_trackmouse.y;

    
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouseUp(int x, int y)
{
    SpringSystem.unholdHoldingPoint();
}

void MassSpringSystemSimulator::onMouseDown(int x, int y)
{
    SpringSystem.holdTheFirstPoint();
}

void MassSpringSystemSimulator::onKeyDown(UINT nChar) 
{
    switch (nChar)
    {
        case VK_SPACE:
        {
            SpringSystem.releaseHoldingPoint();
        }
    }
}


void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
    m_externalForce = force;
}
