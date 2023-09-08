#include "MassSpringSystem.h"

MassSpringSystem::MassSpringSystem()
{
    this->m_fMass = 10;
    this->m_fStiffness = 100;
    this->m_fDamping = 1;

    this->m_iIntegrator = 0;
    holdingPoint = nullptr;
}

MassSpringSystem::MassSpringSystem(float mass, float stiffness, float damping, int integrator, shared_ptr<bool> gravity)
{
    this->m_fMass = mass;
    this->m_fStiffness = stiffness;
    this->m_fDamping = damping;
    this->m_iIntegrator = integrator;
    holdingPoint = nullptr;
    this->gravity = gravity;
}



void MassSpringSystem::setMass(float mass)
{
    m_fMass = mass;
}

void MassSpringSystem::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystem::setDampingFactor(float damping)
{
    m_fDamping = damping;
}

int MassSpringSystem::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    int index = massPoints.size();
    float mass = isFixed ? INT_MAX : m_fMass;
    auto pointPtr = make_shared<Point>(Point(Velocity, position, Vec3(0, 0, 0), isFixed, mass, index));
    massPoints.push_back(pointPtr);

    return index;
}

int MassSpringSystem::addMassPoint(shared_ptr<Point> point)
{
    point->index = massPoints.size();
    massPoints.push_back(point);
    return point->index;
}

void MassSpringSystem::addSpring(int masspoint1, int masspoint2, float initialLength)
{
    springs.push_back(Spring(massPoints[masspoint1].get(), massPoints[masspoint2].get(), initialLength));
}

void MassSpringSystem::addSling(int masspoint1, int masspoint2, float initialLength)
{
    springs.push_back(Sling(massPoints[masspoint1].get(), massPoints[masspoint2].get(), initialLength));
}

bool MassSpringSystem::removeSpring(int index)
{
    if (index < 0 || index >= springs.size())
        return false;

    springs.erase(springs.begin() + index);
    return true;
}

void MassSpringSystem::removeAllSprings()
{
    springs.clear();
}

int MassSpringSystem::getNumberOfMassPoints()
{
    return massPoints.size();
}

int MassSpringSystem::getNumberOfSprings()
{
    return springs.size();
}

Vec3 MassSpringSystem::getPositionOfMassPoint(int index)
{
    return massPoints[index]->position;
}

Vec3 MassSpringSystem::getVelocityOfMassPoint(int index)
{
    return massPoints[index]->velocity;
}

void MassSpringSystem::simulateTimestep(float timeStep)
{
    clearForces(massPoints);
    computeForces();

    for (shared_ptr<Point> p : massPoints)
    {
        integrate(*p, timeStep);
    }
}

void MassSpringSystem::reset()
{
    this->massPoints = {};
    this->springs = {};
}

bool MassSpringSystem::isHoldingAPoint()
{
    return holdingPoint != nullptr;
}

void MassSpringSystem::draw(DrawingUtilitiesClass* DUC)
{
    for (shared_ptr<Point> p : massPoints)
    {
        p->draw(DUC);
    }

    for (Spring s : springs)
    {
        s.draw(DUC);
    }
}

void MassSpringSystem::holdTheFirstPoint()
{
    if (getNumberOfMassPoints() > 0) {
        holdPoint(*massPoints[0]);
    }
}

void MassSpringSystem::releaseHoldingPoint()
{
    if (!isHoldingAPoint())
        return;

    releasePoint(*holdingPoint);
}

void MassSpringSystem::unholdHoldingPoint()
{
    if (isHoldingAPoint())
    {
        unHoldPoint(*holdingPoint);
    }
}

void MassSpringSystem::updateHoldingPoint(Vec3 position)
{
    if (!isHoldingAPoint())
        return;

    holdingPoint->position += position;
}

void MassSpringSystem::holdPoint(Point& point)
{
    point.makeFixed();
    holdingPoint = &point;
}

void MassSpringSystem::unHoldPoint(Point& point)
{
    point.isFixed = false;
}

void MassSpringSystem::releasePoint(Point& point)
{
    for (int si = springs.size() - 1; si >= 0; si--)
    {
        Spring& s = springs[si];
        if (s.point1 == &point || s.point2 == &point)
        {
            removeSpring(si);
        }
    }
    holdingPoint = nullptr;
}

void MassSpringSystem::clearForces(vector<shared_ptr<Point>>& points)
{
    for (shared_ptr<Point> p : points)
    {
        p->clearForce();

        if (p->isFixed) { continue; }

        //p.addForce(m_externalForce);
        if (auto g = gravity.lock()) 
        {
            float force = *g ? -GRAVITATIONAL_FORCE : 0;
            p->addForce(Vec3{ 0, force, 0 });
        }

        //p.position.y = max((float)p.position.y, m_fGroundLevel);
    }
}

void MassSpringSystem::computeForces()
{
    for (Spring& s : springs)
    {
        Vec3 force = s.computeElasticForces(m_fStiffness);
        s.addForceToEndPoints(force);
    }
}

void MassSpringSystem::integrate(Point& p, float timeStep)
{
    if (m_iIntegrator == 0)
    {
        integrateEuler(p, timeStep);
    }
    else if (m_iIntegrator == 1) 
    {
        integrateMidPoint(p, timeStep);
    }
}

void MassSpringSystem::integrateEuler(Point& p, float timeStep)
{
    p.position += timeStep * p.velocity;
    p.velocity += timeStep * m_fDamping * (p.force / p.mass);
}

void MassSpringSystem::integrateMidPoint(Point& p, float timeStep)
{
    Vec3 midPos = p.position + timeStep * 0.5 * p.velocity;
    Vec3 midVel = p.velocity + timeStep * 0.5 * m_fDamping * (p.force / p.mass);

    p.position += timeStep * midVel;

    vector<shared_ptr<Point>> midPoints = massPoints;

    clearForces(midPoints);

    for (Spring& s : springs)
    {
        // spring can calculate it since position is already updated
        Vec3 force = s.computeElasticForces(m_fStiffness);
        midPoints[s.point1->index]->addForce(force);
        midPoints[s.point2->index]->addForce(-force);
    }

    p.velocity += timeStep * m_fDamping * (midPoints[p.index]->force / p.mass);
}

