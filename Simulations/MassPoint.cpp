#include "MassPoint.h"

Point::Point()
{

}

Point::Point(Vec3 vel, Vec3 pos, Vec3 force, bool isFixed, float mass, int index)
{
    this->force = force;
    this->velocity = vel;
    this->position = pos;
    this->isFixed = isFixed;
    this->mass = mass;
    this->index = index;
}

void Point::clearForce()
{
    force = Vec3(0, 0, 0);
}

void Point::addForce(Vec3 f)
{
    //Vec3(0, GRAVITATIONAL_FORCE, 0);
    if (isFixed) {
        return;
    }
    force += f;
}

void Point::makeFixed()
{
    clearForce();
    velocity = Vec3(0, 0, 0);
    isFixed = true;
}

void Point::draw(DrawingUtilitiesClass* DUC)
{
    //if (!position.x) return;
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
    DUC->drawSphere(position, Vec3{ 0.1f,0.1f,0.1f });
}

