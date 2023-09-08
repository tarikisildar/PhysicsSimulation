#include "Spring.h"

Spring::Spring() {};

Spring::Spring(Point* p1, Point* p2, float initialLength) 
{
    this->point1 = p1;
    this->point2 = p2;
    this->initialLength = initialLength;
}


Vec3 Spring::computeElasticForces(float stiffness)
{
    Vec3 distance = (point2->position - point1->position);
    Vec3 length = sqrt(point2->position.squaredDistanceTo(point1->position));

    Vec3 dNorm = distance / length;

    Vec3 force = stiffness * (length - initialLength) * dNorm;

    return force;
}

void Spring::addForceToEndPoints(Vec3 force)
{
    point1->addForce(force);
    point2->addForce(-force);
}

void Spring::draw(DrawingUtilitiesClass* DUC)
{
    //if (!point1->position.x) return;

    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
    Vec3 color = Vec3(0, 0, 1);

    DUC->beginLine();
    DUC->drawLine(point1->position, color, point2->position, color);
    DUC->endLine();
}