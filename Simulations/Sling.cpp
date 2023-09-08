#include "Sling.h"

Sling::Sling() {};

Sling::Sling(Point* p1, Point* p2, float initialLength) 
{
    this->point1 = p1;
    this->point2 = p2;
    this->initialLength = initialLength;
}

Vec3 Sling::computeElasticForces(float stiffness) 
{
    Vec3 distance = (point2->position - point1->position);
    float length = sqrt(point2->position.squaredDistanceTo(point1->position));

    Vec3 dNorm = distance / length;

    Vec3 force = stiffness * std::max(length - initialLength, 0.0f) * dNorm;

    return force;
}