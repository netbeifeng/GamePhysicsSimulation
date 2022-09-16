#include "Spring.h"

/*
Constructs a spring, storing its endpoints through the index of the point in a vector lying outside the influence of this class.
*/
Spring::Spring(float k, float L, float d, size_t pt1, size_t pt2)
{
	stiffness = k;
	restLength = L;
	dampingFactor = d;
	p1 = pt1;
	p2 = pt2;
}

float Spring::getRestLength() { return restLength; }
float Spring::getStiffness() { return stiffness; }
MassPoint* Spring::getP1(std::vector<MassPoint*> points) { return points[p1]; }
MassPoint* Spring::getP2(std::vector<MassPoint*> points) { return points[p2]; }

Vec3 Spring::getHookeForce(std::vector<MassPoint*> points)
{
	MassPoint* pt1 = getP1(points);
	MassPoint* pt2 = getP2(points);

	// Hooke's law:
	Vec3 distVect = pt1->getPosition() - pt2->getPosition();
	float currentLength = sqrt(distVect.squaredDistanceTo(Vec3(0, 0, 0)));
	Vec3 force = stiffness * (restLength - currentLength) * distVect / currentLength;

	return force;
}

void Spring::applyElasticForceToPoints(std::vector<MassPoint*> points)
{
	MassPoint* pt1 = getP1(points);
	MassPoint* pt2 = getP2(points);
	Vec3 f1 = getHookeForce(points);
	Vec3 f2 = -f1;

	//std::cout << "F1 " << f1 << std::endl;
	//std::cout << "F2 " << f2 << std::endl;
	pt1->addForce(f1);
	pt2->addForce(f2);

	// Damping
	Vec3 dampingForceP1 = dampingFactor * pt1->getVelocity() * pt1->getMass();
	Vec3 dampingForceP2 = dampingFactor * pt2->getVelocity() * pt2->getMass();
	pt1->addForce(-dampingForceP1);
	pt2->addForce(-dampingForceP2);
}