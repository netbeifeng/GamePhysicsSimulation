#pragma once

#include <vector>

#include "util/vectorbase.h"
using namespace GamePhysics;
#include "MassPoint.h"

class Spring
{
public:
	Spring(float k = 0, float L = 0, float d = 0, size_t p1 = 0, size_t p2 = 0);

	MassPoint* getP1(std::vector<MassPoint*> points);
	MassPoint* getP2(std::vector<MassPoint*> points);
	float getRestLength();
	float getStiffness();

	Vec3 getHookeForce(std::vector<MassPoint*>);
	void applyElasticForceToPoints(std::vector<MassPoint*>);

private:
	size_t p1;
	size_t p2;
	float restLength;
	float stiffness;
	float dampingFactor;
};