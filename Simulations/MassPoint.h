#pragma once

#include "util/vectorbase.h"
using namespace GamePhysics;

class MassPoint
{
public:
	MassPoint(Vec3 p = Vec3(0, 0, 0), Vec3 v = Vec3(0, 0, 0), Vec3 f = Vec3(0, 0, 0), float m = 0, bool fixed = 0);
	MassPoint(MassPoint* p);
	void setPosition(Vec3);
	void setVelocity(Vec3);
	Vec3 getPosition();
	Vec3 getVelocity();
	Vec3 getForce();
	float getMass() { return m_mass; }
	float getFixed() { return m_fixed; }

	void clearForce();
	void addForce(Vec3 f);
	void addAcceleration(Vec3 a);

private:
	Vec3 m_position;
	Vec3 m_velocity;
	Vec3 m_force;
	float m_mass;
	bool m_fixed;
};