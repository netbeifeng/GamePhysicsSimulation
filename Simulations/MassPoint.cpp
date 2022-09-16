#include "MassPoint.h"

MassPoint::MassPoint(Vec3 p, Vec3 v, Vec3 f, float m, bool b)
{
	m_position = p;
	m_velocity = v;
	m_force = f;
	m_mass = m;
	m_fixed = b;
}

MassPoint::MassPoint(MassPoint* p) {
	m_position = p->getPosition();
	m_velocity = p->getVelocity();
	m_force = p->getForce();
	m_mass = p->getMass();
	m_fixed = p->getFixed();
}

void MassPoint::setPosition(Vec3 p) { m_position = p; }
void MassPoint::setVelocity(Vec3 v) { m_velocity = v; }

Vec3 MassPoint::getPosition() { return m_position; }
Vec3 MassPoint::getVelocity() { return m_velocity; }
Vec3 MassPoint::getForce() { return m_force; }

void MassPoint::clearForce() {
	if (!m_fixed)
		m_force = Vec3(0, 0, 0);
}

void MassPoint::addForce(Vec3 f)
{
	if (!m_fixed)
		m_force += f;
}

void MassPoint::addAcceleration(Vec3 a)
{
	if (!m_fixed)
		m_force += a * m_mass;
}

