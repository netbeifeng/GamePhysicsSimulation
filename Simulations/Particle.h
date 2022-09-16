#pragma once

class FluidParticle {
public:
	FluidParticle(Vec3 pos);

	Vec3 getVelocity() { return m_vel; }
	void updateVelocity(Vec3 nVel);
	Vec3 getPosition() { return m_pos; }
	void updatePosition(Vec3 nPos);

private:
	float m_mass = 0.05f;
	float m_densities = -1.f;
	Vec3 m_pos;
	Vec3 m_vel = Vec3(0,0,0);
	Vec3 m_viscocityForce = Vec3(0,0,0);
	std::vector<Vec3> m_relPos = std::vector<Vec3>();
	std::vector<int> m_neighborIndices = std::vector<int>();
};