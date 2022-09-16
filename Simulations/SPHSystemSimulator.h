#pragma once
#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "Particle.h"
#include "BoundaryBox.h"
#include "Kernel.h"

class SPHSystemSimulator : public Simulator {
public:
	SPHSystemSimulator();

	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	void initPositions(std::vector<Vec3>& positions);

	void init();
	void integrateAllByTimeStep(float timestep);
	float getPressureFromDensity(float density);
	BoundaryBox& getBoundaryBox() { return m_boundaryBox; }

	int getNumberOfParticles() const { return m_particlePositions.size(); }
	std::vector<Vec3>& getCurrentParticlePositions() { return m_particlePositions; }
private:

	void computeDensities();
	void velocityIntegration(float timestep);
	void computeViscosity();
	void updatePositions(float timestep);

	float m_particleNum = - 1.f; // error then 1
	float m_particleRadius = 0.02f;
	/* particle_mass = pow(particle_spacing, 3) * rest_density */
	float m_particleMass = 0.05f;
	float m_gasCoefficient = 1.655f;
	const float m_stiffness = 20000.f;
	const float m_viscosity = 0.02f;

	const float m_collisionBoundaryBackwards = .5f;

	std::vector<Vec3> m_particlePositions; // 

	std::vector<float> m_particleDensities; // as described

	std::vector<std::vector<long>> m_neighboursOfParticles; // neighbor particles of self
	std::vector<std::vector<Vec3>> m_relativePositions; // relative position of self, self - neighbor

	std::vector<Vec3> m_particleVelocities;
	std::vector<Vec3> m_viscocityForce;

	SPHKernels m_kernels;
	BoundaryBox m_boundaryBox;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};

#endif