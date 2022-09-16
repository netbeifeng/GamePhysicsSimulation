#include "SPHSystemSimulator.h"
#include "parallel_util.hpp"

SPHSystemSimulator::SPHSystemSimulator()
	: m_kernels(m_particleRadius * 4),
	m_boundaryBox(m_particleRadius, Vec3(0.5, 1, 0.5), Vec3(-0.5, -0.5, -0.5)) {
}

void SPHSystemSimulator::init() {
	std::cout << "Initializing . . ." << std::endl;
	if (this->getNumberOfParticles() > 0) {
		this->reset();
	}

	else {
		std::vector<Vec3> pos;

		for (int i = 0; i < 10; ++i) {
			for (int j = 0; j < 20; ++j) {
				for (int k = 0; k < 10; ++k) {
					pos.push_back(Vec3(i * 2 * m_particleRadius - 0.125, j * 2 * m_particleRadius + 0.25, k * 2 * m_particleRadius - 0.125));
				}
			}
		}

		this->initPositions(pos);

		m_boundaryBox.setTopBoundary(Vec3(0.5, 1, 0.5));
		m_boundaryBox.setBottomBoundary(Vec3(-0.5, -0.5, -0.5));
	}
	std::cout << "Initialized ! ! !" << std::endl;
}

void SPHSystemSimulator::initPositions(std::vector<Vec3>& positions) {
	m_particlePositions = positions;
	m_particleNum = positions.size();
	m_particleDensities.resize(m_particleNum);
	m_neighboursOfParticles.resize(m_particleNum);
	m_relativePositions.resize(m_particleNum);
	m_particleVelocities.assign(m_particleNum, Vec3(0.0f, 0.0f, 0.0f));
	m_viscocityForce.resize(m_particleNum);
}

void SPHSystemSimulator::integrateAllByTimeStep(float timestep) {
	// for each integration step
	// 1. get neighbor lists for each particle
	// 2. get relative distance for each particle (relative means this.pos - neighbor.pos)
	// 3. compute density for each particle
	// 4. compute pressure for each particle
	// 5. compute acceleration by pressure
	// 6. compute viscocity for each particle
	// 7. compute acceleration by viscocity
	// 8. update position by accleration

	m_boundaryBox.searchNeighbors(m_particlePositions, m_neighboursOfParticles, m_relativePositions);
	computeDensities();
	velocityIntegration(timestep);
	computeViscosity();
	updatePositions(timestep);
}

void SPHSystemSimulator::computeDensities() {
	auto densityIteration = [&](int index) {
		std::vector<Vec3>& relativePositions = m_relativePositions[index];
		if (relativePositions.size() == 0) return;

		float density = m_kernels.W0();
		for (Vec3& rp : relativePositions)
			density += m_kernels.W(rp);
		density *= m_particleMass;

		m_particleDensities[index] = density;
	};

	parallelutil::parallel_for(m_particlePositions.size(), densityIteration);
}

float SPHSystemSimulator::getPressureFromDensity(float density) {
	float ratio = density / 1000.0f; // < rest density 1000.f
	float pressure = m_gasCoefficient * ratio - 1.0f;
	if (pressure < .0f || ratio < 1.) {
		return 0.0f;
	}
	return pressure;
}

void SPHSystemSimulator::velocityIntegration(float timestep) {
	parallelutil::parallel_for(m_particlePositions.size(), [&](int i) {
		std::vector<long> neighbors = m_neighboursOfParticles[i];
		if (neighbors.size() == 0) {
			m_particleVelocities[i].y -= timestep * 9.8f; // gravity
			return;
		}

		std::vector<Vec3>& relPositions = m_relativePositions[i];
		float density_i = m_particleDensities[i];
		float pressure_i = getPressureFromDensity(density_i);
		float part_i = pressure_i / (density_i * density_i);

		/* Compute the pressure acceleration caused by normal fluid particles */
		Vec3 acc( 0.0f, 0.0f, 0.0f);

		for (int j = 0; j < neighbors.size(); ++j) {
			int neighbor_j = neighbors[j];
			const float neighbor_j_density = float(m_particleDensities[neighbor_j]);
			const float neighbor_j_pressure = getPressureFromDensity(neighbor_j_density);
			const float part_neighbor_j = neighbor_j_pressure / (neighbor_j_density * neighbor_j_density);
			const Vec3 r = relPositions[j];

			/* Pressure acceleration */
			acc -= (part_i + part_neighbor_j) * m_kernels.gradW(r);
		}

		/* Compute the pressure acceleration caused by ghost boundary particles */
		for (int idx = neighbors.size(); idx < relPositions.size(); ++idx) {
			const Vec3 r = relPositions[idx];
			acc -= part_i * m_kernels.gradW(r);
		}

		acc *= m_stiffness * m_particleMass;
		acc.y -= 9.8f; /* add gravity */

		/* Update velocity from acceleration and gravity */
		m_particleVelocities[i] += acc * timestep;
	});
}

void SPHSystemSimulator::computeViscosity() {
	auto diffusionVelocityIteration = [&](const int index) {
		std::vector<long>& neighbors = m_neighboursOfParticles[index];
		if (neighbors.empty()) {
			m_viscocityForce[index] = Vec3(0, 0, 0);
			return;
		}

		std::vector<Vec3>& relPositions = m_relativePositions[index];
		Vec3 pvel = m_particleVelocities[index];

		Vec3 force(0.f, 0.f, 0.f);
		for (int j = 0; j != neighbors.size(); ++j) {
			int neighbor_j = neighbors[j];
			Vec3 neighbor_j_velocity = m_particleVelocities[neighbor_j];
			float neighbor_j_density = m_particleDensities[neighbor_j];
			Vec3 r = relPositions[j];

			force += (1.0f / neighbor_j_density) * m_kernels.W(r) * (neighbor_j_velocity - pvel);
		}

		force *= m_viscosity * m_particleMass;
		m_viscocityForce[index] = force;
		m_particleVelocities[index] += m_viscocityForce[index];
	};

	parallelutil::parallel_for(m_particlePositions.size(), diffusionVelocityIteration);

}

void SPHSystemSimulator::updatePositions(float timestep) {
	auto positionUpdateIteration = [&](int index) {
		Vec3 velocity_i = m_particleVelocities[index];
		auto position_i = m_particlePositions[index] + velocity_i * timestep;

		if (m_boundaryBox.collideWithBoundary(position_i, velocity_i, m_collisionBoundaryBackwards)) {
			m_particleVelocities[index] = velocity_i;
		}

		m_particlePositions[index] = position_i;
	};

	parallelutil::parallel_for(m_particlePositions.size(), positionUpdateIteration);
}

const char* SPHSystemSimulator::getTestCasesStr() {
	return "DEMO";
}

void SPHSystemSimulator::reset() {
	std::cout << "Reset Called" << std::endl;
	m_particlePositions.clear();
	m_particleVelocities.clear();
	m_particleDensities.clear();
	m_neighboursOfParticles.clear();
	m_relativePositions.clear();
	m_viscocityForce.clear();
	init();
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	//TwAddVarRW(DUC->g_pTweakBar, "Particle Radius", TW_TYPE_FLOAT, &m_particleRadius, "min=0.0 max=1.0 step=0.001");

	TwAddButton(DUC->g_pTweakBar, "One_Big_Step", [](void* clientData) {
		SPHSystemSimulator* simulator = static_cast<SPHSystemSimulator*>(clientData);
			// Call back
			simulator->integrateAllByTimeStep(simulator->m_particleRadius);
	}, this, "");
}

void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	init();
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_boundaryBox.setTopBoundary(m_boundaryBox.getTopBoundary() + inputWorld);
		m_boundaryBox.setBottomBoundary(m_boundaryBox.getBottomBoundary() + inputWorld);
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
}

void SPHSystemSimulator::simulateTimestep(float timeStep)
{
	this->integrateAllByTimeStep(timeStep);
}

void SPHSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i < m_particlePositions.size(); i++) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(m_particlePositions[i], Vec3(0.015f, 0.015f, 0.015f));
	}
}

void SPHSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SPHSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}



