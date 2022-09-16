#include "MSRSSystemSimulator.h"
using namespace std;

MSRSSystemSimulator::MSRSSystemSimulator() {
	mss_simulator = MassSpringSystemSimulator();
	rbs_simulator = RigidBodySystemSimulator();
}

void MSRSSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	//this->DUC = DUC;

	mss_simulator.initUI(DUC);
	rbs_simulator.initUI(DUC);
}

void MSRSSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	// draw objects (springs and rigidbodies)
	mss_simulator.drawFrame(pd3dImmediateContext);
	rbs_simulator.drawFrame(pd3dImmediateContext);
}

void MSRSSystemSimulator::reset() {
	mss_simulator.reset();
	rbs_simulator.reset();
}

void MSRSSystemSimulator::notifyCaseChanged(int testCase) {
	//mss_simulator.initFrameObjects();

	//rbs_simulator.initFrameObjects();


	int height = 6, width = 6;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			Vec3 pos = Vec3(-0.25 + 0.1 * j, 0.5, -0.25 + 0.1 * i);
			Vec3 size = Vec3(0.005, 0.005, 0.005);
			rbs_simulator.addRigidBody(pos, size , 1);
			//rbs_simulator.applyForceOnBody(i * height + j, pos + size / 2, Vec3(0, -9.8f, 0));
			rbs_simulator.setOrientationOf(i * height + j, Quat(0, 0, 0));
			mss_simulator.addMassPoint(pos, Vec3(0, 0, 0), Vec3(0, 0, 0), 1, false);
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width - 1; j++)
		{
			mss_simulator.addSpring(700, 0.1f, 0.5, i * width + j, i * width + j + 1);
		}
	}
	for (int j = 0; j < width; j++)
	{
		for (int i = 0; i < height - 1; i++)
		{
			mss_simulator.addSpring(700, 0.1f, 0.5, i * width + j, (i + 1) * width + j);
		}
	}


	rbs_simulator.addRigidBody(Vec3(0, -0.125, 0), Vec3(0.25, 0.25, 0.25), 999999);
	//rbs_simulator.addImmo(Vec3(0, -0.25, 0), Vec3(0.5, 0.5, 0.5));
	rbs_simulator.applyForceOnBody(0, Vec3(0.25, 0.25, 0.25), Vec3(0, 0, 0));
	rbs_simulator.setOrientationOf(rbs_simulator.getNumberOfRigidBodies() - 1, Quat(0, 0, 0));

	rbs_simulator.setUpFloorAndWalls();
}

void MSRSSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void MSRSSystemSimulator::simulateTimestep(float timeStep) {
	mss_simulator.addGravity(Vec3(0, -9.8f, 0));



	for (int i = 0; i < rbs_simulator.getNumberOfRigidBodies() - 1; i++) {
		rbs_simulator.getRigidBodyByIdx(i)->applyForce(mss_simulator.getMassPoints()[i]->getForce());
	}

	rbs_simulator.integrateAll(timeStep);
	rbs_simulator.checkCollision();

	for (int i = 0; i < rbs_simulator.getNumberOfRigidBodies() - 1; i++) {
		Vec3 RP = rbs_simulator.getRigidBodyByIdx(i)->getCenterPosition();
		Vec3 MP = mss_simulator.getMassPoints()[i]->getPosition();
		//std::cout << "RP:" << rbs_simulator.getRigidBodyByIdx(i)->getCenterPosition() << std::endl;
		//std::cout << "MP:" << mss_simulator.getMassPoints()[i]->getPosition() << std::endl;
		//std::cout << rbs_simulator.getNormOfVector(RP - MP) << std::endl;

		//Vec3 RV = rbs_simulator.getRigidBodyByIdx(i)->getLinearVelocity();
		//Vec3 MV = mss_simulator.getMassPoints()[i]->getVelocity();
		//std::cout << rbs_simulator.getNormOfVector(RV - MV) << std::endl;

		mss_simulator.getMassPoints()[i]->setPosition(rbs_simulator.getRigidBodyByIdx(i)->getCenterPosition());
		mss_simulator.getMassPoints()[i]->setVelocity(rbs_simulator.getRigidBodyByIdx(i)->getLinearVelocity());

	}
	//mss_simulator.makeEulerStep(0.002);
	mss_simulator.makeEulerStep(timeStep);
}

const char* MSRSSystemSimulator::getTestCasesStr() {
	return "Demo";
}

void MSRSSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MSRSSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
