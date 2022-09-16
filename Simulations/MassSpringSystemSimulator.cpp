#include "MassSpringSystemSimulator.h"
#include "MassPoint.h"
#include "Spring.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	setMass(10);
	setStiffness(10000);
	setDampingFactor(0.8);

	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

/// *** UI functions *** ///

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4 (Euler),Demo 4 (Midpoint), Demo 5";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	// Draw line for each spring
	DUC->beginLine();
	for each (Spring s in m_springs)
	{
		DUC->drawLine(s.getP1(m_points)->getPosition(), Vec3(1, 0, 0), s.getP2(m_points)->getPosition(), Vec3(0, 1, 0));

	}
	DUC->endLine();
	for each (Spring s in m_springs)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(1, 1, 1));
		DUC->drawSphere(s.getP1(m_points)->getPosition(), Vec3(0.008, 0.008, 0.008));
		DUC->drawSphere(s.getP2(m_points)->getPosition(), Vec3(0.008, 0.008, 0.008));
	}
}
void MassSpringSystemSimulator::addMassPoint(Vec3 pos, Vec3 vel, Vec3 foc, float mass, bool fixed) {
	m_points.push_back(new MassPoint(pos, vel, foc, mass, fixed));
}

void MassSpringSystemSimulator::addSpring(float stif, float rl, float damping, int pdx1, int pdx2) {
	m_springs.push_back(Spring(stif, rl, damping, pdx1, pdx2));
}

void MassSpringSystemSimulator::initFrameObjects() {
	
	float restLength = 0.1f;
	setStiffness(700);
	m_points.clear();
	m_springs.clear();
	int height = 11, width = 11;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			m_points.push_back(new MassPoint(Vec3(-0.5 + 0.1 * j, 0.4 , -0.5 + 0.1 * i), Vec3(0,0,0), Vec3(0,0,0), m_fMass, false));
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width - 1; j++)
		{
			m_springs.push_back(Spring(m_fStiffness, restLength, m_fDamping, i * width + j, i * width + j + 1));
		}
	}
	for (int j = 0; j < width; j++)
	{
		for (int i = 0; i < height - 1; i++)
		{
			m_springs.push_back(Spring(m_fStiffness, restLength, m_fDamping, i * width + j, (i + 1) * width + j));
		}
	}
}
vector<MassPoint*> MassSpringSystemSimulator::getMassPoints() {
	return m_points;
}
/*
Reacts to change in test case (e.g. the drop-down menu, or when "reset scene" is pressed).
- Sets up scene, and
- prints out a short message to standard output.
*/
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	// Pass argument to the attribute.
	m_iTestCase = testCase;

	MassPoint* pt0, * pt1;
	Spring s;
	float m0, m1;
	Vec3 p0, p1, v0, v1;

	// ** Setup Scene ** //
	switch (testCase)
	{
		// Setup for Demos 1-3 (Table 1.1)
	case 0: case 1: case 2:
		// * Points * //
		p0 = Vec3(0, 0, 0);
		v0 = Vec3(-1, 0, 0);
		p1 = Vec3(0, 2, 0);
		v1 = Vec3(1, 0, 0);
		m0 = 10, m1 = 10;

		// The point objects:
		pt0 = new MassPoint(p0, v0, Vec3(0, 0, 0), m0, false);
		pt1 = new MassPoint(p1, v1, Vec3(0, 0, 0), m1, false);

		// Push points onto the attribute
		for (auto p : m_points) { delete p; }
		m_points.clear();
		m_points.push_back(pt0);
		m_points.push_back(pt1);

		// * Spring * //
		// The spring object:
		s = Spring(40, 1, 0, 0, 1);

		// Push points onto the attribute
		m_springs.clear();
		m_springs.push_back(s);

		break;

		// Setup for Demo 3,4,5
	case 3: case 4: case 5:
	{
		// Construct a "sheet" of points connected with springs,
		// with a small upward velocity
		// and two points at the top corners being fixed.

		// Rest length of the springs
		double restLen = 0.1;
		// Initial length of the springs
		double stepSize = 0.1;
		// Number of points: width*height
		int width = 8; int height = 8;

		// * Points * //
		for (auto p : m_points) { delete p; }
		m_points.clear();
		for (double i = 0; i < height; i++)
		{
			for (double j = 0; j < width; j++)
			{
				// i running from from top=0.5 to bottom,
				// j running from left=-0.5 to right,
				// with a slight forward tilt of stepSize/10 per row
				p0 = Vec3(-0.5 + j * stepSize, 0.5 - i * stepSize, i * stepSize / 10);
				// Push point onto the attribute, with a small upward velocity no force;
				// fix the top corners.
				m_points.push_back(new MassPoint(p0, Vec3(0, 1, 0), Vec3(0, 0, 0), m_fMass, i == 0 && (j == 0 || j + 1 == width)));
			}
		}

		// * Springs * //
		m_springs.clear();
		// Horizontal connections
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width - 1; j++)
			{
				s = Spring(m_fStiffness, restLen, m_fDamping, i * width + j, i * width + j + 1);
				m_springs.push_back(s);
			}
		}
		// Vertical connections
		for (int j = 0; j < width; j++)
		{
			for (int i = 0; i < height - 1; i++)
			{
				s = Spring(m_fStiffness, restLen, m_fDamping, i * width + j, (i + 1) * width + j);
				m_springs.push_back(s);
			}
		}
		break;
	}
	default:
		break;
	}
	// END Setup Scene //

	// ** Write out to std output ** //
	switch (testCase)
	{
	case 0:
	{
		cout << "Demo 1 selected.\n\n";

		cout << "Euler: \n";
		makeEulerStep(0.1);
		cout << "position p1: " << m_springs[0].getP1(m_points)->getPosition() << "\n";
		cout << "position p2: " << m_springs[0].getP2(m_points)->getPosition() << "\n";
		cout << "velocity p1: " << m_springs[0].getP1(m_points)->getVelocity() << "\n";
		cout << "velocity p2: " << m_springs[0].getP2(m_points)->getVelocity() << "\n";
		cout << "\n";

		// Reset scene
		for (auto p : m_points) { delete p; }
		m_points.clear();
		pt0 = new MassPoint(p0, v0, Vec3(0, 0, 0), m0, false);
		pt1 = new MassPoint(p1, v1, Vec3(0, 0, 0), m1, false);
		m_points.push_back(pt0);
		m_points.push_back(pt1);

		cout << "Midpoint: \n";
		makeMidpointStep(0.1);
		cout << "position p1: " << m_springs[0].getP1(m_points)->getPosition() << "\n";
		cout << "position p2: " << m_springs[0].getP2(m_points)->getPosition() << "\n";
		cout << "velocity p1: " << m_springs[0].getP1(m_points)->getVelocity() << "\n";
		cout << "velocity p2: " << m_springs[0].getP2(m_points)->getVelocity() << "\n";
		cout << "\n";
	}
	break;

	case 1:
		cout << "Demo 2 selected.\n\n";
		break;

	case 2:
		cout << "Demo 3 selected.\n\n";
		break;

	case 3:
		cout << "Demo 4 (Euler) selected.\n";
		break;

	case 4:
		cout << "Demo 4 (Midpoint) selected.\n\n";
		break;

	case 5:
		cout << "Demo 5 selected.\n\n";
		break;
	default:
		break;
	}
}

/*
In Demo 4, allow the top left corner to be moved.
Based on the mouse movement during a click, add difference vector to the current position.
*/
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 3 || m_iTestCase == 4 || m_iTestCase == 5)
	{
		// Apply the mouse deltas to point[0] (move along cameras view plane)
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
			float inputScale = 0.000015f;
			inputWorld = inputWorld * inputScale;
			// Apply difference vector
			Vec3 curpos = m_points[0]->getPosition();
			m_points[0]->setPosition(curpos + inputWorld);
		}
	}

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 1:
		makeEulerStep(0.005);
		break;
	case 2:
		makeMidpointStep(0.005);
		break;
	case 3:
		addGravity(Vec3(0, -9, 0));
		makeEulerStep(timeStep);
		enforceFloorBoundary();
		break;
	case 4:
		addGravity(Vec3(0, -9, 0));
		makeMidpointStep(timeStep);
		enforceFloorBoundary();
		break;
	case 5:
		addGravity(Vec3(0, -9, 0));
		makeLeapForg(timeStep);
		enforceFloorBoundary();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::addGravity(Vec3 g)
{
	for (size_t i = 0; i < m_points.size(); i++)
	{
		m_points[i]->addAcceleration(g);
	}
}

void MassSpringSystemSimulator::makeEulerStep(float timeStep) {
	//std::cout << "Enter" << std::endl;
	for each (Spring s in m_springs) {
		// compute and update force
		s.applyElasticForceToPoints(m_points);
	}

	for each (MassPoint * p in m_points) {
		if (!p->getFixed()) {
			p->setPosition(p->getPosition() + timeStep * p->getVelocity());
			//std::cout << p->getForce() << " | " << p->getMass() << std::endl;
			p->setVelocity(p->getVelocity() + timeStep * (p->getForce() / p->getMass()));
		}
		p->clearForce();
	}
}

void MassSpringSystemSimulator::makeMidpointStep(float timeStep) {
	for each (MassPoint * p in m_points) {
		p->clearForce();
	}

	vector<MassPoint*> midStatePoints;
	midStatePoints.clear();

	for each (Spring s in m_springs) {
		// compute and update force
		s.applyElasticForceToPoints(m_points);
	}

	for each (MassPoint * p in m_points) {
		Vec3 pos = (p->getPosition() + 0.5 * timeStep * p->getVelocity());
		Vec3 vel = (p->getVelocity() + 0.5 * timeStep * (p->getForce() / p->getMass()));
		bool fi = p->getFixed();
		float ma = p->getMass();

		MassPoint* midPoint = new MassPoint(pos, vel, Vec3(), ma, fi);
		if (m_iTestCase == 4) {
			midPoint->addAcceleration(Vec3(0, -9, 0));
		}

		midStatePoints.push_back(midPoint);
	}

	for each (Spring s in m_springs) {
		// compute and update force
		s.applyElasticForceToPoints(midStatePoints);
	}

	for (size_t idx = 0; idx < m_points.size(); idx++) {
		if (!m_points[idx]->getFixed()) {
			m_points[idx]->setPosition(m_points[idx]->getPosition() + timeStep * midStatePoints[idx]->getVelocity());
			m_points[idx]->setVelocity(m_points[idx]->getVelocity() + timeStep * (midStatePoints[idx]->getForce() / midStatePoints[idx]->getMass()));
		}
		m_points[idx]->clearForce();
	}
	
	midStatePoints.clear();
}

void MassSpringSystemSimulator::makeLeapForg(float timeStep) {
	for each (Spring s in m_springs) {
		// compute and update force
		s.applyElasticForceToPoints(m_points);
	}

	for each (MassPoint * p in m_points) {
		if (!p->getFixed()) {
			p->setVelocity(p->getVelocity() + timeStep * (p->getForce() / p->getMass()));
			p->setPosition(p->getPosition() + timeStep * p->getVelocity());
		}
		p->clearForce();
	}
}

void MassSpringSystemSimulator::enforceFloorBoundary()
{
	for each (MassPoint * p in m_points)
	{
		Vec3 pos = p->getPosition();
		if (pos.y < -0.5) {
			p->setPosition(Vec3(pos.x, -0.5, pos.z));
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

/// *** Specific functions *** ///

// Setters

void MassSpringSystemSimulator::setMass(float m) { m_fMass = m; }

void MassSpringSystemSimulator::setStiffness(float k) { m_fStiffness = k; }

void MassSpringSystemSimulator::setDampingFactor(float d) { m_fDamping = d; }


int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) { return 0; };
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {};

// Getters

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_points.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_points[index]->getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_points[index]->getVelocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {};