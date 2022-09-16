#include "RigidBodySystemSimulator.h"
#include <math.h>
#include "collisionDetect.h"
#define _USE_MATH_DEFINES

#define DEG2RAD (M_PI * 2)/360.f

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_rigidBodyList.clear();
	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };

	// intialize
	m_Bounciness = .7;
	m_floorAndWalls.clear();
	m_rigidBodyList.clear();
}

/// *** UI functions *** ///

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	// add UI element for tuning Bounciness (c)
	if (m_iTestCase == 2 || m_iTestCase == 3) {
		TwAddVarRW(DUC->g_pTweakBar, "Bounciness?", TW_TYPE_FLOAT, &m_Bounciness, "min=0.0 max=1.0 step=0.01");
	}
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	// reset variables
	m_Bounciness = 0.7;
	m_floorAndWalls.clear();
	m_rigidBodyList.clear();
}

// get the norm of an arbitary vector
float RigidBodySystemSimulator::getNormOfVector(Vec3 v) {
	return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	//std::cout << m_rigidBodyList.size() << std::endl;

	for each (RigidBody * rb in m_rigidBodyList) {
		//std::cout << "Involved 0" << std::endl;
		drawRigidBodyFrame(rb, true);
	}
	//drawRigidBodyFrame(m_floorAndWalls[m_floorAndWalls.size() - 1], true);
	//if (m_iTestCase == 3) {
		drawFloorAndWall();
	//}
}

void RigidBodySystemSimulator::addRigidBody_ex4(Vec3 pos, Vec3 size, int mass) {
	addRigidBody(pos, size, mass);
}

void RigidBodySystemSimulator::initFrameObjects() {
	//std::cout << "Involved 1" << std::endl;
	m_floorAndWalls.clear();
	m_rigidBodyList.clear();
	//std::cout << m_rigidBodyList.size() << std::endl;
	addRigidBody(Vec3(0, -0.25, 0), Vec3(0.5, 0.5, 0.5), 2);
	applyForceOnBody(0, Vec3(0.25, 0.25, 0.25), Vec3(0, 0, 0));
	setOrientationOf(0, Quat(0, 0, 0));

	//std::cout << m_rigidBodyList[0]->getColor() << std::endl;
	setUpFloorAndWalls();
	//std::cout << m_rigidBodyList.size() << std::endl;
}

void RigidBodySystemSimulator::addGravity(Vec3 g) {

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	// when the case is going to be changed, reset it.
	reset();
	
	m_iTestCase = testCase;

	// ** Setup Scene ** //
	switch (testCase)
	{
	case 0: {
		cout << "Demo 1 selected.\n\n";
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0, 0, M_PI / 2));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));

		// single step integration with h = 2
		integrateAll(2.f);

		std::cout << "Position  (-0.3, -0.5, -0.25) \n" << std::endl;

		Vec3 pos = Vec3(-0.3, -0.5, -0.25);
		Vec3 linearVelocity = getLinearVelocityOfRigidBody(0);
		Vec3 angularVelocity = getAngularVelocityOfRigidBody(0);

		std::cout << "Velocity: " << linearVelocity + cross(angularVelocity, pos) << std::endl;
		std::cout << "Linear Velocity: " << linearVelocity << std::endl;
		std::cout << "Angular Velocity: " << angularVelocity << std::endl;
		break;
	}
	case 1: {
		cout << "Demo 2 selected.\n\n";
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0, 0, M_PI / 2));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	}
	case 2: {
		// two rigid bodies collision
		cout << "Demo 3 selected.\n\n";
		addRigidBody(Vec3(-0.25, -0.5, 0.1), Vec3(0.5, 0.5, 0.5), 2);
		addRigidBody(Vec3(0.25, 0.5, -0.1), Vec3(0.5, 0.5, 0.5), 2);

		setOrientationOf(0, Quat(0, 0, 0));
		setOrientationOf(1, Quat(0, 0, 0));

		applyForceOnBody(0, getRigidBodyByIdx(0)->getCenterPosition(), Vec3(0.8f, 1.4f, 0.2f));
		applyForceOnBody(1, getRigidBodyByIdx(1)->getCenterPosition(), Vec3(-0.3f, -1, 0));

		break;
	}
	case 3:
	{
		// set up walls which are also 4 rigid bodies but they are fixed and has an infinite mass
		setUpFloorAndWalls();

		addRigidBody(Vec3(-0.25, 0.25, 0), Vec3(1, 0.6, 0.5), 5);
		setOrientationOf(0, Quat(0, 0, M_PI / 2));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(3.0f, 0.0f, 0.0f));

		addRigidBody(Vec3(1, 0.25, 1), Vec3(0.7, 0.6, 0.5), 3);
		setOrientationOf(1, Quat(0, 0, 0));
		applyForceOnBody(1, Vec3(0.3f, 0.5f, 0.25f), Vec3(-2.3f, 0.0f, 0.0f));

		addRigidBody(Vec3(0.25, 1,-1), Vec3(0.5, 0.5, 0.5), 2);
		setOrientationOf(2, Quat(0, 0, 0));
		applyForceOnBody(2, Vec3(0, 0, 0), Vec3(0.0f, -0.9f, 0.0f));


		addRigidBody(Vec3(0.25, 0.1, -1), Vec3(0.5, 0.5, 0.5), 2);
		setOrientationOf(3, Quat(0, 0, 0));
		applyForceOnBody(3, Vec3(0, 0, 0), Vec3(0.0f, 1.8f, 0.0f));
		cout << "Demo 4 selected.\n\n";
		break;
	}
	default:
		break;
	}
	// END Setup Scene //
}


void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {

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

			// interaction only for demo 2 and demo 4
			if (m_iTestCase == 1)
			{
				m_rigidBodyList[0]->applyForce(Vec3(0, 0, 0.01f));
				m_rigidBodyList[0]->applyForceLoc(getRigidBodyByIdx(0)->getCenterPosition());
			}
			else if (m_iTestCase == 3) {
				m_rigidBodyList[getNumberOfRigidBodies() - 1]->updateCenterPosition(inputWorld);
			}

		}

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{

	switch (m_iTestCase)
	{
		case 0: break;
		case 1:
			integrateAll(0.01f);
			break;
		case 2:
			integrateAll(timeStep * 0.1f);
			checkCollision();
			break;
		case 3:
			// timeStep has to be a very big number, otherwise bodies move extremly slow
			integrateAll(timeStep);
			checkCollision();
			break;
		default: break;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	if (mass > 99999) {
		RigidBody* rb = new RigidBody(position, size, mass);
		rb->setAsFixed();
		m_rigidBodyList.push_back(rb);
	}
	else {
		m_rigidBodyList.push_back(new RigidBody(position, size, mass));
	}
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RigidBody* toBeUpdatedRB = getRigidBodyByIdx(i);
	if (toBeUpdatedRB != nullptr) {
		toBeUpdatedRB->applyForce(force);       // add force
		toBeUpdatedRB->applyForceLoc(loc);      // set force loc
	}
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	RigidBody* toBeUpdatedRB = getRigidBodyByIdx(i);
	if (toBeUpdatedRB != nullptr) {
		toBeUpdatedRB->setUpRotation(orientation);
		toBeUpdatedRB->computeWorldMatrix();
	}
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_rigidBodyList.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	RigidBody* fetchedRB = getRigidBodyByIdx(i);
	if (fetchedRB != nullptr) {
		return fetchedRB->getCenterPosition();
	}
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	RigidBody* fetchedRB = getRigidBodyByIdx(i);
	if (fetchedRB != nullptr) {
		return fetchedRB->getAngularVelocity();
	}
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	RigidBody* fetchedRB = getRigidBodyByIdx(i);
	if (fetchedRB != nullptr) {
		return fetchedRB->getLinearVelocity();
	}
	return Vec3();
}


RigidBody* RigidBodySystemSimulator::getRigidBodyByIdx(int idx)
{
	if (idx > m_rigidBodyList.size() || idx < 0) {
		string error_message = "Invalid index " + std::to_string(idx) + ", out of range [ 0 , " + std::to_string(m_rigidBodyList.size()) + " ]";
		throw std::invalid_argument(error_message);
		return nullptr; // if not found, return null pointer and check in each concrete functions
	}
	return m_rigidBodyList[idx]; // else return the fetched pointer
}

// Integration Method, for all rigid bodies in the rigidBody list will be integrated
void RigidBodySystemSimulator::integrateAll(float timeStep) {
	for each (RigidBody * rb in m_rigidBodyList) {
		rb->integrate(timeStep);
	}
}

// check whether there is a collision occur
void RigidBodySystemSimulator::checkCollision() {
	int rigidBodiesNumber = getNumberOfRigidBodies();
	
	for (int idx = 0; idx < rigidBodiesNumber; idx++) {
		RigidBody* rigidBody1 = getRigidBodyByIdx(idx);

		// check with other rigid bodies
		for (int idx_s = idx + 1; idx_s < rigidBodiesNumber; idx_s++) {
			RigidBody* rigidBody2 = getRigidBodyByIdx(idx_s);
			calculateCollision(rigidBody1 ,rigidBody2);
		}

		// check with floors and walls
		for (int idx_w = 0; idx_w < m_floorAndWalls.size(); idx_w++) {
			RigidBody* wall = m_floorAndWalls[idx_w];
			calculateCollision(rigidBody1, wall);
		}
	}
}
vector<RigidBody*> RigidBodySystemSimulator::getWalls() {
	return m_floorAndWalls;
}
// calculate the impulse and update the velocity and position
void RigidBodySystemSimulator::calculateCollision(RigidBody* rigidBody1, RigidBody* rigidBody2) {
	CollisionInfo collisionInfo = checkCollisionSAT(rigidBody1->getToWorldMatrix(), rigidBody2->getToWorldMatrix());
	if (collisionInfo.isValid && (!rigidBody1->isFixed() || !rigidBody2->isFixed())) {
		std::cout << "Collide!" << std::endl;
		Vec3 normal = collisionInfo.normalWorld;
		Vec3 collisionPoint = collisionInfo.collisionPointWorld;

		// Resolve the penetration 
		Vec3 penetration = (normal * collisionInfo.depth) / 2;
		rigidBody1->updateCenterPosition(penetration);
		rigidBody2->updateCenterPosition(-penetration);

		Vec3 collisionPositionRigidBody1 = collisionPoint - rigidBody1->getCenterPosition();
		Vec3 collisionPositionRigidBody2 = collisionPoint - rigidBody2->getCenterPosition();

		Vec3 collisionVelocityRigidBody1 = rigidBody1->getLinearVelocity() + cross(rigidBody1->getAngularVelocity(), collisionPositionRigidBody1);
		Vec3 collisionVelocityRigidBody2 = rigidBody2->getLinearVelocity() + cross(rigidBody2->getAngularVelocity(), collisionPositionRigidBody2);

		// Refer to Slide 32 - RigidBody 2D => 
		// https://www.moodle.tum.de/pluginfile.php/3399652/mod_resource/content/0/lecture03-rigid-bodies-2D.pdf

		float relativeVelocityDotNormal = dot(normal, (collisionVelocityRigidBody1 - collisionVelocityRigidBody2));

		double impulseCalculationPart1 = -(1.f + m_Bounciness) * relativeVelocityDotNormal;

		double impulseCalculationPart2 = ((1.f / rigidBody1->getMass()) + (1.f / rigidBody2->getMass()));

		Vec3 impulseCalculationPart3RigidBody1 = cross((rigidBody1->getInertiaTensorInv() * cross(collisionPositionRigidBody1, normal)), collisionPositionRigidBody1);
		Vec3 impulseCalculationPart3RigidBody2 = cross((rigidBody2->getInertiaTensorInv() * cross(collisionPositionRigidBody2, normal)), collisionPositionRigidBody2);

		double impulseCalculationPart3 = dot((impulseCalculationPart3RigidBody1 + impulseCalculationPart3RigidBody2), normal);

		double impulse = impulseCalculationPart1 / (impulseCalculationPart2 + impulseCalculationPart3);

		//std::cout << "Impulse: " << impulse << std::endl;

		if (rigidBody1->isFixed()) { // only update RigidBody2
			//std::cout << "Updating RigidBody 2" << std::endl;
			rigidBody2->updateLinearVelocity((impulse * normal) / rigidBody2->getMass());
			rigidBody2->updateAngularMomentum(cross(collisionPositionRigidBody2, impulse * normal));
		}
		else if (rigidBody2->isFixed()) { // only update RigidBody1
			//std::cout << "Updating RigidBody 1" << std::endl;
			rigidBody1->updateLinearVelocity((impulse * normal) / rigidBody1->getMass());
			rigidBody1->updateAngularMomentum(cross(collisionPositionRigidBody1, impulse * normal));
		}

		// update both
		if (!rigidBody1->isFixed() && !rigidBody2->isFixed()) {
			//std::cout << "Both" << std::endl;
			rigidBody1->updateLinearVelocity((impulse * normal) / rigidBody1->getMass());
			rigidBody1->updateAngularMomentum(cross(collisionPositionRigidBody1, impulse * normal));
			rigidBody2->updateLinearVelocity(-(impulse * normal) / rigidBody2->getMass());
			rigidBody2->updateAngularMomentum(-cross(collisionPositionRigidBody2, impulse * normal));
		}
	}
}

// draw rigid body and also a wireframe for identifying the rigid body boundary
void RigidBodySystemSimulator::drawRigidBodyFrame(RigidBody* rb, bool drawBody) {
	// Body 
	if (drawBody) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, rb->getColor());
		DUC->drawRigidBody(rb->getToWorldMatrix());
	}

	// Wireframe
	Vec3 p1 = rb->getVertexInWorldByIdx(0);
	Vec3 p2 = rb->getVertexInWorldByIdx(1);
	Vec3 p3 = rb->getVertexInWorldByIdx(2);
	Vec3 p4 = rb->getVertexInWorldByIdx(3);
	Vec3 p5 = rb->getVertexInWorldByIdx(4);
	Vec3 p6 = rb->getVertexInWorldByIdx(5);
	Vec3 p7 = rb->getVertexInWorldByIdx(6);
	Vec3 p8 = rb->getVertexInWorldByIdx(7);

	DUC->beginLine();
	DUC->drawLine(p1, Vec3(1, 1, 1), p2, Vec3(1, 1, 1));
	DUC->drawLine(p1, Vec3(1, 1, 1), p4, Vec3(1, 1, 1));
	DUC->drawLine(p1, Vec3(1, 1, 1), p5, Vec3(1, 1, 1));

	DUC->drawLine(p3, Vec3(1, 1, 1), p2, Vec3(1, 1, 1));
	DUC->drawLine(p3, Vec3(1, 1, 1), p4, Vec3(1, 1, 1));
	DUC->drawLine(p3, Vec3(1, 1, 1), p7, Vec3(1, 1, 1));

	DUC->drawLine(p6, Vec3(1, 1, 1), p2, Vec3(1, 1, 1));
	DUC->drawLine(p6, Vec3(1, 1, 1), p5, Vec3(1, 1, 1));
	DUC->drawLine(p6, Vec3(1, 1, 1), p7, Vec3(1, 1, 1));

	DUC->drawLine(p8, Vec3(1, 1, 1), p4, Vec3(1, 1, 1));
	DUC->drawLine(p8, Vec3(1, 1, 1), p5, Vec3(1, 1, 1));
	DUC->drawLine(p8, Vec3(1, 1, 1), p7, Vec3(1, 1, 1));
	DUC->endLine();

	// Vertices
	for (Vec3 vertex : rb->getVertices()) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(1, 1, 1));
		DUC->drawSphere(vertex, 0.0005f);
	}
}
void RigidBodySystemSimulator::addImmo(Vec3 pos, Vec3 size) {
	RigidBody* immo = new RigidBody(pos, size, INFINITY);

	m_rigidBodyList.push_back(new RigidBody(pos, size, INFINITY));
	immo->setUpRotation(Quat(0, 0, 0));
	immo->computeWorldMatrix();
}
// build boundary walls
void RigidBodySystemSimulator::setUpFloorAndWalls() {
	
	m_floorAndWalls.clear();

	RigidBody* floor = new RigidBody(Vec3(0, -0.5f, 0), Vec3(3, 0.01f, 3), INFINITY);
	RigidBody* ceil = new RigidBody(Vec3(0, 2.5f, 0), Vec3(3, 0.01f, 3), INFINITY);

	RigidBody* leftWall = new RigidBody(Vec3(-1.5, 1, 0), Vec3(0.01f, 3, 3), INFINITY);
	RigidBody* rightWall = new RigidBody(Vec3(1.5, 1, 0), Vec3(0.01f, 3, 3), INFINITY);

	RigidBody* nearWall = new RigidBody(Vec3(0, 1, -1.5), Vec3(3, 3, 0.01f), INFINITY);
	RigidBody* farWall = new RigidBody(Vec3(0, 1, 1.5), Vec3(3, 3, 0.01f), INFINITY);

	m_floorAndWalls.push_back(floor);
	m_floorAndWalls.push_back(ceil);

	m_floorAndWalls.push_back(leftWall);
	m_floorAndWalls.push_back(rightWall);

	m_floorAndWalls.push_back(nearWall);
	m_floorAndWalls.push_back(farWall);

	floor->setUpRotation(Quat(0, 0, 0));
	ceil->setUpRotation(Quat(0, 0, 0));

	leftWall->setUpRotation(Quat(0, 0, 0));
	rightWall->setUpRotation(Quat(0, 0, 0));

	nearWall->setUpRotation(Quat(0, 0, 0));
	farWall->setUpRotation(Quat(0, 0, 0));

	floor->computeWorldMatrix();
	ceil->computeWorldMatrix();

	leftWall->computeWorldMatrix();
	rightWall->computeWorldMatrix();

	nearWall->computeWorldMatrix();
	farWall->computeWorldMatrix();

	floor->setAsFixed();
	ceil->setAsFixed();

	leftWall->setAsFixed();
	rightWall->setAsFixed();

	nearWall->setAsFixed();
	farWall->setAsFixed();
}

// draw walls
void RigidBodySystemSimulator::drawFloorAndWall() {
	for each (RigidBody * rb in m_floorAndWalls) {
		drawRigidBodyFrame(rb, false);
	}
}