#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "RigidBody.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Original Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Drawers
	void drawRigidBodyFrame(RigidBody* rb, bool drawBody);
	void drawFloorAndWall();
	
	// Getters
	int getNumberOfRigidBodies();
	RigidBody* getRigidBodyByIdx(int idx);
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);

	// Set Up Functions
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setUpFloorAndWalls();

	// Integration
	void integrateAll(float timeStep);

	// Collision Detection
	void checkCollision();
	void calculateCollision(RigidBody* rigidBody1, RigidBody* rigidBody2);

	// Helper functions
	float getNormOfVector(Vec3 v);

	// for ex4
	void initFrameObjects();
	void addGravity(Vec3 gravity);
	void addRigidBody_ex4(Vec3 pos, Vec3 size, int mass);
	void addImmo(Vec3 pos, Vec3 size);
	vector<RigidBody*> getWalls();
private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 

	float m_Bounciness = 0; // fully plastic 1.f for elastic

	vector<RigidBody*> m_rigidBodyList; // rigid body list
	vector<RigidBody*> m_floorAndWalls; // wall list

	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif