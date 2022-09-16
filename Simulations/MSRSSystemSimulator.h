#pragma once
#ifndef MSRSSYSTEMSIMULATOR_h
#define MSRSSYSTEMSIMULATOR_h
#include "MassSpringSystemSimulator.h"
#include "RigidBodySystemSimulator.h"
#include "Simulator.h"


class MSRSSystemSimulator : public Simulator {
public:
	MSRSSystemSimulator();

	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

private:
	//float m_Bounciness = 1.f; // fully plastic 1.f for elastic
	//float m_gravity = 9.8f;

	MassSpringSystemSimulator mss_simulator;
	RigidBodySystemSimulator rbs_simulator;


	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};

#endif