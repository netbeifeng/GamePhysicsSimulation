#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h
#include <stdexcept>
#include "Simulator.h"
#include "vectorbase.h"
//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(unsigned int x, unsigned int y, unsigned int z) {
		// idx starts from zero
		m_width = x;
		m_height = y;
		m_depth = z;

		m_maxTemperature = 1.f;

		m_values = vector<float>();
		m_values.resize(m_width * m_height * m_depth + m_width * m_height + m_width);
	}

	float getValue(unsigned int x, unsigned int y, unsigned int z) {
		unsigned int idx = m_width * m_height * z + m_width * y + x;
		if (idx < 0 || idx >  m_width * m_height * m_depth + m_width * m_height + m_width) {
			return 0;
		}
		return m_values[idx];
	} 

	void setValue(unsigned int x, unsigned int y, unsigned int z, float val) {
		unsigned int idx = m_width * m_height * z + m_width * y + x;
		if (idx < 0 || idx >  m_width * m_height * m_depth + m_width * m_height + m_width) {
			m_values[idx] = 0;

		}
		m_values[idx] = val;
	}

	Vec3 getSize() {
		return Vec3(m_width, m_height, m_depth);
	}

	vector<float> getVectorArray() {
		return m_values;
	}

	float getMaxTemperature() {
		return m_maxTemperature;
	}

	void setMaxTemperature(float maxTemperature) {
		m_maxTemperature = maxTemperature;
	}

	void debugPrint() {
		std::cout << m_width << ", " << m_height << ", " << m_depth << std::endl;
		for (unsigned int zIdx = 0; zIdx < m_depth; zIdx++) {
			std::cout << "Z-Plane index = " << zIdx << std::endl;
			
			for (unsigned int xIdx = 0; xIdx < m_width; xIdx++) {
				std::cout << "  x" << xIdx << "   ";
			}

			std::cout << "\n";

			for (unsigned int yIdx = 0; yIdx < m_height; yIdx++) {
				std::cout << "y" << yIdx;
				for (unsigned int xIdx = 0; xIdx < m_width; xIdx++) {
					unsigned int idx = m_width * m_height * zIdx + m_width * yIdx + xIdx;
					std::cout << "   " << m_values[idx] << "   ";
				}
				std::cout << "\n\n";
			}
			std::cout << "\n";
		}
	}

private:
	// Attributes
	unsigned int m_width = 0;
	unsigned int m_height = 0;
	unsigned int m_depth = 0;
	float m_maxTemperature = 1.f;


	vector<float> m_values = vector<float>();
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects(Grid* grid);
	Grid* diffuseTemperatureExplicit(float timeStep);
	Grid* diffuseTemperatureImplicit(float timeStep);

	bool isBoundary(int xIdx, int yIdx, int zIdx, Vec3 size);

	unsigned int checkOutOfBoundary(Grid* g, unsigned int index);

	void updateDemo1Grid(Grid* grid);

	float getAlpha();
private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step

	unsigned int m_width = 16;  // x
	unsigned int m_height = 16; // y
	unsigned int m_depth = 1;  // z

	bool m_debug = false;


	Grid* m_demo1_baseGrid;
	Grid* m_demo2_baseGrid;

	Grid* m_drawing_grid;

	float m_alpha = 0.003f;
};

#endif