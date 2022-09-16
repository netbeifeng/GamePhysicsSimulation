#pragma once
#ifndef BOUNDARYBOX_H
#define BOUNDARYBOX_H

#include <vector>
#include "Simulator.h"
#include "Particle.h"

class BoundaryBox {
public:
	explicit BoundaryBox(float particleRadius, const Vec3& lowerDomainBound, const Vec3& upperDomainBound);

	Vec3& getBottomBoundary() { return m_bottomBoundary; }
	Vec3& getTopBoundary() { return m_topBoundary; }

	void searchNeighbors(std::vector<Vec3>& positions,
		std::vector<std::vector<long>>& neighborIdxList,
		std::vector<std::vector<Vec3>>& relPositionList);

	void findBoundaryNeighbors(Vec3 pos, std::vector<Vec3>& relPositionList);

	bool collideWithBoundary(Vec3& particlePosition, Vec3& particleVelocity, float restitution);

	void setTopBoundary(Vec3& top) {
		m_topBoundary.x = top.x;
		m_topBoundary.y = top.y;
		m_topBoundary.z = top.z;
	}

	void setBottomBoundary(Vec3& bottom) {
		m_bottomBoundary.x = bottom.x;
		m_bottomBoundary.y = bottom.y;
		m_bottomBoundary.z = bottom.z;
	}
private:
	void generateGhostFloorParticles();
	void initGridandCells(std::vector<Vec3>& positions);
	void initGrid(std::vector<Vec3>& positions);

	bool isValidIndex(int idx, int dimension) {
		return idx >= 0 && int(idx) < m_gridSize[dimension];
	}

	Vec3 getCellIndex(const Vec3& pos) {
		Vec3 cellIdx = Vec3(0, 0, 0);
		for (int i = 0; i != 3; ++i) {
			cellIdx[i] = int((pos[i] - m_lowerGridBound[i]) / m_cellLength);
		}
		return cellIdx;
	}

	int getOneDimensionId(int i, int j, int k) {
		return int(i) + int(j) * m_gridSize[0] + int(k) * m_gridSize[0] * m_gridSize[1];
	}

	float squareVec3(Vec3 v) {
		return v.x * v.x + v.y * v.y + v.z * v.z;
	}

	std::vector<std::vector<int>> m_cells;
	std::vector<Vec3> m_boundaryParticles;

	Vec3 m_bottomBoundary, m_topBoundary;
	Vec3 m_lowerGridBound;
	Vec3 m_upperGridBound;
	int m_gridSize[3]{ 1,1,1 };
	float m_velLossPercentage;
	float m_cellLength;
	float m_searchDistance;
	float m_particleRadius;
	float m_overlappingDistance;
};


#endif
