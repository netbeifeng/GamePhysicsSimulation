#include "BoundaryBox.h"
#include <random>
#include "parallel_util.hpp"

BoundaryBox::BoundaryBox(float particleRadius, const Vec3& lowerCorner, const Vec3& upperCorner) :
	m_bottomBoundary{ lowerCorner }, 
	m_topBoundary{ upperCorner },
	m_cellLength{ particleRadius * 4.0f },
	m_searchDistance{ pow(particleRadius * 4.0f, 2.f) },
	m_particleRadius{ particleRadius },
	m_velLossPercentage { .4f },
	m_overlappingDistance{ pow(particleRadius * .0001f , 2.f) }
{
	generateGhostFloorParticles();
}

void BoundaryBox::searchNeighbors(std::vector<Vec3>& positions,
	std::vector<std::vector<long>>& neighbors,
	std::vector<std::vector<Vec3>>& relPositions)
{
	// Assign Cell index to for particles
	initGridandCells(positions);

	const float lowerX = m_bottomBoundary.x + 2.0f * m_particleRadius;
	const float upperX = m_topBoundary.x - 2.0f * m_particleRadius;
	const float upperY = m_topBoundary.y - 2.0f * m_particleRadius;
	const float lowerY = m_bottomBoundary.y + 2.0f * m_particleRadius;
	const float lowerZ = m_bottomBoundary.z + 2.0f * m_particleRadius;
	const float upperZ = m_topBoundary.z - 2.0f * m_particleRadius;

	parallelutil::parallel_for(positions.size(), [&](unsigned long p) {
		Vec3 pos = positions[p];
		std::vector<long>& neighborList = neighbors[p];
		std::vector<Vec3>& relPositionList = relPositions[p];

		neighborList.clear();
		relPositionList.clear();

		Vec3 cellIdx = getCellIndex(pos);

		// scan in 3*3 grid cells
		// for 9 neighbor cell area

		for (int k = -1; k <= 1; ++k) {
			int zIdx = cellIdx.z + k;
			if (!isValidIndex(zIdx, 2)) continue;

			for (int j = -1; j <= 1; ++j) {
				int yIdx = cellIdx.y + j;
				if (!isValidIndex(yIdx, 1)) continue;

				for (int i = -1; i <= 1; ++i) {
					int xIdx = cellIdx.x + i;
					if (!isValidIndex(xIdx, 0)) continue;

					std::vector<int>& cell = m_cells[getOneDimensionId(xIdx, yIdx, zIdx)];

					for (int q : cell) {
						/* Exclude particle p from its neighbor list */
						if (int(p) == q) continue;

						const Vec3 qpos = positions[q];
						const Vec3 r = pos - qpos;
						const float l2 = squareVec3(r);
						if (l2 < m_searchDistance && l2 > m_overlappingDistance) {
							neighborList.push_back(q);
							relPositionList.push_back(r);
						}
					}
				}
			}
		}

		// If a particle is closed
		// to the boundary, compute the relative position with the boundary
		// particles. 
		findBoundaryNeighbors(pos, relPositionList);
	});
}

void BoundaryBox::findBoundaryNeighbors(Vec3 pos, std::vector<Vec3>& relPositionList) {
	const float lowerX = m_bottomBoundary.x + 2.0f * m_particleRadius;
	const float upperX = m_topBoundary.x - 2.0f * m_particleRadius;
	const float upperY = m_topBoundary.y - 2.0f * m_particleRadius;
	const float lowerY = m_bottomBoundary.y + 2.0f * m_particleRadius;
	const float lowerZ = m_bottomBoundary.z + 2.0f * m_particleRadius;
	const float upperZ = m_topBoundary.z - 2.0f * m_particleRadius;

	// floor
	if (pos.y < lowerY) {
		Vec3 transPos = pos - Vec3(
			m_cellLength * floor(pos.x / m_cellLength),
			0.0f,
			m_cellLength * floor(pos.z / m_cellLength)
		);

		float y = m_bottomBoundary.y - 2.0f * m_particleRadius;

		for (Vec3& bdpos : m_boundaryParticles) {
			Vec3 r = transPos - Vec3(bdpos.x, y + bdpos.y, bdpos.z);
			float l2 = squareVec3(r);
			// no need to add indices to neighbor lists, because ghost
			if (l2 < m_searchDistance && l2 > m_overlappingDistance)
				relPositionList.push_back(r);
		}
	}

	// left and right plane
	if (pos.x < lowerX || pos.x > upperX) {
		const Vec3 fixedPos = pos - Vec3(
			0.0f,
			m_cellLength * floor(pos.y / m_cellLength),
			m_cellLength * floor(pos.z / m_cellLength)
		);

		const float x = pos.x < lowerX ?
			m_bottomBoundary.x - 2.0f * m_particleRadius :
			m_topBoundary.x + 2.0f * m_particleRadius;

		for (const Vec3& bdpos : m_boundaryParticles) {
			const Vec3 r = fixedPos - Vec3(x + bdpos.x, bdpos.y, bdpos.z);
			const float l2 = squareVec3(r);
			if (l2 < m_searchDistance && l2 > m_overlappingDistance)
				relPositionList.push_back(r);
		}
	}

	// near and far
	if (pos.z < lowerZ || pos.z > upperZ) {
		const Vec3 fixedPos = pos - Vec3(
			m_cellLength * floor(pos.x / m_cellLength),
			m_cellLength * floor(pos.y / m_cellLength),
			0.0f);
		const float z = pos.z < lowerZ ?
			m_bottomBoundary.z - 2.0f * m_particleRadius :
			m_topBoundary.z + 2.0f * m_particleRadius;

		for (const Vec3& bdpos : m_boundaryParticles) {
			const Vec3 r = fixedPos - Vec3(bdpos.x, bdpos.y, z + bdpos.z);
			const Vec3 l2 = squareVec3(r);
			if (l2 < m_searchDistance && l2 > m_overlappingDistance)
				relPositionList.push_back(r);
		}
	}

}

void BoundaryBox::generateGhostFloorParticles() {
	// Generate 1 layer of ghots boundary particles as floor make floor density more?
	float spacing = m_particleRadius * 4;
	Vec3 base = Vec3(-m_cellLength + spacing, -m_cellLength + spacing, 0);

	std::random_device rd;
	std::mt19937 gen(rd());

	// jitter pos get more randomized result
	std::uniform_real_distribution<float> distr(-m_particleRadius, m_particleRadius);

	for (int l1 = 0; l1 < 10; ++l1) {
		for (int l2 = 0; l2 < 10; ++l2) {
			Vec3 pos2D = base + Vec3(l1, l2, 0) * spacing;
			if (abs(pos2D.x) > 0.5f) {
				pos2D.x = 0.5f;
			} 

			if (abs(pos2D.z) > 0.5f) {
				pos2D.z = 0.5f;
			}

			Vec3 pos3D = Vec3(
				pos2D[0] + distr(gen) * 0.2f, /* Store jittered position */
				pos2D[1] + distr(gen) * 0.2f, /* Store jittered position */
				pos2D[2] + distr(gen) * 0.1f);
			std::cout << pos3D << std::endl;
			m_boundaryParticles.push_back(pos3D);
		}
	}
}

void BoundaryBox::initGridandCells(std::vector<Vec3>& positions) {
	if (positions.size() == 0) {
		return;
	}

	// initialize Grid to correct size
	initGrid(positions);

	// for each particle, get Cell ID of it, stored in Vec3 id in 3 achsen
	for (int i = 0; i < positions.size(); ++i) {
		Vec3 cellId = getCellIndex(positions[i]);
		// transfer 3D id (1,2,3) => to one dimension 1 + size.x * 2 + size.x * size.y * 3
		int oneDimensionId = getOneDimensionId(cellId.x, cellId.y, cellId.z);
		m_cells[oneDimensionId].push_back(i);
	}
}


void BoundaryBox::initGrid(std::vector<Vec3>& positions) {
	if (positions.size() == 0) {
		return;
	}

	// Get the real upper and lower bound, avoid outlier cause break
	Vec3 lowerBound = positions[0];
	Vec3 upperBound = positions[0];
	for (int p = 1; p < positions.size(); ++p) {
		Vec3 ppos = positions[p];
		for (int i = 0; i < 3; ++i) {
			lowerBound[i] = std::min(lowerBound[i], ppos[i]);
			upperBound[i] = std::max(upperBound[i], ppos[i]);
		}
	}

	m_lowerGridBound = lowerBound;
	m_upperGridBound = upperBound;

	int numCells = 1;
	for (std::size_t i = 0; i != 3; ++i) {
		m_gridSize[i] = int(ceil((upperBound[i] - lowerBound[i]) / m_cellLength));
		// std::cout << m_gridSize[i] << "\n"; 5 10 5
		if (m_gridSize[i] == 0) {
			m_gridSize[i] = 1;
		}
		numCells *= m_gridSize[i];
	}
	// 5 * 10 * 5 = 250 cells
	m_cells.resize(numCells);

	// clean up the indices stored in cells, old
	parallelutil::parallel_for(m_cells.size(), [&](int idx) {
		m_cells[idx].resize(0);
	});
}

bool BoundaryBox::collideWithBoundary(Vec3& position, Vec3& velocity, float backPercent) {
	/* Enforce particle position to be within the given boundary */
	bool hasCollision = false;
	for (std::size_t i = 0; i != 3; ++i) {
		if (position[i] < m_bottomBoundary[i]) {
			position[i] += (m_bottomBoundary[i] - position[i]) * (backPercent + 1.0f);
			velocity[i] *= - (m_velLossPercentage);
			hasCollision = true;
		}
		else if (position[i] > m_topBoundary[i]) {
			position[i] -= (position[i] - m_topBoundary[i]) * (backPercent + 1.0f);
			velocity[i] *= - (m_velLossPercentage);
			hasCollision = true;
		}
	}

	return hasCollision;
}
