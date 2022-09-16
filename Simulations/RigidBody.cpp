#include "RigidBody.h"

RigidBody::RigidBody() {}

RigidBody::RigidBody(Vec3 center_position, Vec3 size, float mass) {
	m_centerPosition = center_position;
	m_size = size;
	m_mass = mass;

	m_linearVelocity = Vec3();
	m_angularVelocity = Vec3();
	m_angularMomentum = Vec3();
	m_force = Vec3();
	m_forceLoc = Vec3();
	m_torque = Vec3();
	m_inertiaTensor = Mat4();
	m_inertiaTensor0 = Mat4();

	m_isFixed = false;
	m_toWorld = Mat4();
	m_vertices.clear();

	srand(time(nullptr) + rand());
	m_color = Vec3(((float)rand()) / (float)RAND_MAX, ((float)rand()) / (float)RAND_MAX, ((float)rand()) / (float)RAND_MAX);

	preComputeInertiaTensor0();
}

Vec3 RigidBody::getColor() {
	return m_color;
}

Vec3 RigidBody::getSize() {
	return m_size;
}

Quat RigidBody::getRotation() {
	return m_rotation;
}

float RigidBody::getMass() {
	return m_mass;
}

Mat4 RigidBody::getToWorldMatrix() {
	return m_toWorld;
}

Vec3 RigidBody::getAngularVelocity() {
	return m_angularVelocity;
}

Vec3 RigidBody::getLinearVelocity() {
	return m_linearVelocity;
}

Vec3 RigidBody::getCenterPosition() {
	return m_centerPosition;
}

Mat4 RigidBody::getInertiaTensorInv() {
	return m_inertiaTensor;
}

// get the world matrix for drawing
void RigidBody::computeWorldMatrix() {
	Mat4 translation;
	Mat4 scale;
	Mat4 rotation;

	translation.initTranslation(m_centerPosition.x, m_centerPosition.y, m_centerPosition.z);
	scale.initScaling(m_size.x, m_size.y, m_size.z);
	rotation = m_rotation.getRotMat();

	// ScaleFac * Rot * Trans
	m_toWorld = scale * rotation * translation;
	updateVertexOfWorld();
}

void RigidBody::preComputeInertiaTensor0() {
	Mat4 tmpInetriaTensor;
	// Refer to the formula on Wikipedia https://en.wikipedia.org/wiki/List_of_moments_of_inertia

	tmpInetriaTensor.initId();
	tmpInetriaTensor.value[0][0] = (1.f / 12.f) * m_mass * (pow(m_size.y, 2) + pow(m_size.z, 2));
	tmpInetriaTensor.value[1][1] = (1.f / 12.f) * m_mass * (pow(m_size.x, 2) + pow(m_size.z, 2));
	tmpInetriaTensor.value[2][2] = (1.f / 12.f) * m_mass * (pow(m_size.x, 2) + pow(m_size.y, 2));
	tmpInetriaTensor.inverse();

	m_inertiaTensor0 = tmpInetriaTensor;
}

void RigidBody::setUpRotation(Quat rotation) {
	m_rotation = rotation;
}

void RigidBody::applyForce(Vec3 force) {
	m_force += force;
}

void RigidBody::applyForceLoc(Vec3 loc) {
	m_forceLoc = loc;
}

void RigidBody::calculateTorque(Vec3 force, Vec3 loc) {
	m_torque += cross(loc - m_centerPosition, force);
}

void RigidBody::updateCenterPosition(Vec3 pos) {
	m_centerPosition += pos;
}

void RigidBody::setAngularVelocity(Vec3 av) {
	m_angularVelocity = av;
}

void RigidBody::updateLinearVelocity(Vec3 lv) {
	m_linearVelocity += lv;
}

void RigidBody::updateRotation(Quat r) {
	m_rotation += r;
	m_rotation /= m_rotation.norm(); // normalization
}

void RigidBody::updateAngularMomentum(Vec3 am) {
	m_angularMomentum += am;
}

void RigidBody::setInetriaTensor(Mat4 it) {
	m_inertiaTensor = it;
}

void RigidBody::integrate(float timeStep) {
	// Refer to lecture05-rigid-bodies-3D.pdf Slide 14
	if (!m_isFixed) {
		m_force += Vec3(0, -9.8f, 0);
		calculateTorque(m_force, m_forceLoc);
		updateCenterPosition(timeStep * m_linearVelocity); // update pos
		updateLinearVelocity(timeStep * (m_force / m_mass)); // update linear velocity

		// Integration Rotation
		Quat angularVelocityQuat = Quat(m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z, 0);
		updateRotation(0.5 * timeStep * angularVelocityQuat * m_rotation);

		// Integration Angular Momentum
		updateAngularMomentum(timeStep * m_torque);

		// Integration (Inverse) Inetria Tensor
		Mat4 m_rotationTransposed = m_rotation.getRotMat();
		m_rotationTransposed.transpose(); // return void -_-!! Why? NG!

		setInetriaTensor(m_rotation.getRotMat() * m_inertiaTensor0 * m_rotationTransposed);

		// Integration Angualr Velocity
		setAngularVelocity(m_inertiaTensor * m_angularMomentum);

		computeWorldMatrix(); // calculate the new world matrix
		updateVertexOfWorld(); // update the vertices coordinates

		clearForce();
	}
}

void RigidBody::clearForce() {
	m_force = Vec3();
	m_torque = Vec3();
}

void RigidBody::setAsFixed() {
	m_isFixed = true;
}

void RigidBody::setCenterPosition(Vec3 cm) {
	m_centerPosition = cm;
}

bool RigidBody::isFixed() {
	return m_isFixed;
}

void RigidBody::updateVertexOfWorld() {
	//
	//        1 --------- 4
	//       /|          /|
	//      / |         / |
 	//     /  |        /  |
	//    2 --|------ 3   |
	//    |   5 ------|-- 8
	//    |  /        |  /
	//    | /         | /
	//    |/          |/
	//    6 --------- 7
	//
	//

	m_vertices.clear();
	Vec3 p1 = 0.5 * Vec3(-1, 1, -1);
	Vec3 p2 = 0.5 * Vec3(-1, 1, 1);
	Vec3 p3 = 0.5 * Vec3(1,  1, 1);
	Vec3 p4 = 0.5 * Vec3(1,  1, -1);

	Vec3 p5 = 0.5 * Vec3(-1, -1, -1);
	Vec3 p6 = 0.5 * Vec3(-1, -1, 1);
	Vec3 p7 = 0.5 * Vec3(1,  -1, 1);
	Vec3 p8 = 0.5 * Vec3(1,  -1, -1);

	Mat4 rotation = m_rotation.getRotMat();

	m_vertices.push_back(m_toWorld.transformVector(p1));
	m_vertices.push_back(m_toWorld.transformVector(p2));
	m_vertices.push_back(m_toWorld.transformVector(p3));
	m_vertices.push_back(m_toWorld.transformVector(p4));
	m_vertices.push_back(m_toWorld.transformVector(p5));
	m_vertices.push_back(m_toWorld.transformVector(p6));
	m_vertices.push_back(m_toWorld.transformVector(p7));
	m_vertices.push_back(m_toWorld.transformVector(p8));
}

vector<Vec3> RigidBody::getVertices() {
	return m_vertices;
}

Vec3 RigidBody::getVertexInWorldByIdx(int idx) {
	if (idx < m_vertices.size()) {
		return (m_vertices[idx]);
	}
}