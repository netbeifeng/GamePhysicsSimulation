#ifndef RIGIDBODY_h
#define RIGIDBODY_h
#include "Simulator.h"
struct Point {
	Point(Vec3 pos, float m) {
		this->pos = pos;
		this->m = m;
	};
	Vec3 pos;
	float m;
};

class RigidBody {

public:
	RigidBody();
	RigidBody(Vec3 position, Vec3 size, float mass);
	void preComputeInertiaTensor0(); // pre computing (I0)

	Mat4 getToWorldMatrix(); // get World matrix for drawing rigid body and detecting collision

	
	// Trivial setters, for modifying the corresponding variables
	void setUpRotation(Quat rotation);
	void setCenterPosition(Vec3 cm);

	// Trivial getters
	Vec3 getCenterPosition();
	Vec3 getLinearVelocity();
	Vec3 getAngularVelocity();
	Mat4 getInertiaTensorInv();
	Quat getRotation();
	Vec3 getSize();
	vector<Vec3> getVertices();
	Vec3 getVertexInWorldByIdx(int idx);
	Vec3 getColor();
	float getMass();

	// Force 
	void applyForce(Vec3 force);
	void applyForceLoc(Vec3 loc);
	void calculateTorque(Vec3 force, Vec3 loc); // torque = (Loc - CM) X Force

	// Fixed control
	void setAsFixed();
	bool isFixed();

	// Here after => Integration Methods
	void integrate(float timeStep);

	// Updaters for updates, which rely on those previous value (+=)
	void updateCenterPosition(Vec3 pos); // x' = x + h * lv
	void updateLinearVelocity(Vec3 lv);  // lv' = lv + h * ( f / m)
	void updateAngularMomentum(Vec3 am); // L' = L + h * torque
	void updateRotation(Quat r);         // r' = r + 0.5 * h * Quat(w,0) * r

	// Setters for updates, which derive from calculation 
	void setInetriaTensor(Mat4 it);      // I^-1' = r.getRot() * I^-1 * r.getRot().inverse()
	void setAngularVelocity(Vec3 av);    // av' = I^-1 * L

	void computeWorldMatrix();           // new world matrix -> update rigidbody drawing
	void updateVertexOfWorld();          // new verticves -> update wireframe drawing
	void clearForce();

private:
	Mat4 m_toWorld = Mat4();

	Vec3 m_force = Vec3();
	Vec3 m_forceLoc = Vec3();
	Vec3 m_centerPosition = Vec3();
	Vec3 m_size = Vec3();

	Vec3 m_torque = Vec3();

	Quat m_rotation = Quat();

	Vec3 m_angularVelocity = Vec3();
	Vec3 m_linearVelocity = Vec3();
	Mat4 m_inertiaTensor = Mat4();
	Mat4 m_inertiaTensor0 = Mat4();

	Vec3 m_angularMomentum = Vec3();

	float m_mass = 0.0f;

	bool m_isFixed = false;

	vector<Vec3> m_vertices = vector<Vec3>();

	Vec3 m_color = Vec3(); // for each rigid body enable different colours
};

#endif