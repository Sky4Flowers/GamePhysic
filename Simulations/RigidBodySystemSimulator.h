#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h"


#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void newApplyForce(Vec3 * forceSum, Vec3 * q, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	//Own Functions
	void RigidBodySystemSimulator::applyCollisionForces(const int&, const float&);
	void RigidBodySystemSimulator::doTheJ(Vec3, Vec3, int, int, float, float);
	void RigidBodySystemSimulator::printRigidInfo(int);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	//variable for use of the first demo: only use one timestep
	bool isFirstStep = true;
	Vec3 m_externalForce;
	float m_gravity=0;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	//for forces with mouse
	bool isDragged = false;
	Vec3 first;
	float clickRadius;
	Vec3 newF;

	//rigidBody struct
	struct RigidBody {
		Vec3 pos; //position
		Vec3 size; //size
		Vec3 vel; //v
		Vec3 angularVel;//w
		Quat rot;//r
		Mat4 inertiaTensor;// I
		Vec3 angularMomentum; // L
		
		int mass;
		float iTensor;
	};

	//collision struct
	struct Collision {
		int body_a;
		int body_b;
	};

	//vector of rigidBodies
	vector<RigidBody> bodies;

	//vector of collisions
	vector<Collision> collisions;

	//self made methods
	void updateTensor(RigidBody * body);
	Vec3 calcAccel(Vec3 force, int index);
	};
#endif