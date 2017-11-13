#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

//additional defines for more scenes
#define DEMO_ONE 3
#define COMPLEX_EULER 4
#define COMPLEX_MIDPOINT 5

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	void updateLength(int index);

	Vec3 calcAccel(Vec3 Pos, Vec3 otherPos, float currentLength, float initialLength);

	void eulerStep(float timeStep);

	void midPointStep(float timeStep);

	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	int m_numSpheres;
	float m_sphereSize;
	//used to scale the system to the box in the simulator, hard coded to 1/4
	float fitToBoxCoef;
	//used to store temporary force and acceleration
	Vec3 force;
	Vec3 accel;
	int count;

	//struct for mass points
	struct massPoint {
		Vec3 Pos;//positon
		Vec3 Vel;//velocity
		float mass;
		bool isFixed;
	};

	//a vector of massPoints, to iterate over
	vector <massPoint> points;

	//struct for springs
	struct spring {
		massPoint *point1;
		massPoint *point2;
		float initialLength;
		float currentLength;
		//missing stiffness -> is a field in MassSpringSystemSim
	};

	//a vector of springs
	vector<spring> springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif