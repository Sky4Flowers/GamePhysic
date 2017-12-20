#pragma once
#include "Simulator.h"

class SphereSystem :public Simulator {
public:
	// Construtors
	SphereSystem(Vec3, float);

	//Structs
	struct sphere {
		Vec3 Pos;//Position
		Vec3 Vel;//Velocity
		float radius;//radius of the sphere
		float mass;
	};

	// Functions
	void reset();
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void applyMidpoint(float timeStep);
	Vec3 findCollisions(int, int);
	const vector<sphere>* getContent();

protected:
	// Attributes
	Vec3 externalForce;
	float m_fMass;
	float m_fRadius;
	float m_fForceScaling;
	float m_fDamping;
	int   m_iNumSpheres;

	int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	static std::function<float(float)> m_Kernels[5];

	int   m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)
	float m_gravity;

	//vector containing spheres
	vector<sphere> spheres;

	int* grid = new int[];

	struct confines {
		Vec3 centre;
		float dist;
	};

	confines walls;

	void checkWalls(sphere * ball);
	void addSphere(Vec3, Vec3);
	void rasterise();
};