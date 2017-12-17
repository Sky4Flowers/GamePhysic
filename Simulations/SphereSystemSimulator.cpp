﻿#include "SphereSystemSimulator.h"

std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// SphereSystemSimulator member functions

// Construtors
SphereSystemSimulator::SphereSystemSimulator() {
	//set the confines
	walls.centre = Vec3();
	walls.dist = 0.5f;
}
// Functions
const char * SphereSystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
	//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_numSpheres, "min=1");
	//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_sphereSize, "min=0.01 step=0.01");
	//TwAddVarRW(DUC->g_pTweakBar, "Ext. Force Intensity", TW_TYPE_FLOAT, &intensity, "min=0.0 step=0.1");
	//TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_gravity, "min=-20.0 step=0.1");
}

void SphereSystemSimulator::reset() {

}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

	for (int i = 0; i < spheres.size(); i++) 
	{
		DUC->drawSphere(spheres[i].Pos, spheres[i].radius);
	}
}

void SphereSystemSimulator::notifyCaseChanged(int testCase) {
	//clear the points
	spheres.clear();
	//add test sphere
	sphere test;
	test.mass = 2;
	test.Pos = Vec3();
	test.radius = 0.1f;
	test.Vel = Vec3(1, 0.7f, 0.2f);
	spheres.push_back(test);
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void SphereSystemSimulator::simulateTimestep(float timeStep) {
	//apply midpoint step
	applyMidpoint(timeStep);
}

void SphereSystemSimulator::applyMidpoint(float timeStep) 
{
	//iterate through spheres
	for (int i = 0; i < spheres.size(); i++) 
	{
		//do midpoint for each of the spheres
		sphere *sph = &(spheres[i]);
		Vec3 xtmp = sph->Pos + sph->Vel * timeStep / 2.0f;
		Vec3 accel;
		//calc accel from collisions
		Vec3 vtmp = sph->Vel + accel * timeStep / 2.0f;
		sph->Pos += timeStep * vtmp;
		//update accel from collisions again! based on xtmp and vtmp
		sph->Vel += timeStep * accel;
		//handle wall collisions
		checkWalls(sph);
	}
}

void SphereSystemSimulator::checkWalls(sphere *ball) 
{
	//check x value
	if ((ball->Pos.x - ball->radius) <= (walls.centre.x - walls.dist) ||
		(ball->Pos.x + ball->radius) >= (walls.centre.x + walls.dist))
		ball->Vel *= Vec3(-1, 1, 1);
	//check y value
	if ((ball->Pos.y - ball->radius) <= (walls.centre.y - walls.dist) ||
		(ball->Pos.y + ball->radius) >= (walls.centre.y + walls.dist))
		ball->Vel *= Vec3(1, -1, 1);
	//check z value
	if ((ball->Pos.z - ball->radius) <= (walls.centre.z - walls.dist) ||
		(ball->Pos.z + ball->radius) >= (walls.centre.z + walls.dist))
		ball->Vel *= Vec3(1, 1, -1);
}

void SphereSystemSimulator::onClick(int x, int y) {

}

void SphereSystemSimulator::onMouse(int x, int y) {

}