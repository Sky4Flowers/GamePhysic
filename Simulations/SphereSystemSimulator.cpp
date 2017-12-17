#include "SphereSystemSimulator.h"

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

}
// Functions
const char * getTestCasesStr() {

}

void initUI(DrawingUtilitiesClass * DUC) {

}

void reset() {

}

void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

}

void notifyCaseChanged(int testCase) {

}

void externalForcesCalculations(float timeElapsed) {

}

void simulateTimestep(float timeStep) {

}

void onClick(int x, int y) {

}

void onMouse(int x, int y) {

}