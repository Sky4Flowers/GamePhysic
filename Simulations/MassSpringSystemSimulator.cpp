#include "MassSpringSystemSimulator.h"
// Construtors
MassSpringSystemSimulator::MassSpringSystemSimulator(){
	//TODO:
}

// UI Functions
const char * MassSpringSystemSimulator::getTestCasesStr(){
	//Only Dummyimplementation
	return "Version1, Version2";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC){

}

void MassSpringSystemSimulator::reset(){
	//Dummyimplementation (Check it)
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){

}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase){

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep){

}

void MassSpringSystemSimulator::onClick(int x, int y){
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y){
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions

void MassSpringSystemSimulator::setMass(float mass){
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness){
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping){
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed){
	
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){

}

int MassSpringSystemSimulator::getNumberOfMassPoints(){

}

int MassSpringSystemSimulator::getNumberOfSprings(){

}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){

}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){

}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){

}