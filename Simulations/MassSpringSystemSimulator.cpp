#include "MassSpringSystemSimulator.h"
// Construtors
MassSpringSystemSimulator::MassSpringSystemSimulator(){
	//TODO:
	
	setIntegrator(0); //euler is the default integrator
}

// UI Functions
const char * MassSpringSystemSimulator::getTestCasesStr(){
	//Only Dummyimplementation
	return "EULER, LEAPFROG, MIDPOINT";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC){
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_numSpheres, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_sphereSize, "min=0.01 step=0.01");
}

void MassSpringSystemSimulator::reset(){
	//Dummyimplementation (Check it)
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	cout << "reset!\n";
	//clear vectors
	//springs.clear();
	//points.clear();
	//call constructor
	//MassSpringSystemSimulator::MassSpringSystemSimulator();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	DUC->setUpLighting(Vec3(0,1,0), 0.4*Vec3(0, 0, 0), 100, 0.6*Vec3(1,1,1));
	//For more Spheres
	for (int i = 0; i<m_numSpheres; i++)
	{
		DUC->drawSphere(points[i].Pos, Vec3(m_sphereSize, m_sphereSize, m_sphereSize));
	}	
		//begin line
		DUC->beginLine();
		//DUC->drawLine(Vec3(0, 0, 0), Vec3(1, 1, 1), Vec3(0, 2, 0), Vec3(0, 0, 1));
		DUC->drawLine(springs[0].point1, Vec3(1, 1, 1), springs[0].point2, Vec3(1, 1, 1));
		DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase){
	cout << "Notify : " << testCase << "\n";
	setIntegrator(testCase);
	switch (m_iIntegrator) 
	{
	case 0:
		cout << "Integrator: Euler\n";
		points.clear();
		springs.clear();
		//set field values
		m_numSpheres = 2;
		m_sphereSize = 0.05f;
		//fitToBoxCoef = 0.25f;

		//add mass points
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		//add a spring between the 2 mass points
		addSpring(0, 1, 1);
		setStiffness(40);
		break;
	case 1:
		cout << "Integrator: Leap Frog\n";
		break;
	case 2:
		cout << "Integrator: Midpoint\n";
		points.clear();
		springs.clear();
		//set field values
		m_numSpheres = 2;
		m_sphereSize = 0.05f;
		//fitToBoxCoef = 0.25f;

		//add mass points
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		//add a spring between the 2 mass points
		addSpring(0, 1, 1);
		setStiffness(40);
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	//euler
	switch (m_iIntegrator)
	{
	case (0) :
		for (int i = 0; i < m_numSpheres; i++) {
			force = -1.0f * m_fStiffness*(springs[0].currentLength - springs[0].initialLength)*
				(points[i].Pos - points[(i + 1) % m_numSpheres].Pos) / springs[0].currentLength;
			accel = force / points[i].mass;
			points[i].Vel += accel * timeStep;
			points[i].Pos += points[i].Vel * timeStep;
		}
			 //update spring points
			 springs[0].point1 = points[0].Pos;
			 springs[0].point2 = points[1].Pos;
			 springs[0].currentLength = sqrt(springs[0].point1.squaredDistanceTo(springs[0].point2));
			 break;
	case(1) :
		//leap frog
		break;
	case(2) :
		//midpoint a la VL
		for (int i = 0; i < m_numSpheres; i++)
		{
			Vec3 xtmp = points[i].Pos + points[i].Vel * timeStep * 0.5f;
			//cout << xtmp <<"\n";
			force = -1.0f * m_fStiffness*(springs[0].currentLength - springs[0].initialLength)*
				(points[i].Pos - points[(i + 1) % m_numSpheres].Pos) / springs[0].currentLength;
			accel = force / points[i].mass;

			Vec3 vtmp = points[i].Vel + accel * 0.5f;
			points[i].Pos += timeStep * vtmp;

			force = -1.0f * m_fStiffness*(springs[0].currentLength - springs[0].initialLength)*
				(xtmp - points[(i + 1) % m_numSpheres].Pos) / springs[0].currentLength;
			accel = force / points[i].mass;

			points[i].Vel += timeStep * accel;
		}
			//update spring points
			springs[0].point1 = points[0].Pos;
			springs[0].point2 = points[1].Pos;
			springs[0].currentLength = sqrt(springs[0].point1.squaredDistanceTo(springs[0].point2));
			break;
	}

	//placeholder for other implementors

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
	massPoint tmp;
	tmp.isFixed = isFixed;
	tmp.Pos = position;
	tmp.Vel = Velocity;
	tmp.mass = 10.0f;
	points.push_back(tmp);
	cout << "Added a point to the list!\n";
	return 0;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	spring tmp;
	tmp.point1 = points[masspoint1].Pos;
	tmp.point2 = points[masspoint2].Pos;
	tmp.initialLength = initialLength;
	tmp.currentLength = sqrt(tmp.point1.squaredDistanceTo(tmp.point2));
	springs.push_back(tmp);
	cout << "Added a spring! it's currentLength is: " << tmp.currentLength << "\n";
}

int MassSpringSystemSimulator::getNumberOfMassPoints(){
	return m_numSpheres;
}

int MassSpringSystemSimulator::getNumberOfSprings(){
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){
	return points[index].Pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){
	return points[index].Vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){
	//TODO
}