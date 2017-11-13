#include "MassSpringSystemSimulator.h"
// Construtors
MassSpringSystemSimulator::MassSpringSystemSimulator(){
	//TODO:
	
	setIntegrator(0); //euler is the default integrator
	setMass(10.0f);
	
}

// UI Functions
const char * MassSpringSystemSimulator::getTestCasesStr(){
	//Only Dummyimplementation
	return "EULER, LEAPFROG, MIDPOINT, DEMO_ONE, COMPLEX_EULER, COMPLEX_MIDPOINT";
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
		for (int i = 0; i < springs.size(); i++) 
			DUC->drawLine(springs[0].point1->Pos, Vec3(1, 1, 1), springs[0].point2->Pos, Vec3(1, 1, 1));
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

	case 3:
		points.clear();
		springs.clear();
		cout << "Demo one: Calculating single step with euler and midpoint, output follows on console:\n";

	case 4:
		points.clear();
		springs.clear();
		cout << "Complex Demo with euler integration\n";
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){
	//Collisions
	int counter = 1;
	for (int i = 0;i<m_numSpheres;i++) {
		//Floorcollision
		if (points[i].Pos.y-m_sphereSize <= -1){
			points[i].Pos.y = -1+m_sphereSize;
			//With bouncing
			//NTH: DotProduct of m1.vel at Vec3(0,1,0)
		}
		if (points[i].Pos.x - m_sphereSize <= -1){
			points[i].Pos.x = m_sphereSize - 1;
		}
		else if (points[i].Pos.x + m_sphereSize >= 1){
			points[i].Pos.x = -m_sphereSize + 1;
		}
		if (points[i].Pos.z - m_sphereSize <= -1){
			points[i].Pos.z = m_sphereSize - 1;
		}
		else if (points[i].Pos.z + m_sphereSize >= 1){
			points[i].Pos.z = -m_sphereSize + 1;
		}
		//Collision with obstacles
		//for each(obstacle o in obstacles){}
		//Collision of moving spheres
		//for (int i = counter; i < points.size(); i++){
			//massPoint m2 = points[i];
			
			//If two spheres collide
			//if (sqrt(pow(m1.Pos.x - m2.Pos.x, 2) + pow(m1.Pos.y - m2.Pos.y, 2) + pow(m1.Pos.z - m2.Pos.z, 2)) <= m_sphereSize){
				//m1.Vel = (m2.mass / m1.mass)*-(DirectX::XMVector3Dot(m1.Vel.toDirectXVector, (m1.Pos - m2.Pos).toDirectXVector));
				//m2.Vel = (m1.mass / m2.mass)*-(DirectX::XMVector3Dot(m2.Vel.toDirectXVector, (m2.Pos - m1.Pos).toDirectXVector));
			//}
		//}
		//counter++;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	//euler
	switch (m_iIntegrator)
	{
	case (0) :
		/*
		for (int i = 0; i < m_numSpheres; i++) {
			force = -1.0f * m_fStiffness*(springs[0].currentLength - springs[0].initialLength)*
				(points[i].Pos - points[(i + 1) % m_numSpheres].Pos) / springs[0].currentLength;
			accel = force / points[i].mass;
			points[i].Pos += points[i].Vel * timeStep;
			points[i].Vel += accel * timeStep;
		}
			 //update spring points
			 //springs[0].point1 = points[0].Pos;
			 //springs[0].point2 = points[1].Pos;
			 //springs[0].currentLength = sqrt(springs[0].point1.squaredDistanceTo(springs[0].point2));
			 updateLength(0);
			 */
		eulerStep(timeStep);
		break;
	case(1) :
		//leap frog
		break;
	case(2) :
		//midpoint a la VL
		/*
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
			//springs[0].point1 = points[0].Pos;
			//springs[0].point2 = points[1].Pos;
			//springs[0].currentLength = sqrt(springs[0].point1.squaredDistanceTo(springs[0].point2));
			updateLength(0);
			*/
		midPointStep(timeStep);
		break;
	}
	//placeholder for other implementors

	externalForcesCalculations(timeStep);}

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
	tmp.mass = m_fMass;
	points.push_back(tmp);
	cout << "Added a point to the list!\n";
	return 0;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	spring tmp;
	tmp.point1 = &points[masspoint1]; //&points[masspoint1].Pos;
	tmp.point2 = &points[masspoint2];
	tmp.initialLength = initialLength;
	Vec3 a = tmp.point1->Pos;
	Vec3 b = tmp.point2->Pos;
	tmp.currentLength = sqrt(a.squaredDistanceTo(b));
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

//method updates the currentLength value of a spring, index of the spring in the vector
void MassSpringSystemSimulator::updateLength(int index) {
	Vec3 a = springs[index].point1->Pos;
	Vec3 b = springs[index].point2->Pos;
	springs[index].currentLength = sqrt(a.squaredDistanceTo(b));
}
//returns acceleration for a mass point
Vec3 MassSpringSystemSimulator::calcAccel(Vec3 Pos,
	Vec3 otherPos, float currentLength, float initialLength)
{
	Vec3 force = -1.0f * m_fStiffness * (currentLength - initialLength) * (Pos - otherPos) / currentLength;
	return force / m_fMass;
}
//calculates a single euler step
void MassSpringSystemSimulator::eulerStep(float timeStep) 
{
	for (int i = 0; i < springs.size(); i++) 
	{
		massPoint *a = springs[i].point1;
		massPoint *b = springs[i].point2;
		Vec3 accel = calcAccel(a->Pos, b->Pos, springs[i].currentLength, springs[i].initialLength);
		//update position of first point
		a->Pos += a->Vel * timeStep;
		//update velocity of first point
		a->Vel += accel * timeStep;
		//update position of second point
		b->Pos += b->Vel * timeStep;
		//update velocity of second point
		b->Vel += -1.0f * accel * timeStep;
		updateLength(i);
	}
}
//calculates a single midpoint step
void MassSpringSystemSimulator::midPointStep(float timeStep)
{
	for (int i = 0; i < springs.size(); i++) 
	{
		massPoint *a = springs[i].point1;
		massPoint *b = springs[i].point2;
		Vec3 xtmpA = a->Pos + a->Vel * timeStep / 2.0f;
		Vec3 xtmpB = b->Pos + b->Vel * timeStep / 2.0f;
		float currentLength = sqrt(xtmpA.squaredDistanceTo(xtmpB));
		//elastic forces, using xtmp?!
		Vec3 accel = calcAccel(a->Pos, b->Pos, springs[i].currentLength, springs[i].initialLength);
		Vec3 vtmpA = a->Vel + timeStep / 2.0f * accel;
		Vec3 vtmpB = b->Vel + timeStep / 2.0f * (-1.0f) * accel;
		a->Pos += timeStep * vtmpA;
		b->Pos += timeStep * vtmpB;
		accel = calcAccel(xtmpA, xtmpB, currentLength, springs[i].initialLength);
		a->Vel += timeStep * accel;
		b->Vel += timeStep * -1.0f * accel;
		updateLength(i);
	}
}