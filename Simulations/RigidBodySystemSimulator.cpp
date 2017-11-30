#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	//set rotation
	setOrientationOf(0, Quat(0, 0, M_PI / 2.0f));

}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "DEMO 1, DEMO 2, DEMO 3, DEMO 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	/*m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	Mat4 scaleMat, transMat, rotMat, Obj2WorldMatrix;
	for (int i = 0; i < bodies.size(); i++)
	{
		//assign values of current body
		transMat.initTranslation(bodies[i].pos.x, bodies[i].pos.y, bodies[i].pos.z);
		rotMat = bodies[i].rot.getRotMat();
		scaleMat.initScaling(bodies[i].size.x, bodies[i].size.y, bodies[i].size.z);
		//calc Obj2WorldMatrix
		Obj2WorldMatrix = scaleMat * rotMat * transMat;
		DUC->drawRigidBody(Obj2WorldMatrix);
	}*/
	//cout << "reset!\n";
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	//lighting
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4*Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	//variable matrices
	Mat4 scaleMat, transMat, rotMat, Obj2WorldMatrix;
	for (int i = 0; i < bodies.size(); i++)
	{
		//assign values of current body
		transMat.initTranslation(bodies[i].pos.x, bodies[i].pos.y, bodies[i].pos.z);
		rotMat = bodies[i].rot.getRotMat();
		scaleMat.initScaling(bodies[i].size.x, bodies[i].size.y, bodies[i].size.z);
		//calc Obj2WorldMatrix
		Obj2WorldMatrix = scaleMat * rotMat * transMat;
		DUC->drawRigidBody(Obj2WorldMatrix);
	}
	
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	//cout << "Notify: " << testCase << "\n";

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//adds all external forces to one, we have a vector external forces!
	//later: for all forces we have!
	//float force = m_externalForce.x + m_externalForce.y + m_externalForce.z;

}

//returns acceleration for a mass point
Vec3 RigidBodySystemSimulator::calcAccel(Vec3 force, int index)
{
	return force / bodies[index].mass;
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	//update q in this method

	for (int i = 0; i < bodies.size(); i++) {
		//TBD
		Vec3 forceSum = Vec3(0, 0, 0);
		//here: calc of q
		Vec3 q = Vec3(0, 0, 0);
		//all pts
		//forces - do the apply-force on body method

		//we still have to get them !!!!!!!!!!!!!!!

		//hard coded force which is only relevant to the first demo
		if (isFirstStep) 
		{
			newApplyForce(&forceSum, &q, Vec3(0.3f, 0.5f, 0.25f), Vec3(1, 1, 0));
			isFirstStep = false;
		}
		
		//collision here...

		//euler

		Vec3 accel = calcAccel(forceSum, i);
			
		//update position of first point
		bodies[i].pos += bodies[i].vel *timeStep;
		bodies[i].vel += accel *timeStep;
	}

}


void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return bodies[i].pos;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return bodies[i].vel;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return bodies[i].angularVel;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	//Vec3 relPos = loc - bodies[i].pos;
	//bodies[i].torque += cross(relPos, force);
	//bodies[i].acc += force;
}

void RigidBodySystemSimulator::newApplyForce(Vec3 *forceSum, Vec3 *q, Vec3 loc, Vec3 force) 
{
	*q += cross(force,loc);
	*forceSum += force;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidBody tmp;
	tmp.pos = position;
	tmp.size = size;
	tmp.mass = mass;
	tmp.rot = Quat();

	//set initial inertia tensor for rigidBodies
	//note: this is a 4x4 matrix, although the calculation was 3x3
	tmp.inertiaTensor = Mat4(
		pow(tmp.size.y, 2) + pow(tmp.size.z, 2), 0.0f, 0.0f, 0.0f
		, 0.0f, pow(tmp.size.x, 2) + pow(tmp.size.z, 2), 0.0f, 0.0f
		, 0.0f, 0.0f, pow(tmp.size.x, 2) + pow(tmp.size.y, 2), 0.0f
		, 0, 0, 0, 1
		);

	tmp.inertiaTensor *= (float)tmp.mass / 12.0f ;
	//set the last entry of the matrix as a 1 (not sure about this)
	tmp.inertiaTensor.value[3][3] = 1.0f;
	cout << "Added a rigidBody. Inertia tensor:\n" << tmp.inertiaTensor << "\n";
	
	bodies.push_back(tmp);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	bodies[i].rot = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	bodies[i].vel = velocity;
}

//Soll Gesamtforce (Später auch Angular) zurückgeben
void RigidBodySystemSimulator::getCollisionForcesOf(const int& i) {
	Mat4 scaleMat, transMat, rotMat, Obj2WorldMatrix_A, Obj2WorldMatrix_B;
	//calc Obj2WorldMatrix for Object A
	rotMat = bodies[i].rot.getRotMat();
	scaleMat.initScaling(bodies[i].size.x, bodies[i].size.y, bodies[i].size.z);
	Obj2WorldMatrix_A = scaleMat * rotMat * transMat;

	for (int k = i + 1; k < bodies.size();k++) {
		//calc Obj2WorldMatrix for Object B
		rotMat = bodies[k].rot.getRotMat();
		scaleMat.initScaling(bodies[k].size.x, bodies[k].size.y, bodies[k].size.z);
		Obj2WorldMatrix_B = scaleMat * rotMat * transMat;

		//Collisiondetection
		CollisionInfo ci = checkCollisionSAT(Obj2WorldMatrix_A, Obj2WorldMatrix_B); //To test: Is the collpoint of A or of B?
		cout << "In RigidBodySystemSimulator.cpp > getCollisionForceOf() : Hardcoded parameter" << endl;
		float c = 0.5f;
		doTheJ(ci.collisionPointWorld, ci.normalWorld, i, k, c);
	}
}

void RigidBodySystemSimulator::doTheJ(const Vec3& point, const Vec3& normal_n, const int& body_a, const int& body_b, const float& c) {
	//Crossproduct of x(i) and n
	Vec3 pxn_a = cross(point - bodies[body_a].pos, normal_n);
	Vec3 pxn_b = cross(point - bodies[body_b].pos, normal_n);
	Vec3 vrel = (bodies[body_a].vel + cross(bodies[body_a].angularVel, pxn_a)) - (bodies[body_b].vel + cross(bodies[body_b].angularVel, pxn_b));
	//Calculate J
	float J = dot(-(1 + c)*(vrel), normal_n) /
		((1 / bodies[body_a].mass)
			+ (1 / bodies[body_b].mass)
			+ dot(pxn_a,pxn_a) / bodies[body_a].iTensor
			+ dot(pxn_b,pxn_b) / bodies[body_b].iTensor);
	bodies[body_a].vel += J*normal_n / bodies[body_a].mass;
	bodies[body_b].vel -= J*normal_n / bodies[body_b].mass;
	bodies[body_a].angularVel += cross(point - bodies[body_a].pos, J*normal_n) / bodies[body_a].iTensor;
	bodies[body_b].angularVel += cross(point - bodies[body_b].pos, J*normal_n) / bodies[body_b].iTensor;
}void RigidBodySystemSimulator::updateTensor(rigidBody *body) 
{
	Mat4 transposed = body->rot.getRotMat();
	transposed.transpose();
	//get I_0
	body->inertiaTensor = body->rot.getRotMat() *body->inertiaTensor * transposed;
	body->inertiaTensor.value[3][3] = 1.0f;
}

