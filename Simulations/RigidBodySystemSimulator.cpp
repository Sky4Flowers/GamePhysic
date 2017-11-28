#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	//set rotation
	setOrientationOf(0, Quat(0, 0, M_PI / 2.0f));
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return nullptr;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
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
		//forces - do the apply-force on body method

		//we still have to get them !!!!!!!!!!!!!!!
		vector <Vec3> forces;
		//TBD
		Vec3 forceSum = Vec3(0,0,0);
		for (int k = 0; k < forces.size();k++) {
			forceSum += forces[k];
		}
		//all pts
		vector <Vec3> locations;

		//here: calc of q
		Vec3 q = Vec3(0, 0, 0);
		for (int j = 0; j < locations.size();j++) {
			//crossproduct: locations with forces
			q += cross(locations[j],forces[j]);
		}

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
	return bodies.size;
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
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidBody tmp;
	tmp.pos = position;
	tmp.size = size;
	tmp.mass = mass;
	tmp.rot = Quat();

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

