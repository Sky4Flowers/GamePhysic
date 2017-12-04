#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	clickRadius = 3.0f;
	isDragged = false;
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "DEMO 1, DEMO 2, DEMO 3, DEMO 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	//TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_gravity, "min=-20.0 step=0.1");
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
	m_iTestCase = testCase;
	//print case
	cout << "Notify: " << testCase << "\n";
	//clear everything
	bodies.clear();
	switch (testCase) 
	{
	case 0:
		isFirstStep = true;
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		//set rotation
		setOrientationOf(0, Quat(0, 0, M_PI / 2.0f));
		break;
	case 1:
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		//set rotation
		setOrientationOf(0, Quat(0, 0, M_PI / 2.0f));
		break;	
	default:
		//Collision Test
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		//set rotation
		setOrientationOf(0, Quat(0, 0, M_PI / 2.0f));
		//moving RB
		addRigidBody(Vec3(0, 0.5, 3), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(1, Quat(0, 0, M_PI / 2.0f));
		setVelocityOf(1, Vec3(0, 0, -1));
	}
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
		Vec3 q = Vec3(0, 0, 0); //q is torque
		//all pts
		//forces - do the apply-force on body method. relevant for demo 2, 4
		if (m_iTestCase == 1) 
		{
			Vec3 topRightDeep = Vec3(bodies[i].pos.x -(0.5*bodies[i].size.x), bodies[i].pos.y + (0.5*bodies[i].size.y), bodies[i].pos.z + (0.5*bodies[i].size.z));
			topRightDeep = bodies[i].rot.getRotMat().transformVector(topRightDeep);
			newApplyForce(&forceSum,&q,topRightDeep,newF);
		}

		//we still have to get them !!!!!!!!!!!!!!!

		//hard coded force which is only relevant to the first demo
		if (m_iTestCase==0 && isFirstStep)
		{
			newApplyForce(&forceSum, &q, Vec3(0.3f, 0.5f, 0.25f), Vec3(1, 1, 0));
		}

		//collision here... only relevant for demos 3 and 4
		
		//euler

		Vec3 accel = calcAccel(forceSum, i);

		//update position of first point
		bodies[i].pos += bodies[i].vel *timeStep;
		bodies[i].vel += accel *timeStep;

		//rotation
		//update rotation r
		bodies[i].rot += (timeStep / 2.0f)*Quat( //caution! unsure about the angularVel as quaternion
			0, bodies[i].angularVel.x, bodies[i].angularVel.y, bodies[i].angularVel.z
			) * bodies[i].rot;
		bodies[i].rot /= bodies[i].rot.norm(); //normalize the rotation quaternion
											   //update angualr momentrum L
		bodies[i].angularMomentum += timeStep*q;
		applyCollisionForces(i, timeStep);//Collisions
		//update inertia tensor I
		updateTensor(&(bodies[i]));
		//update angualr velocity w
		bodies[i].angularVel = bodies[i].inertiaTensor.inverse() * bodies[i].angularMomentum;
		//world position stuff -- maybe there is no need for this.

		//if this is the first step, print the solution: angular / linear velocity
		if (isFirstStep)
		{
			isFirstStep = false;
			cout << "---RESULTS FOR DEMO 1---\nAngular VelocitY = " << bodies[i].angularVel <<
				".\nLinear Velocity = " << bodies[i].vel << "\n";
		}
	}	newF = Vec3(0, 0, 0);
}


void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	cout << "mouse clicked with pos: " << x << "," << y << "\n";
	if (!isDragged){//&& (m_iIntegrator == 4 || m_iIntegrator == 5)) {
		isDragged = true;
		first = Vec3(x, y, 0);
		cout << "firstest vec: " << first << "\n";
	}
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	if (isDragged) {
		//cout << "first vec: " << first.x << "," << first.y;
		if ((sqrt(Vec3(x, y, 0).squaredDistanceTo(first)) >= clickRadius)) {
			isDragged = false;
			newF = Vec3(x, y, 0) - first;
			newF *= Vec3(1, -1, 0);
			cout << "external force: " << newF.x << "," << newF.y << "\n";
			//test in welchen rigidbody die external force reingeht
		}
	}
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
	*q += cross(force, loc);
	*forceSum += force;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidBody tmp;
	tmp.pos = position;
	tmp.size = size;
	tmp.mass = mass;
	tmp.rot = Quat();
	tmp.angularVel = Vec3();
	tmp.vel = Vec3();
	//set initial inertia tensor for rigidBodies
	//note: this is a 4x4 matrix, although the calculation was 3x3
	tmp.inertiaTensor = Mat4(
		pow(tmp.size.y, 2) + pow(tmp.size.z, 2), 0.0f, 0.0f, 0.0f
		, 0.0f, pow(tmp.size.x, 2) + pow(tmp.size.z, 2), 0.0f, 0.0f
		, 0.0f, 0.0f, pow(tmp.size.x, 2) + pow(tmp.size.y, 2), 0.0f
		, 0, 0, 0, 1
		);

	tmp.inertiaTensor *= (float)tmp.mass / 12.0f;
	//set the last entry of the matrix as a 1 (not sure about this)
	tmp.inertiaTensor.value[3][3] = 1.0f;
	cout << "Added a rigidBody. Inertia tensor:\n" << tmp.inertiaTensor << "\n";

	//set the Angular Momentum
	tmp.angularMomentum = tmp.inertiaTensor * tmp.angularVel;
	cout << (tmp.angularMomentum) << "\n";

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

void RigidBodySystemSimulator::applyCollisionForces(const int& i, const float& timeStep) {
	Mat4 scaleMat, transMat, rotMat, Obj2WorldMatrix_A, Obj2WorldMatrix_B;
	//calc Obj2WorldMatrix for Object A
	transMat.initTranslation(bodies[i].pos.x, bodies[i].pos.y, bodies[i].pos.z);
	rotMat = bodies[i].rot.getRotMat();
	scaleMat.initScaling(bodies[i].size.x, bodies[i].size.y, bodies[i].size.z);
	Obj2WorldMatrix_A = scaleMat * rotMat * transMat;

	for (int k = i + 1; k < bodies.size(); k++) {
		//calc Obj2WorldMatrix for Object B
		transMat.initTranslation(bodies[k].pos.x, bodies[k].pos.y, bodies[k].pos.z);
		rotMat = bodies[k].rot.getRotMat();
		scaleMat.initScaling(bodies[k].size.x, bodies[k].size.y, bodies[k].size.z);
		Obj2WorldMatrix_B = scaleMat * rotMat * transMat;

		//Collisiondetection
		CollisionInfo ci;
		ci = checkCollisionSAT(Obj2WorldMatrix_A, Obj2WorldMatrix_B); //To test: Is the collpoint of A or of B?
		
		float c = 0.5f;
		if (ci.isValid) {
			cout << "In RigidBodySystemSimulator.cpp > applyCollisionForces() : Hardcoded parameter" << endl;
			cout << "Collision!" << endl;
			doTheJ(ci.collisionPointWorld, ci.normalWorld, i, k, c, timeStep);
		}
		//cout << "Collisions done" << endl;
	}
}

//TODO: fix this, inertia tensor is not a float
void RigidBodySystemSimulator::doTheJ(const Vec3& point, const Vec3& normal_n, const int& body_a, const int& body_b, const float& c, const float& timeStep) {
	//Crossproduct of x(i) and n
	Vec3 pxn_a = cross(point - bodies[body_a].pos, normal_n);
	Vec3 pxn_b = cross(point - bodies[body_b].pos, normal_n);
	Vec3 vrel = (bodies[body_a].vel + cross(bodies[body_a].angularVel, pxn_a)) - (bodies[body_b].vel + cross(bodies[body_b].angularVel, pxn_b));
	Quat invITa = bodies[body_a].inertiaTensor.inverse();
	Quat invITb = bodies[body_b].inertiaTensor.inverse();
	//Calculate J
	float J = dot(-(1 + c)*(vrel), normal_n) /
		((1 / bodies[body_a].mass)
			+ (1 / bodies[body_b].mass)
			+ dot(cross(invITa.getAxis()*pxn_a, normal_n)//dot(pxn_a,pxn_a) / bodies[body_a].iTensor
			+ cross(invITb.getAxis()*pxn_b, normal_n), normal_n));//dot(pxn_b,pxn_b) / bodies[body_b].iTensor);
	//Update the velocities	
	bodies[body_a].vel += J*normal_n / bodies[body_a].mass;
	bodies[body_b].vel -= J*normal_n / bodies[body_b].mass;

	bodies[body_a].angularMomentum += timeStep*cross(point - bodies[body_a].pos, J*normal_n);//cross(point - bodies[body_a].pos, J*normal_n) / bodies[body_a].iTensor;
	bodies[body_b].angularMomentum += timeStep*cross(point - bodies[body_b].pos, J*normal_n);//cross(point - bodies[body_b].pos, J*normal_n) / bodies[body_b].iTensor;
}
void RigidBodySystemSimulator::updateTensor(rigidBody *body) {
	Mat4 transposed = body->rot.getRotMat();
	transposed.transpose();
	//get I_0
	body->inertiaTensor = body->rot.getRotMat() *body->inertiaTensor * transposed;
	body->inertiaTensor.value[3][3] = 1.0f;
}

