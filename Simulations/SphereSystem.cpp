#include "SphereSystem.h"

std::function<float(float)> SphereSystem::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// SphereSystemSimulator member functions

// Construtors
SphereSystem::SphereSystem(Vec3 center, float distance) {
	//set the confines
	walls.centre = center;
	walls.dist = distance;
}

void SphereSystem::reset() {
	spheres.clear();
	grid = new int[];
}

void SphereSystem::externalForcesCalculations(float timeElapsed) {

}

void SphereSystem::simulateTimestep(float timeStep) {
	//apply midpoint step
	applyMidpoint(timeStep);
}

void SphereSystem::applyMidpoint(float timeStep)
{

	vector<sphere> newSpheres;
	//iterate through spheres
	for (int i = 0; i < spheres.size(); i++)
	{
		//do midpoint for each of the spheres
		sphere *sph = &(spheres[i]);
		Vec3 xtmp = sph->Pos + sph->Vel * timeStep / 2.0f;
		Vec3 accel;
		//calc accel from collisions
		accel = findCollisions(0, i) / sph->mass;
		Vec3 vtmp = sph->Vel + accel * timeStep / 2.0f;
		//sph->Pos += timeStep * vtmp;

		//Zwischenspeichern alter Werte
		Vec3 oldPos = sph->Pos;
		Vec3 oldVel = sph->Vel;
		//Setzen von Zwischenwerten
		sph->Pos = xtmp;
		sph->Vel = vtmp;
		//Berechnen von Acceleration und Endwerten
		accel = findCollisions(0, i) / sph->mass;
		sph->Pos = oldPos;
		sph->Vel = oldVel;

		sphere neu;
		neu.mass = sph->mass;
		neu.Pos = oldPos + timeStep * vtmp;
		neu.Vel = oldVel + timeStep * accel;
		neu.radius = sph->radius;
		//update accel from collisions again! based on xtmp and vtmp

		//sph->Vel += timeStep * accel;
		//handle wall collisions
		checkWalls(&neu);
		newSpheres.push_back(neu);
	}
	spheres = newSpheres;
}

void SphereSystem::checkWalls(sphere *ball)
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

Vec3 SphereSystem::findCollisions(int collisionCase, int obj_a) {
	Vec3 force = Vec3(0, 0, 0);
	for (int i = 0; i < spheres.size(); i++) {
		if (i == obj_a) {
			continue;
		}
		if (collisionCase == 1) {//Grid
			cout << "Not implemented! Case Grid" << endl;
		}
		else if (collisionCase == 2) {//KD-Tree
			cout << "Not implemented! Case KD-Tree" << endl;
		}
		else {//Check all spheres
			Vec3 absRelPos = (spheres[obj_a].Pos - spheres[i].Pos).getAbsolutes();
			float relativeDistance = sqrt(pow(absRelPos.x, 2) + pow(absRelPos.y, 2) + pow(absRelPos.z, 2)) / (m_fRadius * 2);
			if (relativeDistance < 1) {
				cout << "Hit" << endl;
				//Calculate Forcedirection
				Vec3 normal = spheres[obj_a].Pos - spheres[i].Pos;
				normal /= norm(normal);
				normal *= m_Kernels[2](10) * (1 - relativeDistance);
				force += normal;
			}
		}
	}
	return force;
}

void SphereSystem::addSphere(Vec3 position, Vec3 velocity) {
	sphere neu;
	neu.mass = m_fMass;
	neu.Pos = position;
	neu.Vel = velocity;
	neu.radius = m_fRadius;
	spheres.push_back(neu);
}

const vector<SphereSystem::sphere>* SphereSystem::getContent() {
	return &spheres;
}

void SphereSystem::rasterise() {
	int size1D = walls.dist / m_fRadius;
	grid = new int[size1D*size1D*size1D];
	for (int i = 0; i < spheres.size()-1; i++) {
		if (spheres[i].Pos.getAbsolutes.x > spheres[i + 1].Pos.getAbsolutes.x) {
			
		}
	}
}