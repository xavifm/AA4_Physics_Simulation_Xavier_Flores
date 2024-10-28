#include "ParticleSystem.h"
#include "Particle.h"
//#include "CircularBuffer.h"
#include <stdio.h>
#include <glm/glm.hpp>
#include <iostream>
#include <glm/glm.hpp>
#include <time.h>
#include <../include/GLFW/glfw3.h>
#include <time.h>

Particle* vertexArray = new Particle[14 * 18];

namespace LilSpheres {
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern int firstParticleIdx;
	extern int particleCount;
}

namespace Sphere {
	extern void setupSphere(glm::vec3 pos = glm::vec3(0.f, 1.f, 0.f), float radius = 1.f);
	extern void cleanupSphere();
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}

namespace ClothMesh {
	extern const int numCols;
	extern const int numRows;
	extern const int numVerts;
	extern void updateClothMesh(float* array_data);
}

void ParticleSystem::updateClothMesh(float* array_data) 
{
	ClothMesh::updateClothMesh(array_data);
}

void ParticleSystem::updateVariables() 
{
	vertexArray->damping = damping;
	vertexArray->damping_2 = damping_2;
	vertexArray->elongation = elongation;
}

void Particle::calculateF(Particle nextP, float dist) {

	float d = glm::length(ActualPos - nextP.ActualPos);
	glm::vec3 speed = vel - nextP.vel;
	glm::vec3 normalized = glm::normalize(ActualPos - nextP.ActualPos);
	float subResult = damping_2 * (d - dist) + glm::dot(damping*speed, normalized);
	glm::vec3 res = -subResult * normalized;
	Force += res;

}

void Particle::distanceRegulation(Particle p) {

	float d = glm::length(ActualPos - p.ActualPos);
	float dMax = d + elongation;
	float difference = dMax - d;
	glm::vec3 v = ActualPos - p.ActualPos;
	if (d > dMax)
	{
		ActualPos = ActualPos + v * (difference / 2);
		p.ActualPos = p.ActualPos - v * (difference / 2);
	}
}

void ParticleSystem::meshSphereCollision(glm::vec3 Q, glm::vec3 &actPos, glm::vec3 &lastPos, glm::vec3 cS) {

	glm::vec3 n = glm::normalize(Q - cS);
	float d = -glm::dot(n, Q);
	if (glm::length(actPos - mesh_mode_sphere_pos) <= mesh_mode_sphere_radius) 
	{
		meshPlaneCollision(n, d, actPos, lastPos);
	}
}

bool hasCollision(glm::vec3 Pt, glm::vec3 n, float d, glm::vec3 PtPost, int plane) {

	float getPos;
	getPos = ((glm::dot(n, Pt) + d) * (glm::dot(n, PtPost) + d));
	if (getPos <= 0) 
	{ 
		return true; 
	}
	else 
	{ 
		return false; 
	}
}

void ParticleSystem::meshPlaneCollision(glm::vec3 n, float d, glm::vec3 &pos, glm::vec3 &prevPos) {

	pos = pos - (1 + 0.8f) * (glm::dot(pos, n) + d) * n;
	prevPos = prevPos - (1 + 0.8f) * (glm::dot(prevPos, n) + d) * n;
}

glm::vec3 ParticleSystem::getContactPointBetweenParticleAndSphere(glm::vec3 position, glm::vec3 lastP, glm::vec3 cS, float r) {

	float a, b, c, alpha = 0;
	glm::vec3 dir = glm::normalize(position - lastP);
	a = glm::dot(dir, dir);
	b = 2 * glm::dot(dir, lastP - cS);
	c = glm::dot(lastP - cS, lastP - cS) - r * r;

	float res1 = (-b + sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
	float res2 = (-b - sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);

	if (res1 <= 1 && res1 >= 0) alpha = res1;
	if (res2 <= 1 && res2 >= 0) alpha = res2;

	glm::vec3 Q = lastP + (position - lastP)*alpha;
	return Q;
}

void ParticleSystem::resetPhysics() {

	delete[] currentMesh;
	delete[] lastMesh;
	delete[] temporalMesh;
	delete[] outputMesh;
	delete vertexArray;
}

ParticleSystem::ParticleSystem(int numParticles) : numParticles(numParticles) {
	if(!meshMode) 
	{
		positions = new glm::vec3[emissionRate];
		velocities = new glm::vec3[emissionRate];
		prevPositions = new glm::vec3[emissionRate];
		for (int i = 0; i < emissionRate; i++)
		{
			prevPositions[i] = glm::vec3(0, 0, 0);
			if (fountainMode)
			{
				positions[i] = glm::vec3(0, 1, 0);
				velocities[i] = glm::vec3(cos(i) * 2, 5, sin(i) * 2);
				age[i] = rand() % 3 + 2;
			}
			else if (cascadeMode)
			{
				positions[i] = glm::vec3(rand() % 10 - 5, 8, 0);
				velocities[i] = glm::vec3(rand() % 10 - 5, 0, rand() % 2 - 1);
				age[i] = rand() % 5 + 2;
			}
		}
	}
	else if(!fluidsMode)
	{
		//NEW VERLET SYSTEM
		mesh_mode_cols = ClothMesh::numCols;
		mesh_mode_rows = ClothMesh::numRows;
		damping = 50;
		damping_2 = 900;
		elongation = 0;
		resetMode = false;
		runTime = 0;
		distanceBetweenSprings = 0.5f;
		temporalMesh = new glm::vec3[14 * 18];
		outputMesh = new glm::vec3[14 * 18];
		currentMesh = new glm::vec3[14 * 18];
		lastMesh = new glm::vec3[14 * 18];
		vertexArray = new Particle[14 * 18];

		for (int j = 0; j < ClothMesh::numRows; j++) 
		{
			for (int i = 0; i < ClothMesh::numCols; i++) 
			{
				if (i == 0 && j == 0) 
				{
					currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5, 9.5, -4.5);
				}
				else if (i == ClothMesh::numCols - 1 && j == 0) 
				{
					currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5, 9.5, -4.5 + 1.7f);
				}
				currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5 + j * 0.5f, 9.5, -4.5 + i * 0.5f);
				lastMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j * ClothMesh::numCols + i)];
				temporalMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j * ClothMesh::numCols + i)];
				outputMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j * ClothMesh::numCols + i)];

				gravity = glm::vec3(0, -9.81, 0);

				vertexArray[j * ClothMesh::numCols + i].ActualPos = currentMesh[(j * ClothMesh::numCols + i)];
				vertexArray[j * ClothMesh::numCols + i].lastPos = lastMesh[(j * ClothMesh::numCols + i)];
				vertexArray[j * ClothMesh::numCols + i].Force = glm::vec3(0, 0, 0);
				vertexArray[j * ClothMesh::numCols + i].vel = glm::vec3(0, 0, 0);
				vertexArray[j * ClothMesh::numCols + i].alpha = 0;
			}
		}
		mesh_mode_sphere_pos_x = -4 + rand() % 8;
		mesh_mode_sphere_pos_y = 0 + rand() % 9;
		mesh_mode_sphere_pos_z = -4 + rand() % 8;
		mesh_mode_sphere_pos = glm::vec3(mesh_mode_sphere_pos_x, mesh_mode_sphere_pos_y, mesh_mode_sphere_pos_z);
		mesh_mode_sphere_radius = 0.5f + rand() % 3;
	}
	else if(fluidsMode) 
	{
		for (int j = 0; j < ClothMesh::numRows; j++) 
		{
			for (int i = 0; i < ClothMesh::numCols; i++) 
			{
				temporalMesh[(j * ClothMesh::numCols + i)] = glm::vec3(0.f, 0.f, 0.f);
				lastMesh[(j * ClothMesh::numCols + i)] = glm::vec3(0.f, 0.f, 0.f);
				initialMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5 + j * 0.5f, 0.f, -4.5 + i * 0.5f);
				currentMesh[(j * ClothMesh::numCols + i)] = initialMesh[(j * ClothMesh::numCols + i)];
			}
		}
		sphereX = -4 + rand() % 8;
		sphereY = 0 + rand() % 9;
		sphereZ = -4 + rand() % 8;
		centreSphere = glm::vec3(sphereX, sphereY, sphereZ);
		giantSphereRadius = 0.5f + rand() % 3;
	}
}

void ParticleSystem::calculateForces(int i, int j) {
	vertexArray[(j * ClothMesh::numCols + i)].Force = gravity;

	if (i < ClothMesh::numCols - 1) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[(j * ClothMesh::numCols + (i + 1))], distanceBetweenSprings);
	}
	if (i > 0) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[(j * ClothMesh::numCols + (i - 1))], distanceBetweenSprings);
	}
	if (i < ClothMesh::numCols - 2) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[(j * ClothMesh::numCols + (i + 2))], distanceBetweenSprings * 2);
	}
	if (i > 1) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[(j * ClothMesh::numCols + (i - 2))], distanceBetweenSprings * 2);
	}
	if (j < ClothMesh::numRows - 1) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j + 1) * ClothMesh::numCols + i)], distanceBetweenSprings);
	}
	if (j > 0) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j - 1) * ClothMesh::numCols + i)], distanceBetweenSprings);
	}
	if (j < ClothMesh::numRows - 2) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j + 2)* ClothMesh::numCols + i)], distanceBetweenSprings * 2);
	}
	if (j > 1) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j - 2) * ClothMesh::numCols + i)], distanceBetweenSprings * 2);
	}
	if (i < ClothMesh::numCols - 1 && j < ClothMesh::numRows - 1) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j + 1) * ClothMesh::numCols + (i + 1))], sqrt(pow(distanceBetweenSprings, 2) + pow(distanceBetweenSprings, 2)));
	}
	if (i < ClothMesh::numCols - 1 && j > 0) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j - 1) * ClothMesh::numCols + (i + 1))], sqrt(pow(distanceBetweenSprings, 2) + pow(distanceBetweenSprings, 2)));
	}
	if (i > 0 && j > 0) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j - 1) * ClothMesh::numCols + (i - 1))], sqrt(pow(distanceBetweenSprings, 2) + pow(distanceBetweenSprings, 2)));
	}
	if (i > 0 && j < ClothMesh::numRows - 1) 
	{ 
		vertexArray[(j * ClothMesh::numCols + i)].calculateF(vertexArray[((j + 1) * ClothMesh::numCols + (i - 1))], sqrt(pow(distanceBetweenSprings, 2) + pow(distanceBetweenSprings, 2)));
	}

	vertexArray[(j * ClothMesh::numCols + i)].Force += gravity;

}

void ParticleSystem::resetModeAll()
{
	Time++;
	if (Time > 800)
	{
		if(!fluidsMode)
		resetPhysics();
		resetModeF();
		Sphere::updateSphere(glm::vec3(mesh_mode_sphere_pos_x, mesh_mode_sphere_pos_y, mesh_mode_sphere_pos_z), mesh_mode_sphere_radius);
		Time = 0;
	}
	if (resetMode == true)
	{
		resetPhysics();
		resetModeF();
		Sphere::updateSphere(glm::vec3(mesh_mode_sphere_pos_x, mesh_mode_sphere_pos_y, mesh_mode_sphere_pos_z), mesh_mode_sphere_radius);
		Time = 0;
	}
	mesh_mode_sphere_pos = glm::vec3(mesh_mode_sphere_pos_x, mesh_mode_sphere_pos_y, mesh_mode_sphere_pos_z);
}

void ParticleSystem::calculateWithverletMethod(int i, int j, float dt)
{
	if (i == 0 && j == 0) 
	{
		currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5, 9.5, -4.5);
	}

	else if (i == ClothMesh::numCols - 1 && j == 0) 
	{
		currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5, 9.5, 1.7f);
	}
	else 
	{
		temporalMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j*ClothMesh::numCols + i)];
		outputMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j*ClothMesh::numCols + i)] + (currentMesh[(j*ClothMesh::numCols + i)] - lastMesh[(j*ClothMesh::numCols + i)]) + (vertexArray[(j*ClothMesh::numCols + i)].Force)*(dt*dt); // Calculates new position by the power of Verlet
		currentMesh[(j * ClothMesh::numCols + i)] = outputMesh[(j * ClothMesh::numCols + i)];
		lastMesh[(j * ClothMesh::numCols + i)] = temporalMesh[(j * ClothMesh::numCols + i)];

		cleft = hasCollision(lastMesh[(j * ClothMesh::numCols + i)], glm::vec3(1, 0, 0), 5, currentMesh[(j * ClothMesh::numCols + i)], 1);
		cright = hasCollision(lastMesh[(j * ClothMesh::numCols + i)], glm::vec3(-1, 0, 0), 5, currentMesh[(j * ClothMesh::numCols + i)], 2);
		cup = hasCollision(lastMesh[(j * ClothMesh::numCols + i)], glm::vec3(0, -1, 0), 10, currentMesh[(j * ClothMesh::numCols + i)], 3);
		cdown = hasCollision(lastMesh[(j * ClothMesh::numCols + i)], glm::vec3(0, 1, 0), 0, currentMesh[(j * ClothMesh::numCols + i)], 4);
		cfront = hasCollision(lastMesh[(j * ClothMesh::numCols + i)], glm::vec3(0, 0, -1), 5, currentMesh[(j * ClothMesh::numCols + i)], 5);
		cback = hasCollision(lastMesh[(j * ClothMesh::numCols + i)], glm::vec3(0, 0, 1), 5, currentMesh[(j * ClothMesh::numCols + i)], 6);

		if (cleft) 
		{
			meshPlaneCollision(glm::vec3(1, 0, 0), 5, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)]);
		}
		if (cright) 
		{
			meshPlaneCollision(glm::vec3(-1, 1, 0), 5, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)]);
		}
		if (cup) 
		{
			meshPlaneCollision(glm::vec3(0, -1, 0), 10, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)]);
		}
		if (cdown) 
		{
			meshPlaneCollision(glm::vec3(0, 1, 0), 0, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)]);
		}
		if (cfront) 
		{
			meshPlaneCollision(glm::vec3(0, 0, -1), 5, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)]);
		}
		if (cback) 
		{
			meshPlaneCollision(glm::vec3(0, 0, 1), 5, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)]);
		}

		vertexArray[(j * ClothMesh::numCols + i)].Q = getContactPointBetweenParticleAndSphere(lastMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)], mesh_mode_sphere_pos, mesh_mode_sphere_radius);
		meshSphereCollision(vertexArray[(j * ClothMesh::numCols + i)].Q, currentMesh[(j * ClothMesh::numCols + i)], lastMesh[(j * ClothMesh::numCols + i)], mesh_mode_sphere_pos);

		vertexArray[(j * ClothMesh::numCols + i)].lastPos = lastMesh[(j * ClothMesh::numCols + i)];
		vertexArray[(j * ClothMesh::numCols + i)].ActualPos = currentMesh[(j * ClothMesh::numCols + i)];
		vertexArray[(j * ClothMesh::numCols + i)].vel = (vertexArray[(j * ClothMesh::numCols + i)].ActualPos - vertexArray[(j * ClothMesh::numCols + i)].lastPos) / dt;

		if (j != ClothMesh::numRows - 1)
			vertexArray[(j * ClothMesh::numCols + i)].distanceRegulation(vertexArray[(j + 1) * ClothMesh::numCols + i]);
	}
}


//_______________________________________________


void ParticleSystem::resetModeF()
{
	positions = new glm::vec3[emissionRate];
	velocities = new glm::vec3[emissionRate];
	prevPositions = new glm::vec3[emissionRate];
	if (!meshMode)
	{
		for (int i = 0; i < emissionRate; i++)
		{
			prevPositions[i] = glm::vec3(0, 0, 0);
			if (fountainMode)
			{
				positions[i] = glm::vec3(0, 1, 0);
				velocities[i] = glm::vec3(cos(i) * 2, 5, sin(i) * 2);
				age[i] = rand() % 3 + 2;
			}
			else if (cascadeMode)
			{
				positions[i] = glm::vec3(rand() % 10 - 5, 8, 0);
				velocities[i] = glm::vec3(rand() % 10 - 5, 0, rand() % 2 - 1);
				age[i] = rand() % 5 + 2;
			}
		}
	}
	else if(!fluidsMode)
	{
		damping = 50;
		damping_2 = 900;
		elongation = 0;
		resetMode = false;
		runTime = 0;
		distanceBetweenSprings = 0.5f;
		temporalMesh = new glm::vec3[14 * 18];
		outputMesh = new glm::vec3[14 * 18];
		currentMesh = new glm::vec3[14 * 18];
		lastMesh = new glm::vec3[14 * 18];
		vertexArray = new Particle[14 * 18];

		for (int j = 0; j < ClothMesh::numRows; j++) {
			for (int i = 0; i < ClothMesh::numCols; i++) {
				if (i == 0 && j == 0) {
					currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5, 9.5, -4.5);
				}
				else if (i == ClothMesh::numCols - 1 && j == 0) {
					currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5, 9.5, -4.5 + 1.7f);
				}
				currentMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5 + j * 0.5f, 9.5, -4.5 + i * 0.5f);
				lastMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j * ClothMesh::numCols + i)];
				temporalMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j * ClothMesh::numCols + i)];
				outputMesh[(j * ClothMesh::numCols + i)] = currentMesh[(j * ClothMesh::numCols + i)];

				gravity = glm::vec3(0, -9.81, 0);

				vertexArray[j * ClothMesh::numCols + i].ActualPos = currentMesh[(j * ClothMesh::numCols + i)];
				vertexArray[j * ClothMesh::numCols + i].lastPos = lastMesh[(j * ClothMesh::numCols + i)];
				vertexArray[j * ClothMesh::numCols + i].Force = glm::vec3(0, 0, 0);
				vertexArray[j * ClothMesh::numCols + i].vel = glm::vec3(0, 0, 0);
				vertexArray[j * ClothMesh::numCols + i].alpha = 0;
			}
		}
		mesh_mode_sphere_pos_x = -4 + rand() % 8;
		mesh_mode_sphere_pos_y = 0 + rand() % 9;
		mesh_mode_sphere_pos_z = -4 + rand() % 8;
		mesh_mode_sphere_pos = glm::vec3(mesh_mode_sphere_pos_x, mesh_mode_sphere_pos_y, mesh_mode_sphere_pos_z);
		mesh_mode_sphere_radius = 0.5f + rand() % 3;
		Sphere::updateSphere(glm::vec3(mesh_mode_sphere_pos_x, mesh_mode_sphere_pos_y, mesh_mode_sphere_pos_z), mesh_mode_sphere_radius);
	}
	else if(fluidsMode) 
	{
		for (int j = 0; j < ClothMesh::numRows; j++) 
		{
			for (int i = 0; i < ClothMesh::numCols; i++) 
			{
				temporalMesh[(j * ClothMesh::numCols + i)] = glm::vec3(0.f, 0.f, 0.f);
				lastMesh[(j * ClothMesh::numCols + i)] = glm::vec3(0.f, 0.f, 0.f);

				initialMesh[(j * ClothMesh::numCols + i)] = glm::vec3(-4.5 + j * 0.5f, 0.f, -4.5 + i * 0.5f);
				currentMesh[(j * ClothMesh::numCols + i)] = initialMesh[(j * ClothMesh::numCols + i)];
			}
		}
		sphereX = -4 + rand() % 8;
		sphereY = 0 + rand() % 9;
		sphereZ = -4 + rand() % 8;
		centreSphere = glm::vec3(sphereX, sphereY, sphereZ);
		giantSphereRadius = 0.5f + rand() % 3;
	}
}

void ParticleSystem::updateParticle(int idx, glm::vec3 newPosition) {
	positions[idx] = newPosition;
}

void ParticleSystem::updateParticleVelocity(int idx, glm::vec3 newVelocity) {
	velocities[idx] = newVelocity;
}

glm::vec3 ParticleSystem::getParticleVelocity(int pos) {
	return velocities[pos];
}

glm::vec3 ParticleSystem::getParticleVector(int pos) {
	return positions[pos];
}

glm::vec3 ParticleSystem::getInitialParticleVector(int pos) {
	return InitialPos[pos];
}

glm::vec3 ParticleSystem::getNormalPlane(glm::vec3 initial, glm::vec3 finalV, glm::vec3 secondFv)
{
	glm::vec3 vec1 = finalV - initial;
	glm::vec3 vec2 = secondFv - initial;
	return glm::cross(vec1, vec2);
}

//EULER OLD METHOD (THERE'S ONLY ONE PLANE)
bool ParticleSystem::wallCheckCollision(int pos, glm::vec3 prevParticlePos) 
{
	glm::vec3 normal = glm::normalize(getNormalPlane(glm::vec3(-5.f, 0.f, -5.f), glm::vec3(-5.f, 0.f, 5.f), glm::vec3(5.f, 0.f, -5.f)));
	float Dp = (normal.x * -5.f + normal.y * 0.f + normal.z * -5.f) - 0.8;
	float dist = (abs(normal.x + normal.y + normal.z + Dp)) / sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
	if ((glm::dot(normal, prevParticlePos) + dist) * (glm::dot(normal, positions[pos]) + dist) <= 0)
	{
		updateParticle(pos, getParticleVector(pos) - (1 + 0.8f) * (glm::dot(normal, getParticleVector(pos) + dist) * normal));
		updateParticleVelocity(pos, getParticleVelocity(pos) - (1 + 0.8f) * (glm::dot(normal, getParticleVelocity(pos) + dist) * normal));
		return true;
	}

	//NONE
	return false;
}



glm::vec3 ParticleSystem::getPreviousVec(int pos) 
{
	return prevPositions[pos];
}

void ParticleSystem::setPreviousVec(int pos) 
{
	prevPositions[pos] = positions[pos];
}

void ParticleSystem::setPreviousVec(int pos, glm::vec3 tempPos) 
{
	prevPositions[pos] = tempPos;
}

bool ParticleSystem::checkCollision(int pos) {
	//walls
	if (positions[pos].x <= -5 || positions[pos].x >= 5 || positions[pos].z <= -5 || positions[pos].z >= 5)
	{
		return true;
	}
	//sphere
	if (particleCollisionsBetweenThem)
	{
		for (size_t i = 0; i < emissionRate; i++)
		{
			if (i != pos && sqrt(pow(positions[i].x - positions[pos].x, 2) + pow(positions[i].y - positions[pos].y, 2) + pow(positions[i].z - positions[pos].z, 2)) <= 0.2f)
			{
				return true;
			}
		}
	}
	//giant sphere
	if(simulateAGiantSphere) 
	{
		if (sqrt(pow(giantSpherePosition.x - positions[pos].x, 2) + pow(giantSpherePosition.y - positions[pos].y, 2) + pow(giantSpherePosition.z - positions[pos].z, 2)) <= giantSphereRadius)
			return true;
	}
	return false;
}

void ParticleSystem::sumPosParticle(int dx, glm::vec3 newPosition)
{
	positions[dx] += newPosition;
}

bool ParticleSystem::calculateBounce(int pos) {

	//COLLISION BETWEEN WALLS
	if (positions[pos].x <= -5 || positions[pos].x >= 5)
	{
		float vx = ((-getParticleVelocity(pos).x + (0 * 0.05f)));
		float vy = (getParticleVelocity(pos).y + (-9.81 * 0.05f));
		float vz = (getParticleVelocity(pos).z + (0 * 0.05f));

		float px = getParticleVector(pos).x - (getParticleVelocity(pos).x * 0.05f);
		float py = getParticleVector(pos).y + (getParticleVelocity(pos).y * 0.05f);
		float pz = getParticleVector(pos).z + (getParticleVelocity(pos).z * 0.05f);

		updateParticle(pos, glm::vec3(px, py, pz));

		updateParticleVelocity(pos, glm::vec3(vx, vy, vz));

		if (positions[pos].x < -5 || positions[pos].x > 5)
		{
			if (fountainMode)
			{
				updateParticle(pos, glm::vec3(0, 1, 0));
				updateParticleVelocity(pos, glm::vec3(cos(pos), 5, sin(pos)));
			}
			else if (cascadeMode)
			{
				updateParticle(pos, glm::vec3(0, 8, 0));
				positions[pos] = glm::vec3(rand() % 10 - 5, 8, 0);
			}
		}
		return true;
	}
	if (positions[pos].z <= -5 || positions[pos].z >= 5)
	{
		float vx = getParticleVelocity(pos).x + (0 * 0.05f);
		float vy = getParticleVelocity(pos).y + (-9.81 * 0.05f);
		float vz = -getParticleVelocity(pos).z + (0 * 0.05f);

		float px = getParticleVector(pos).x + (getParticleVelocity(pos).x * 0.05f);
		float py = getParticleVector(pos).y + (getParticleVelocity(pos).y * 0.05f);
		float pz = getParticleVector(pos).z - (getParticleVelocity(pos).z * 0.05f);

		updateParticle(pos, glm::vec3(px, py, pz));

		updateParticleVelocity(pos, glm::vec3(vx, vy, vz));

		if (positions[pos].z < -5 || positions[pos].z > 5)
		{
			if (fountainMode)
			{
				updateParticle(pos, glm::vec3(0, 1, 0));
				updateParticleVelocity(pos, glm::vec3(cos(pos), 5, sin(pos)));
			}
			else if (cascadeMode)
			{
				updateParticle(pos, glm::vec3(0, 8, 0));
				positions[pos] = glm::vec3(rand() % 10 - 5, 8, 0);
			}
		}
		return true;
	}
	//COLLISION WITH A GIANT SPHERE
	if (simulateAGiantSphere)
	{
			if (sqrt(pow(giantSpherePosition.x - positions[pos].x, 2) + pow(giantSpherePosition.y - positions[pos].y, 2) + pow(giantSpherePosition.z - positions[pos].z, 2)) <= giantSphereRadius)
			{
				float di = sqrt(pow(giantSpherePosition.x - positions[pos].x, 2) + pow(giantSpherePosition.y - positions[pos].y, 2) + pow(giantSpherePosition.z - positions[pos].z, 2));

				float nx = positions[pos].x;
				float ny = positions[pos].y;
				float nz = positions[pos].z;

				float nox = (giantSpherePosition.x - positions[pos].x) / di;
				float noy = (giantSpherePosition.y - positions[pos].y) / di;
				float noz = (giantSpherePosition.z - positions[pos].z) / di;

				float diPos = (nox * nx) + (noy * ny) + (noz * nz);

				diPos = -diPos;

				float vx2 = getParticleVelocity(pos).x + (0 * 0.05f);
				float vy2 = getParticleVelocity(pos).y + (-9.81 * 0.05f);
				float vz2 = getParticleVelocity(pos).z + (0 * 0.05f);

				float px2 = getParticleVector(pos).x + (getParticleVelocity(pos).x * 0.05f);
				float py2 = getParticleVector(pos).y + (getParticleVelocity(pos).y * 0.05f);
				float pz2 = getParticleVector(pos).z + (getParticleVelocity(pos).z * 0.05f);

				updateParticle(pos, glm::vec3(px2, py2, pz2));

				updateParticleVelocity(pos, glm::vec3(vx2, vy2, vz2));

				float mult1 = -2 * (((nox*positions[pos].x) + (noy*positions[pos].y) + (noz*positions[pos].z)) + diPos / di);

				float pf1x = (mult1 * nox);
				float pf1y = (mult1 * noy);
				float pf1z = (mult1 * noz);

				float pf2x = (positions[pos].x + pf1x);
				float pf2y = (positions[pos].y + pf1y);
				float pf2z = (positions[pos].z + pf1z);

				updateParticle(pos, glm::vec3(pf2x, pf2y, pf2z));

				float mult1_2 = -2 * (((nox*velocities[pos].x) + (noy*velocities[pos].y) + (noz*velocities[pos].z)) + diPos);

				float vf1x = (mult1_2 * nox);
				float vf1y = (mult1_2 * noy);
				float vf1z = (mult1_2 * noz);

				float vf2x = (velocities[pos].x + vf1x);
				float vf2y = (velocities[pos].y + vf1y);
				float vf2z = (velocities[pos].z + vf1z);

				updateParticleVelocity(pos, glm::vec3(vf2x, vf2y, vf2z));

				return true;
			}
	}
	if (particleCollisionsBetweenThem)
	{
		for (size_t i = 0; i < emissionRate; i++)
		{
			if (i != pos && sqrt(pow(positions[i].x - positions[pos].x, 2) + pow(positions[i].y - positions[pos].y, 2) + pow(positions[i].z - positions[pos].z, 2)) <= 0.2f)
			{
				float di = sqrt(pow(positions[i].x - positions[pos].x, 2) + pow(positions[i].y - positions[pos].y, 2) + pow(positions[i].z - positions[pos].z, 2));

				float nx = positions[pos].x;
				float ny = positions[pos].y;
				float nz = positions[pos].z;

				float nox = (positions[i].x - positions[pos].x) / di;
				float noy = (positions[i].y - positions[pos].y) / di;
				float noz = (positions[i].z - positions[pos].z) / di;

				float diPos = (nox * nx) + (noy * ny) + (noz * nz);

				diPos = -diPos;

				float vx2 = getParticleVelocity(pos).x + (0 * 0.05f);
				float vy2 = getParticleVelocity(pos).y + (-9.81 * 0.05f);
				float vz2 = getParticleVelocity(pos).z + (0 * 0.05f);

				float px2 = getParticleVector(pos).x + (getParticleVelocity(pos).x * 0.05f);
				float py2 = getParticleVector(pos).y + (getParticleVelocity(pos).y * 0.05f);
				float pz2 = getParticleVector(pos).z + (getParticleVelocity(pos).z * 0.05f);

				updateParticle(pos, glm::vec3(px2, py2, pz2));

				updateParticleVelocity(pos, glm::vec3(vx2, vy2, vz2));

				float mult1 = -2 * (((nox*positions[pos].x) + (noy*positions[pos].y) + (noz*positions[pos].z)) + diPos / di);

				float pf1x = (mult1 * nox);
				float pf1y = (mult1 * noy);
				float pf1z = (mult1 * noz);

				float pf2x = (positions[pos].x + pf1x);
				float pf2y = (positions[pos].y + pf1y);
				float pf2z = (positions[pos].z + pf1z);

				updateParticle(pos, glm::vec3(pf2x, pf2y, pf2z));

				float mult1_2 = -2 * (((nox*velocities[pos].x) + (noy*velocities[pos].y) + (noz*velocities[pos].z)) + diPos);

				float vf1x = (mult1_2 * nox);
				float vf1y = (mult1_2 * noy);
				float vf1z = (mult1_2 * noz);

				float vf2x = (velocities[pos].x + vf1x);
				float vf2y = (velocities[pos].y + vf1y);
				float vf2z = (velocities[pos].z + vf1z);

				updateParticleVelocity(pos, glm::vec3(vf2x, vf2y, vf2z));

				return true;
			}
		}
	}
	return false;
}

void ParticleSystem::calculateDrag() {
	subArea = abs(2 * 3.14150 * giantSphereRadius * diff);
	drag.y = -0.5 * density * subArea * glm::length(sphereSpeed) * sphereSpeed.y * 0.2;

}

void ParticleSystem::updateLilSpheres() {
	LilSpheres::firstParticleIdx = 0;
	LilSpheres::particleCount = emissionRate;

	LilSpheres::updateParticles(0, emissionRate, &(positions[0].x));
}

void ParticleSystem::spawnBigSphere(glm::vec3 position)
{
	Sphere::setupSphere(giantSpherePosition, giantSphereRadius);
	Sphere::drawSphere();
}

void ParticleSystem::spawnParticle(glm::vec3 position)
{
	updateParticle(currentParticles, position);

	currentParticles++;
}

void ParticleSystem::updateAge(float dt, int pos) {
	maxAge = lifeParticle;

	age[pos] += dt;
	if (age[pos] > maxAge)
	{
		if (fountainMode)
		{
			updateParticle(pos, glm::vec3(0, 1, 0));
			updateParticleVelocity(pos, glm::vec3(cos(pos), 10, sin(pos)));
		}
		else if (cascadeMode)
		{
			updateParticle(pos, glm::vec3(rand() % -5 + 10, 8, 0));
			updateParticleVelocity(pos, glm::vec3(rand() % 10 - 5, 0, rand() % 2 - 1));
		}
		age[pos] = 0;
	}
}

void ParticleSystem::destroyOldParticles(float maxAge) {
	for (int i = 0; i < currentParticles; i++)
	{
		if (age[i] > maxAge) {
			float x = positions[i].x;
			float z = positions[i].z;
			updateParticle(i, glm::vec3(x, 0, z));
		}
	}
}