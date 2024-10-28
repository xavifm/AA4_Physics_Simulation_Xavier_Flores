#pragma once
#include <glm/vec3.hpp>
//#include "CircularBuffer.h"
//#include <Particle.h>
#include <time.h>

class ParticleSystem {
public:
	//P2-P4 VARIABLES
	bool cleft;
	bool cright;
	bool cup;
	bool cdown;
	bool cfront;
	bool cback;
	bool fluidsMode = false;
	float damping = 50;
	float damping_2 = 900;
	float mesh_mode_sphere_pos_x;
	float mesh_mode_sphere_pos_y;
	float mesh_mode_sphere_pos_z;
	int mesh_mode_rows;
	int mesh_mode_cols;
	glm::vec3 mesh_mode_sphere_pos;
	float mesh_mode_sphere_radius;
	glm::vec3 gravity;
	float elongation = 0;
	bool resetMode = false;
	int runTime = 0;
	float distanceBetweenSprings = 0.5f;
	glm::vec3* temporalMesh = new glm::vec3[14 * 18];
	glm::vec3* outputMesh = new glm::vec3[14 * 18];
	glm::vec3* currentMesh = new glm::vec3[14 * 18];
	glm::vec3* lastMesh = new glm::vec3[14 * 18];
	//P4 VARIABLES:
	float mass;
	glm::vec3 centreSphere;
	glm::vec3 sphereSpeed;
	glm::vec3 lift;
	glm::vec3 drag;
	float K;
	float A;
	float lambda;
	float omega;
	float height;
	float density;
	float vsub;
	float subArea;
	glm::vec3 total;
	float diff;
	float sphereX;
	float sphereY;
	float sphereZ;
	glm::vec3* initialMesh;
	glm::vec3* parVerts;
	glm::vec3* q;
	glm::vec3 dir;
	time_t Time = time(0);
	void ParticleSystem::calculateDrag();
	//END OF P4 VARIABLES
	
	void ParticleSystem::calculateWithverletMethod(int i, int j, float dt);
	void ParticleSystem::resetPhysics();
	void ParticleSystem::meshSphereCollision(glm::vec3 Q, glm::vec3 &actPos, glm::vec3 &lastPos, glm::vec3 cS);
	void ParticleSystem::meshPlaneCollision(glm::vec3 n, float d, glm::vec3 &pos, glm::vec3 &prevPos);
	glm::vec3 ParticleSystem::getContactPointBetweenParticleAndSphere(glm::vec3 position, glm::vec3 lastP, glm::vec3 cS, float r);
	void ParticleSystem::updateVariables();
	void ParticleSystem::updateClothMesh(float* array_data);
	void ParticleSystem::resetModeAll();

	void ParticleSystem::resetModeF();
	void ParticleSystem::calculateForces(int i, int j);
	//__________________________________________


	//P1 VARIABLES
	bool fountainMode = false;
	bool cascadeMode = true;
	bool particleCollisionsBetweenThem = false;
	bool simulateAGiantSphere = true;
	bool meshMode = true;
	bool rigidBodyMode = false;
	float giantSphereRadius = 1;
	int emissionRate = 200; // >= 100 almenys
	float lifeParticle = 7; // >= 1 almenys;

	int currentParticles = 0;
	ParticleSystem(int numParticles = 200);
	void updateParticle(int dx, glm::vec3 newPosition);
	void updateLilSpheres();

	void ParticleSystem::sumPosParticle(int dx, glm::vec3 newPosition);

	void ParticleSystem::destroyOldParticles(float maxAge);

	void spawnParticle(glm::vec3 position);

	void ParticleSystem::updateAge(float dt, int pos);

	void ParticleSystem::updateParticleVelocity(int idx, glm::vec3 newPosition);

	glm::vec3 ParticleSystem::getParticleVector(int pos);

	glm::vec3 ParticleSystem::getParticleVelocity(int pos);
	glm::vec3 ParticleSystem::getNormalPlane(glm::vec3 initial, glm::vec3 final, glm::vec3 secondFv);
	glm::vec3 ParticleSystem::getInitialParticleVector(int pos);

	glm::vec3 ParticleSystem::getPreviousVec(int pos);

	void ParticleSystem::setPreviousVec(int pos);
	void ParticleSystem::setPreviousVec(int pos, glm::vec3 tempPos);

	glm::vec3 giantSpherePosition;

	void ParticleSystem::spawnBigSphere(glm::vec3 position);

	bool ParticleSystem::checkCollision(int pos);

	bool ParticleSystem::calculateBounce(int pos);
	bool ParticleSystem::wallCheckCollision(int pos, glm::vec3 prevParticlePos);

	bool enableLine;

	int numParticles;
	int maxAge = 5;
	float age[200];
private:
	glm::vec3* InitialPos;
	glm::vec3* positions;
	glm::vec3* velocities;
	glm::vec3* prevPositions;
};
//__________________________________________