#include <glm/vec3.hpp>
//#include "CircularBuffer.h"
#include "ParticleSystem.h"

class Particle : ParticleSystem {
public:
	void regulateDist(Particle p);
	void calculateForce(Particle nextP, float dist);
	glm::vec3 ActualPos;
	glm::vec3 lastPos;
	glm::vec3 vel;
	glm::vec3 Force;
	glm::vec3 Q;

	bool collided = true;
	bool calculated = false;
	float alpha;
	int mass = 1;
};
