#include <glm/vec3.hpp>
//#include "CircularBuffer.h"

class Particle {
public:
	void distanceRegulation(Particle p);
	void calculateF(Particle nextP, float dist);
	glm::vec3 ActualPos;
	glm::vec3 lastPos;
	glm::vec3 vel;
	glm::vec3 Force;
	glm::vec3 Q;

	bool collided = true;
	bool calculated = false;
	float alpha;
	float damping = 50;
	float damping_2 = 900;
	float elongation = 0;
	int mass = 1;
};
