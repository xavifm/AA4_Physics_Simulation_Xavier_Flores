#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <glm/gtc/type_ptr.hpp>

#pragma once
class RigidBody {
public:
	struct State {
		glm::vec3 com;  // Position of the Cenrer Of Mass
		glm::quat rotation;  // Quaternion that represents the current rotation q(t)
		glm::vec3 linearMomentum;  // P(t)
		glm::vec3 angularMomentum;  // L(t)
		glm::vec3 speed;
		glm::vec3 force;
	};

	RigidBody(float mass);
	void initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed);

	State getState();
	void setState(State state);
	State rollbackState();
	void commitState();

	float getMass();
	glm::mat3 getInertiaTensor();

	virtual void draw() = 0;
protected:
	glm::mat3 getRotationMatrix();
	virtual glm::mat3 getInitialInertiaTensor() = 0;
private:
	float mass;
	glm::mat3 initialInertiaTensor;
	State stableState;
	State state;
};

class Box : public RigidBody {
public:
	Box(float width, float height, float depth, float mass);
	virtual void draw() override;
	float rotX;
	float rotY;
	float rotZ;
	float dt;
	RigidBody::State state;
	//new voids
	glm::vec3 oldVelocity;
	glm::vec3 torqueForce;
	glm::vec3 oldPos;
	glm::mat3 IBody;
	glm::vec3 pC;
	glm::vec3 lC;
	glm::vec3 wC;
	glm::vec3 xC;
	glm::quat qC;
	glm::mat3 iBodyC;
	glm::mat3 rC;
	glm::mat3 iC;
	glm::mat3 qMat4;
	void Box::setDeltaTime(float dt);
	void Box::calculateImpulse(glm::vec3 normal, glm::vec3 colPoint);
	void Box::setOldPos(glm::vec3 oldP);
	void Box::addRotation();
	void Box::moveRigidBody(glm::vec3 dir);
	void Box::rotateRigidBody(float rotationX, float rotationY, float rotationZ);
	bool Box::wallCheckCollision(glm::vec3 initial, glm::vec3 finalV, glm::vec3 secondFv, glm::vec3 normalGroundPlane, glm::vec3 prevParticlePos, std::string colName);
	bool Box::checkIntersectionBetweenPlanes(glm::vec3 groundPlane1, glm::vec3 groundPlane2, glm::vec3 groundPlane3, glm::vec3 boxPlane1, glm::vec3 boxPlane2, glm::vec3 boxPlane3, glm::vec3 boxPlane4, std::string colName);
	glm::vec3 Box::getIntersectionBetweenPlanes(glm::vec3 groundPlane1, glm::vec3 groundPlane2, glm::vec3 groundPlane3, glm::vec3 boxPlane1, glm::vec3 boxPlane2, glm::vec3 boxPlane3, glm::vec3 boxPlane4, std::string colName);
	//________________________________
protected:
	virtual glm::mat3 getInitialInertiaTensor() override;
private:
	float width, height, depth;
};

class Ball : public RigidBody {
public:
	Ball(float radius, float mass);
	virtual void draw() override;
protected:
	virtual glm::mat3 getInitialInertiaTensor() override;
private:
	float radius;
};