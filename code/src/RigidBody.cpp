#include <glm/gtx/transform.hpp>

#include "RigidBody.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#pragma section forwardDeclarations
namespace Cube {
	extern void updateCube(const glm::mat4& transform);
}
namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
}
#pragma endregion

#pragma section RigidBody
RigidBody::RigidBody(float mass) : mass(mass) {};

void RigidBody::initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed) {
	// Initialize the state outside the constructor to use the virtual method getInitialInertiaTensor
	initialInertiaTensor = getInitialInertiaTensor();
	state = {
		initialPosition,
		initialRotation,
		mass * linearSpeed,
		angularSpeed * getInertiaTensor()
	};
}

RigidBody::State RigidBody::getState() {
	return state;
}

void RigidBody::setState(RigidBody::State newState) {
	state = newState;
}

RigidBody::State RigidBody::rollbackState() {
	// If the state is inconsistent, we can go back to the last correct state
	// (for example due to a collision)
	// Return the inconsistent state for cases where we want to use it
	State tmp = state;
	state = stableState;
	return tmp;
}

void RigidBody::commitState() {
	// Convert the state into a stable (correct) state
	stableState = state;
}

float RigidBody::getMass() {
	return mass;
}

void Box::moveRigidBody(glm::vec3 dir) 
{
	state.force = dir;
	state.com = glm::vec3(state.com.x + (state.speed.x * dt), state.com.y + (state.speed.y * dt), state.com.z + (state.speed.z * dt));
}

//PRIMITIVE OLD ROTATION SYSTEM FOR TESTING PURPOSES)
void Box::rotateRigidBody(float rotationX, float rotationY, float rotationZ)
{
	state.rotation = glm::vec3(rotationX, rotationY, rotationZ);
}

void Box::calculateImpulse(glm::vec3 normal, glm::vec3 colPoint)
{
	for (int i = 0; i < 8; i++) {
		if (normal.y > 0) {
			pC = abs(state.force);
			state.speed = state.force / getMass();
			for (int i = 0; i < 8; i++)
				torqueForce = glm::cross((colPoint - state.com), state.force);
			lC = torqueForce;
			wC = glm::inverse(iC)*torqueForce;
		}
		else 
		{
			pC = abs(state.force) * normal;
			state.speed = state.force / getMass();
			for (int i = 0; i < 8; i++)
				torqueForce = glm::cross((colPoint - state.com), state.force);
			lC = torqueForce;
			wC = glm::inverse(iC)*torqueForce;
		}
	}
}

void Box::setDeltaTime(float dt) 
{
	this->dt = dt;
}

void Box::addRotation()
{
	pC = pC + dt * state.force;
	state.speed = pC / getMass();
	xC = xC + dt * state.speed;

	iBodyC = glm::mat3((1.f / 12.f) * getMass()*(1 + 1));
	iBodyC = glm::inverse(iBodyC);
	rC = glm::mat3_cast(qC);
	iC = rC * iBodyC * glm::transpose(rC);
	wC = iC * lC;
	qC = qC + dt * 1.f / 2.f * glm::quat(0.f, wC)  * qC;
	qC = normalize(qC);
	qMat4 = mat4_cast(qC);
	state.rotation = qMat4;

}

void Box::setOldPos(glm::vec3 oldP) 
{
	oldPos = oldP;
}

bool Box::checkIntersectionBetweenPlanes(glm::vec3 groundPlane1, glm::vec3 groundPlane2, glm::vec3 groundPlane3, glm::vec3 boxPlane1, glm::vec3 boxPlane2, glm::vec3 boxPlane3, glm::vec3 boxPlane4, std::string colName)
{
	if(colName == "DownsideCubeWithGroundPlane") 
	{
		return (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane3.y >= boxPlane4.y || groundPlane3.y >= boxPlane1.y || groundPlane3.y >= boxPlane2.y || groundPlane3.y >= boxPlane3.y) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z);
	}
	if (colName == "UpperSideCubeWithUpperWall")
	{
		return (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane3.y <= boxPlane4.y || groundPlane3.y <= boxPlane1.y || groundPlane3.y <= boxPlane2.y || groundPlane3.y <= boxPlane3.y) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z);
	}
	if(colName == "LeftSideCubeWithLeftWall") 
	{
		return (groundPlane3.x >= boxPlane4.x || groundPlane3.x >= boxPlane1.x || groundPlane3.x >= boxPlane2.x || groundPlane3.x >= boxPlane3.x) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z);
	}
	if (colName == "RightSideCubeWithRightWall")
	{
		return (groundPlane3.x <= boxPlane4.x || groundPlane3.x <= boxPlane1.x || groundPlane3.x <= boxPlane2.x || groundPlane3.x <= boxPlane3.x) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z);
	}
	if (colName == "FrontSideCubeWithFrontWall")
	{
		return (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane2.z >= boxPlane4.z || groundPlane2.z >= boxPlane1.z || groundPlane2.z >= boxPlane2.z || groundPlane2.z >= boxPlane3.z);
	}
	if (colName == "BackSideCubeWithBackWall")
	{
		return (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane2.z <= boxPlane4.z || groundPlane2.z <= boxPlane1.z || groundPlane2.z <= boxPlane2.z || groundPlane2.z <= boxPlane3.z);
	}
	return false;
}

glm::vec3 Box::getIntersectionBetweenPlanes(glm::vec3 groundPlane1, glm::vec3 groundPlane2, glm::vec3 groundPlane3, glm::vec3 boxPlane1, glm::vec3 boxPlane2, glm::vec3 boxPlane3, glm::vec3 boxPlane4, std::string colName)
{
	if (colName == "DownsideCubeWithGroundPlane")
	{
		if( (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane3.y >= boxPlane4.y || groundPlane3.y >= boxPlane1.y || groundPlane3.y >= boxPlane2.y || groundPlane3.y >= boxPlane3.y) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z)) 
		{
			if (groundPlane3.y >= boxPlane4.y)
				return boxPlane4;
			if(groundPlane3.y >= boxPlane1.y)
				return boxPlane1;
			if(groundPlane3.y >= boxPlane2.y) 
				return boxPlane2;
			if(groundPlane3.y >= boxPlane3.y)
				return boxPlane3;
		}
	}
	if (colName == "UpperSideCubeWithUpperWall")
	{
		if ((groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane3.y <= boxPlane4.y || groundPlane3.y <= boxPlane1.y || groundPlane3.y <= boxPlane2.y || groundPlane3.y <= boxPlane3.y) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z)) 
		{
			if (groundPlane3.y <= boxPlane4.y)
				return boxPlane4;
			if (groundPlane3.y <= boxPlane1.y)
				return boxPlane1;
			if (groundPlane3.y <= boxPlane2.y)
				return boxPlane2;
			if (groundPlane3.y <= boxPlane3.y)
				return boxPlane3;
		}
	}
	if (colName == "LeftSideCubeWithLeftWall")
	{
		if ((groundPlane3.x >= boxPlane4.x || groundPlane3.x >= boxPlane1.x || groundPlane3.x >= boxPlane2.x || groundPlane3.x >= boxPlane3.x) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z)) 
		{
			if(groundPlane3.x >= boxPlane4.x)
				return boxPlane4;
			if(groundPlane3.x >= boxPlane1.x)
				return boxPlane1;
			if(groundPlane3.x >= boxPlane2.x)
				return boxPlane2;
			if(groundPlane3.x >= boxPlane3.x)
				return boxPlane3;
		}
	}
	if (colName == "RightSideCubeWithRightWall")
	{
		if( (groundPlane3.x <= boxPlane4.x || groundPlane3.x <= boxPlane1.x || groundPlane3.x <= boxPlane2.x || groundPlane3.x <= boxPlane3.x) &&
			(groundPlane1.z <= boxPlane2.z && groundPlane2.z >= boxPlane1.z)) 
		{
			if (groundPlane3.x <= boxPlane4.x)
				return boxPlane4;
			if (groundPlane3.x <= boxPlane1.x)
				return boxPlane1;
			if (groundPlane3.x <= boxPlane2.x)
				return boxPlane2;
			if (groundPlane3.x <= boxPlane3.x)
				return boxPlane3;
		}
	}
	if (colName == "FrontSideCubeWithFrontWall")
	{
		if( (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane2.z >= boxPlane4.z || groundPlane2.z >= boxPlane1.z || groundPlane2.z >= boxPlane2.z || groundPlane2.z >= boxPlane3.z)) 
		{
			if(groundPlane2.z >= boxPlane4.z)
				return boxPlane4;
			if(groundPlane2.z >= boxPlane1.z)
				return boxPlane1;
			if(groundPlane2.z >= boxPlane2.z) 
				return boxPlane2;
			if(groundPlane2.z >= boxPlane3.z)
				return boxPlane3;
		}
	}
	if (colName == "BackSideCubeWithBackWall")
	{
		if( (groundPlane1.x <= boxPlane3.x && groundPlane3.x >= boxPlane1.x) &&
			(groundPlane2.z <= boxPlane4.z || groundPlane2.z <= boxPlane1.z || groundPlane2.z <= boxPlane2.z || groundPlane2.z <= boxPlane3.z)) 
		{
			if (groundPlane2.z <= boxPlane4.z)
				return boxPlane4;
			if (groundPlane2.z <= boxPlane1.z)
				return boxPlane1;
			if (groundPlane2.z <= boxPlane2.z)
				return boxPlane2;
			if (groundPlane2.z <= boxPlane3.z)
				return boxPlane3;
		}
	}
	return glm::vec3(0,0,0);
}

bool Box::wallCheckCollision(glm::vec3 initial, glm::vec3 finalV, glm::vec3 secondFv, glm::vec3 normalGroundPlane, glm::vec3 prevParticlePos, std::string colName)
{
	bool collision = false;

	//DOWNSIDE BOX PLANE
	glm::vec3 groundPlanePos1 = state.com + glm::vec3(-width/2, -height/2, -depth/2) * glm::mat3_cast(state.rotation);
	glm::vec3 groundPlanePos2 = state.com + glm::vec3(-width/2, -height/2, depth/2) * glm::mat3_cast(state.rotation);
	glm::vec3 groundPlanePos3 = state.com + glm::vec3(width/2, -height/2, -depth/2) * glm::mat3_cast(state.rotation);
	glm::vec3 groundPlanePos4 = state.com + glm::vec3(width / 2, -height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	//UPSIDE BOX PLANE
	glm::vec3 upPlanePos1 = state.com + glm::vec3(-width / 2, height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 upPlanePos2 = state.com + glm::vec3(-width / 2, height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 upPlanePos3 = state.com + glm::vec3(width / 2, height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 upPlanePos4 = state.com + glm::vec3(width / 2, height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	//LEFT BOX PLANE
	glm::vec3 leftPlanePos1 = state.com + glm::vec3(-width/2, -height/2, -depth/2) * glm::mat3_cast(state.rotation);
	glm::vec3 leftPlanePos2 = state.com + glm::vec3(-width/2, height/2, depth/2) * glm::mat3_cast(state.rotation);
	glm::vec3 leftPlanePos3 = state.com + glm::vec3(-width/2, -height/2, depth/2) * glm::mat3_cast(state.rotation);
	glm::vec3 leftPlanePos4 = state.com + glm::vec3(-width / 2, height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	//RIGHT BOX PLANE
	glm::vec3 rightPlanePos1 = state.com + glm::vec3(width / 2, -height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 rightPlanePos2 = state.com + glm::vec3(width / 2, height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 rightPlanePos3 = state.com + glm::vec3(width / 2, -height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 rightPlanePos4 = state.com + glm::vec3(width / 2, height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	//FRONT BOX PLANE
	glm::vec3 frontPlanePos1 = state.com + glm::vec3(-width / 2, -height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 frontPlanePos2 = state.com + glm::vec3(-width / 2, height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 frontPlanePos3 = state.com + glm::vec3(width / 2, -height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 frontPlanePos4 = state.com + glm::vec3(-width / 2, height / 2, -depth / 2) * glm::mat3_cast(state.rotation);
	//BACK BOX PLANE
	glm::vec3 backPlanePos1 = state.com + glm::vec3(-width / 2, -height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 backPlanePos2 = state.com + glm::vec3(-width / 2, height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 backPlanePos3 = state.com + glm::vec3(width / 2, -height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	glm::vec3 backPlanePos4 = state.com + glm::vec3(-width / 2, height / 2, depth / 2) * glm::mat3_cast(state.rotation);
	//_____________________________________________________________

	if(checkIntersectionBetweenPlanes(initial, finalV, secondFv, groundPlanePos1, groundPlanePos2, groundPlanePos3, groundPlanePos4, colName))
	{
		calculateImpulse(normalGroundPlane, getIntersectionBetweenPlanes(initial, finalV, secondFv, groundPlanePos1, groundPlanePos2, groundPlanePos3, groundPlanePos4, colName));
		//AUXILIARY IMPULSE DETECTION BY NORMAL PLANE
		if (normalGroundPlane.y > 0)
			state.speed.y += 6.f;
		if (normalGroundPlane.y < 0)
			state.speed.y -= 6.f;
		if (normalGroundPlane.x > 0)
			state.speed.x += 6.f;
		if (normalGroundPlane.x < 0)
			state.speed.x -= 6.f;
		if (normalGroundPlane.z > 0)
			state.speed.z += 6.f;
		if (normalGroundPlane.z < 0)
			state.speed.z -= 6.f;
		//__________________________________________
		collision = true;
		return true;
	}

	if (checkIntersectionBetweenPlanes(initial, finalV, secondFv, upPlanePos1, upPlanePos2, upPlanePos3, upPlanePos4, colName))
	{
		calculateImpulse(normalGroundPlane, getIntersectionBetweenPlanes(initial, finalV, secondFv, upPlanePos1, upPlanePos2, upPlanePos3, upPlanePos4, colName));
		//AUXILIARY IMPULSE DETECTION BY NORMAL PLANE
		if (normalGroundPlane.y > 0)
			state.speed.y += 6.f;
		if (normalGroundPlane.y < 0)
			state.speed.y -= 6.f;
		if (normalGroundPlane.x > 0)
			state.speed.x += 6.f;
		if (normalGroundPlane.x < 0)
			state.speed.x -= 6.f;
		if (normalGroundPlane.z > 0)
			state.speed.z += 6.f;
		if (normalGroundPlane.z < 0)
			state.speed.z -= 6.f;
		//__________________________________________
		collision = true;
		return true;
	}

	if (checkIntersectionBetweenPlanes(initial, finalV, secondFv, leftPlanePos1, leftPlanePos2, leftPlanePos3, leftPlanePos4, colName))
	{
		calculateImpulse(normalGroundPlane, getIntersectionBetweenPlanes(initial, finalV, secondFv, leftPlanePos1, leftPlanePos2, leftPlanePos3, leftPlanePos4, colName));
		//AUXILIARY IMPULSE DETECTION BY NORMAL PLANE
		if (normalGroundPlane.y > 0)
			state.speed.y += 6.f;
		if (normalGroundPlane.y < 0)
			state.speed.y -= 6.f;
		if (normalGroundPlane.x > 0)
			state.speed.x += 6.f;
		if (normalGroundPlane.x < 0)
			state.speed.x -= 6.f;
		if (normalGroundPlane.z > 0)
			state.speed.z += 6.f;
		if (normalGroundPlane.z < 0)
			state.speed.z -= 6.f;
		//__________________________________________
		collision = true;
		return true;
	}

	if (checkIntersectionBetweenPlanes(initial, finalV, secondFv, rightPlanePos1, rightPlanePos2, rightPlanePos3, rightPlanePos4, colName))
	{
		calculateImpulse(normalGroundPlane, getIntersectionBetweenPlanes(initial, finalV, secondFv, rightPlanePos1, rightPlanePos2, rightPlanePos3, rightPlanePos4, colName));
		//AUXILIARY IMPULSE DETECTION BY NORMAL PLANE
		if (normalGroundPlane.y > 0)
			state.speed.y += 6.f;
		if (normalGroundPlane.y < 0)
			state.speed.y -= 6.f;
		if (normalGroundPlane.x > 0)
			state.speed.x += 6.f;
		if (normalGroundPlane.x < 0)
			state.speed.x -= 6.f;
		if (normalGroundPlane.z > 0)
			state.speed.z += 6.f;
		if (normalGroundPlane.z < 0)
			state.speed.z -= 6.f;
		//__________________________________________
		collision = true;
		return true;
	}

	if (checkIntersectionBetweenPlanes(initial, finalV, secondFv, backPlanePos1, backPlanePos2, backPlanePos3, backPlanePos4, colName))
	{
		calculateImpulse(normalGroundPlane, getIntersectionBetweenPlanes(initial, finalV, secondFv, backPlanePos1, backPlanePos2, backPlanePos3, backPlanePos4, colName));
		//AUXILIARY IMPULSE DETECTION BY NORMAL PLANE
		if (normalGroundPlane.y > 0)
			state.speed.y += 6.f;
		if (normalGroundPlane.y < 0)
			state.speed.y -= 6.f;
		if (normalGroundPlane.x > 0)
			state.speed.x += 6.f;
		if (normalGroundPlane.x < 0)
			state.speed.x -= 6.f;
		if (normalGroundPlane.z > 0)
			state.speed.z += 6.f;
		if (normalGroundPlane.z < 0)
			state.speed.z -= 6.f;
		//__________________________________________
		collision = true;
		return true;
	}

	if (checkIntersectionBetweenPlanes(initial, finalV, secondFv, frontPlanePos1, frontPlanePos2, frontPlanePos3, frontPlanePos4, colName))
	{
		calculateImpulse(normalGroundPlane, getIntersectionBetweenPlanes(initial, finalV, secondFv, frontPlanePos1, frontPlanePos2, frontPlanePos3, frontPlanePos4, colName));
		//AUXILIARY IMPULSE DETECTION BY NORMAL PLANE
		if (normalGroundPlane.y > 0)
			state.speed.y += 6.f;
		if (normalGroundPlane.y < 0)
			state.speed.y -= 6.f;
		if (normalGroundPlane.x > 0)
			state.speed.x += 6.f;
		if (normalGroundPlane.x < 0)
			state.speed.x -= 6.f;
		if (normalGroundPlane.z > 0)
			state.speed.z += 6.f;
		if (normalGroundPlane.z < 0)
			state.speed.z -= 6.f;
		//__________________________________________
		collision = true;
		return true;
	}
	return collision;
}

glm::mat3 RigidBody::getRotationMatrix() {
	return glm::mat3_cast(state.rotation);
}

glm::mat3 RigidBody::getInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}

#pragma endregion

#pragma region Box
Box::Box(float width, float height, float depth, float mass)
	: RigidBody(mass), width(width), height(height), depth(depth) 
{
	state = getState();
};

void Box::draw() {
	glm::mat4 transform = glm::translate(glm::mat4(1.f), state.com) *
		glm::mat4_cast(state.rotation) *
		glm::scale(glm::mat4(1.f), glm::vec3(width, height, depth));
	Cube::updateCube(transform);
}

glm::mat3 Box::getInitialInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}
#pragma endregion

#pragma region Ball
Ball::Ball(float radius, float mass) : RigidBody(mass), radius(radius) {};

void Ball::draw() {
	Sphere::updateSphere(getState().com, radius);
}

glm::mat3 Ball::getInitialInertiaTensor() {
	// TODO implement
	return glm::mat3(1.f);
}

#pragma endregion