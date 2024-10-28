#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm/\glm.hpp>
#include <../include/GLFW/glfw3.h>
#include "ParticleSystem.h";
#include "Rigidbody.h";
#include <iostream>;

extern bool renderParticles;
extern bool renderSphere;
extern bool renderCloth;
extern bool renderCube;

namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
}

bool rigidBodyMovementEnabled;
glm::vec3 oldPos;

namespace LilSpheres {
	extern const int maxParticles;
}
bool show_test_window = false;

ParticleSystem ps;
RigidBody* rb;
Box box = Box(2, 2, 2, 5);

int nextParticleIdx = 0;
float initialAngle = 0.0f;
float emissionRate = 1.f;
float maxAge = 5.f;
float timerEnd = 0.0f;
float torque;
float startTime;
int accelerationX = 0.0f;
int accelerationZ = 0.0f;
bool startFluidsEnabled;

void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		if(!ps.rigidBodyMode && ImGui::Button("enable RigidBodyMode")) 
		{
			box.state.com = glm::vec3(0, 4, 0);
			box.state.speed = glm::vec3(0, 0, 0);
			ps.rigidBodyMode = true;
			if(ps.meshMode) 
			{
				ps.resetPhysics();
				renderSphere = false;
				renderCloth = false;
			}
			else 
			{
				ps.meshMode = true;
			}
			ps.resetModeF();
			renderCube = true;
		}
		else if(ps.rigidBodyMode && ImGui::Button("disable RigidBodyMode")) 
		{
			ps.rigidBodyMode = false;
			if(ps.meshMode) 
			{
				renderCloth = true;
			}
			renderCube = false;
		}

		if(!ps.rigidBodyMode) 
		{
			if (ps.meshMode && ImGui::Button("enable particle Mode"))
			{
				ps.meshMode = false;
				renderCloth = false;
				ps.resetPhysics();
				ps.resetModeF();
			}
			else if (!ps.meshMode && ImGui::Button("enable mesh Mode"))
			{
				ps.meshMode = true;
				renderCloth = true;
				ps.resetModeF();
			}

			if (ps.meshMode)
			{
				if(!ps.fluidsMode && ImGui::Button("enable fluids mode"))
				{
					ps.fluidsMode = true;
					ps.resetModeF();
				}
				else if (ps.fluidsMode && ImGui::Button("disable fluids mode"))
				{
					ps.fluidsMode = false;
					ps.resetModeF();
				}

				if(!ps.fluidsMode) 
				{
					ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
					if (ImGui::Button("restart mesh simulation")) {
						ps.resetMode = true;
					}
					ImGui::SliderFloat("damping of direct-link springs", &ps.damping, 1, 53);
					ImGui::SliderFloat("damping of diagonal link springs", &ps.damping_2, 0, 1000);
					ImGui::SliderFloat("Initial Rest distance of the springs between the points of the mesh", &ps.distanceBetweenSprings, 0.3, 0.7);
					ImGui::SliderFloat("elongation", &ps.elongation, -0.5, 0.5);
				}
				else if(ps.fluidsMode)
				{
					ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
					if (ImGui::Button("restart mesh simulation")) {
						//ps.resetMode = true;
						ps.resetModeF();
					}
					ImGui::SliderFloat("Amplitude", &ps.A, 0.1, 1.5);
					ImGui::SliderFloat("Frequency", &ps.omega, 1, 10);
					ImGui::SliderFloat("Direction x", &ps.dir.x, 0, 1);
					ImGui::SliderFloat("Direction y", &ps.dir.y, 0, 1);
					ImGui::SliderFloat("Direction z", &ps.dir.z, 0, 1);
					ImGui::SliderFloat("Height", &ps.height, 0.5, 10);
					ImGui::SliderFloat("Mass", &ps.mass, 0.1, 10);
					ImGui::SliderFloat("Liquid's density", &ps.density, 1, 30);
				}
			}
			else
			{
				if (ImGui::Button("fountainMode"))
				{
					ps.fountainMode = true;
					ps.cascadeMode = false;
				}
				if (ImGui::Button("cascadeMode"))
				{
					ps.fountainMode = false;
					ps.cascadeMode = true;
				}
				if (!ps.simulateAGiantSphere && ImGui::Button("GiantSphereSimulate"))
				{
					ps.simulateAGiantSphere = true;
				}
				else if (ps.simulateAGiantSphere && ImGui::Button("stop GiantSphere simulation"))
				{
					ps.simulateAGiantSphere = false;
				}
				ImGui::SliderFloat("giantSphereRadius", &ps.giantSphereRadius, 0, 10);
				ImGui::SliderFloat("giantSpherePosx", &ps.giantSpherePosition.x, -2, 2);
				ImGui::SliderFloat("giantSpherePosy", &ps.giantSpherePosition.y, 0, 1.5f);
				ImGui::SliderFloat("giantSpherePosz", &ps.giantSpherePosition.z, -2, 2);
				if (!ps.particleCollisionsBetweenThem && ImGui::Button("enableParticleCollisions (SPHERE COLLISIONS)"))
				{
					ps.particleCollisionsBetweenThem = true;
				}
				else if (ps.particleCollisionsBetweenThem && ImGui::Button("disableParticleCollisions (ONLY WALL COLLISIONS)"))
				{
					ps.particleCollisionsBetweenThem = false;
				}

				ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
				if (ImGui::SliderInt("Emission rate", &ps.emissionRate, 1, 200))
				{
					ps.resetModeAll();
				}
				ImGui::SliderFloat("particle life", &ps.lifeParticle, 7, 20);
			}
		}
		else
		{
			//ALL RIGIDBODY PARAMETERS
			if (ImGui::Button("Reset Rigidbody Simulation"))
			{
				oldPos = glm::vec3(0, 0, 0);
				box.state.com = glm::vec3(-4 + rand() % 8, 5, -4 + rand() % 8);
				accelerationX = -10 + rand() % 20;
				accelerationZ = -10 + rand() % 20;
				box.state.rotation = glm::quat(0, 0, 0, 0);
				box.torqueForce = glm::vec3(0, 0, 0);
				torque = 1.f;
				box.lC = glm::vec3(0, 0, 0);
				box.lC = box.lC + torque;
				timerEnd = 0;
			}
		}
	}

	ImGui::End();
}

void PhysicsInit() {
	renderCloth = true;
	renderParticles = true;
	renderCube = false;
	ps = ParticleSystem(200);
	rb = &box;
	box.state.com = glm::vec3(-3 + rand() % 8, 5, -3 + rand() % 8);
	accelerationX = -10 + rand() % 20;
	accelerationZ = -10 + rand() % 20;
	torque = 1.f;
	box.lC = box.lC + torque;
	//P4 INITIALIZATIONS
	ps.mass = 10;
	ps.dir.x = 1;
	ps.sphereSpeed = glm::vec3(0.f, 0.f, 0.f);
	ps.lambda = 2;
	ps.K = (2 * 3.14159) / ps.lambda;
	ps.A = 0.4;
	ps.omega = 6;
	ps.height = 1;
	ps.density = 1.7;
	ps.vsub = 0;
	ps.subArea = 0;
	ps.gravity = glm::vec3(0, -9.81, 0);
	ps.drag = glm::vec3(0, 0, 0);
	ps.lift = glm::vec3(0, 0, 0);
	ps.total = ps.gravity;
	ps.diff = 0;
	ps.initialMesh = new glm::vec3[14 * 18];
	ps.temporalMesh = new glm::vec3[14 * 18];
	ps.lastMesh = new glm::vec3[14 * 18];
	ps.currentMesh = new glm::vec3[14 * 18];
	//ps.parVerts = new Particle[14 * 18];
	ps.q = new glm::vec3[14 * 18];
	//END OF P4 INITIALIZATIONS
}

void spawn() {
}

void PhysicsUpdate(float dt) {

	if (!ps.meshMode && !ps.rigidBodyMode)
	{
		if (ps.simulateAGiantSphere)
		{
			renderSphere = true;
			ps.spawnBigSphere(ps.giantSpherePosition);
		}
		else
		{
			renderSphere = false;
		}

		float angle = 0.f;

		for (int i = 0; i < ps.emissionRate; i++)
		{
			//EULER METHOD (P1)
			if (ps.wallCheckCollision(i, ps.getPreviousVec(i)))
			{
				ps.updateAge(0.033f, i);
			}
			if (ps.checkCollision(i))
			{
				ps.setPreviousVec(i);
				if (ps.calculateBounce(i))
				{
					ps.updateAge(0.033f, i);
				}
			}
			else
			{
				ps.setPreviousVec(i);

				float vx = ps.getParticleVelocity(i).x + (0 * 0.05f);
				float vy = ps.getParticleVelocity(i).y + (-9.81 * 0.05f);
				float vz = ps.getParticleVelocity(i).z + (0 * 0.05f);

				float px = ps.getParticleVector(i).x + (ps.getParticleVelocity(i).x * 0.05f);
				float py = ps.getParticleVector(i).y + (ps.getParticleVelocity(i).y * 0.05f);
				float pz = ps.getParticleVector(i).z + (ps.getParticleVelocity(i).z * 0.05f);

				ps.updateParticle(i, glm::vec3(px, py, pz));

				ps.updateParticleVelocity(i, glm::vec3(vx, vy, vz));

				ps.updateAge(0.033f, i);
			}
		}
	}
	else if(!ps.rigidBodyMode)
	{
		//MESH MODE ENABLED (VERLET MODE) (P2-P4)
		if(!ps.fluidsMode) 
		{
			if(!startFluidsEnabled)
			startTime += dt;
			ps.updateVariables();
			renderSphere = true;
			dt /= 10;
			for (int i = 0; i < 10; i++)
			{
				for (int z = 0; z < ps.mesh_mode_rows; z++)
				{
					for (int o = 0; o < ps.mesh_mode_cols; o++)
					{
						ps.calculateWithverletMethod(o, z, dt);
						ps.calculateForces(o, z);
					}
				}
			}
			ps.resetModeAll();
			ps.updateClothMesh(&ps.currentMesh[0].x);
			//ENABLING FLUIDS MODE
			if(startTime >= 5 && !startFluidsEnabled)
			{
				startFluidsEnabled = true;
				startTime = 0;
				ps.fluidsMode = true;
				ps.resetModeF();
			}
		}
		else 
		{
			renderSphere = true;
			glm::vec3 buoyancy = ps.density * ps.vsub * 9.81f * glm::vec3(0.f, 1.f, 0.f);
			ps.total = (ps.gravity + buoyancy + ps.drag) * ps.mass;
			ps.sphereSpeed.y = ps.sphereSpeed.y + dt * (ps.total.y / ps.mass);

			ps.sphereY = ps.sphereY + dt * ps.sphereSpeed.y;
			for (int j = 0; j < ps.mesh_mode_rows; j++) 
			{
				for (int i = 0; i < ps.mesh_mode_cols; i++) 
				{
					ps.temporalMesh[j * ps.mesh_mode_cols + i] = ps.currentMesh[j * ps.mesh_mode_cols + i];
					ps.currentMesh[j * ps.mesh_mode_cols + i] = ps.initialMesh[j * ps.mesh_mode_cols + i] - (ps.dir / ps.K) * ps.A * sin(glm::dot(ps.dir, (ps.initialMesh[j * ps.mesh_mode_cols + i]) - ps.omega * ps.Time));
					ps.currentMesh[j * ps.mesh_mode_cols + i].y = ps.height + ps.A * cos(glm::dot(ps.dir, ps.currentMesh[j * ps.mesh_mode_cols + i]) - ps.omega * ps.Time);
					ps.lastMesh[j * ps.mesh_mode_cols + i] = ps.temporalMesh[j * ps.mesh_mode_cols + i];
					ps.q[j * ps.mesh_mode_cols + i] = ps.getContactPointBetweenParticleAndSphere(ps.currentMesh[(j * ps.mesh_mode_cols + i)], ps.lastMesh[(j * ps.mesh_mode_cols + i)], ps.centreSphere, ps.giantSphereRadius);
					ps.diff = abs(ps.sphereY - ps.giantSphereRadius - ps.currentMesh[j * ps.mesh_mode_cols + i].y);

					if (ps.sphereY - ps.giantSphereRadius <= ps.currentMesh[j * ps.mesh_mode_cols + i].y) 
					{
						ps.vsub = (ps.giantSphereRadius * 2) * ps.diff;
						ps.calculateDrag();
					}
					else 
					{
						ps.vsub = 0;
					}

					//ps.Time += dt;
				}
			}
			ps.resetModeAll();

			Sphere::updateSphere(glm::vec3(ps.sphereX, ps.sphereY, ps.sphereZ), ps.giantSphereRadius);
			ps.resetModeAll();
			ps.updateClothMesh(&ps.currentMesh[0].x);
		}
	}
	else 
	{
		//RIGIDBODY MODE (P3)
		timerEnd += dt;
		if(timerEnd >= 15) 
		{
			oldPos = glm::vec3(0, 0, 0);
			box.state.com = glm::vec3(-4 + rand() % 8, 5, -4 + rand() % 8);
			accelerationX = -10 + rand() % 20;
			accelerationZ = -10 + rand() % 20;
			box.state.rotation = glm::quat(0, 0, 0, 0);
			box.torqueForce = glm::vec3(0, 0, 0);
			torque = 1.f;
			box.lC = glm::vec3(0, 0, 0);
			box.lC = box.lC + torque;
			timerEnd = 0;
		}
		else
		{
			box.setDeltaTime(dt);
			oldPos = box.state.com;
			box.setOldPos(oldPos);
			box.oldVelocity = box.state.speed;
			box.moveRigidBody(glm::vec3(accelerationX, -9.8, accelerationZ));
			
			//COLLISION WITH FLOOR
			std::string colName = "DownsideCubeWithGroundPlane";
			glm::vec3 initial = glm::vec3(-5.f, 0.f, -5.f);
			glm::vec3 finalV = glm::vec3(-5.f, 0.f, 5.f);
			glm::vec3 secondFv = glm::vec3(5.f, 0.f, -5.f);
			glm::vec3 normalGroundPlane = glm::vec3(0, 1, 0);
			float Dp = 0;
			
			if (box.wallCheckCollision(initial, finalV, secondFv, normalGroundPlane, oldPos, colName))
			{
				std::cout << "COLLISION!" << std::endl;
			}

			//COLLISION WITH ROOF
			colName = "UpperSideCubeWithUpperWall";
			initial = glm::vec3(-5.f, 10.f, -5.f);
			finalV = glm::vec3(-5.f, 10.f, 5.f);
			secondFv = glm::vec3(5.f, 10.f, -5.f);
			normalGroundPlane = glm::vec3(0, -1, 0);
			Dp = 10;

			if (box.wallCheckCollision(initial, finalV, secondFv, normalGroundPlane, oldPos, colName))
			{
				std::cout << "COLLISION!" << std::endl;
			}

			//COLLISION WITH LEFT WALL
			colName = "LeftSideCubeWithLeftWall";
			initial = glm::vec3(-5.f, 0.f, -5.f);
			finalV = glm::vec3(-5.f, 10.f, 5.f);
			secondFv = glm::vec3(-5.f, 0.f, 5.f);
			normalGroundPlane = glm::vec3(1, 0, 0);
			Dp = 5;

			if (box.wallCheckCollision(initial, finalV, secondFv, normalGroundPlane, oldPos, colName))
			{
				std::cout << "COLLISION!" << std::endl;
			}

			//COLLISION WITH RIGHT WALL
			colName = "RightSideCubeWithRightWall";
			initial = glm::vec3(5.f, 0.f, -5.f);
			finalV = glm::vec3(5.f, 10.f, 5.f);
			secondFv = glm::vec3(5.f, 0.f, 5.f);
			normalGroundPlane = glm::vec3(-1, 0, 0);
			Dp = 5;

			if (box.wallCheckCollision(initial, finalV, secondFv, normalGroundPlane, oldPos, colName))
			{
				std::cout << "COLLISION!" << std::endl;
			}

			//COLLISION WITH FRONT WALL
			colName = "BackSideCubeWithBackWall";
			initial = glm::vec3(-5.f, 0.f, 5.f);
			finalV = glm::vec3(5.f, 10.f, 5.f);
			secondFv = glm::vec3(5.f, 0.f, 5.f);
			normalGroundPlane = glm::vec3(0, 0, -1);
			Dp = 5;

			if (box.wallCheckCollision(initial, finalV, secondFv, normalGroundPlane, oldPos, colName))
			{
				std::cout << "COLLISION!" << std::endl;
			}

			//COLLISION WITH BACK WALL
			colName = "FrontSideCubeWithFrontWall";
			initial = glm::vec3(-5.f, 0.f, -5.f);
			finalV = glm::vec3(5.f, 10.f, -5.f);
			secondFv = glm::vec3(5.f, 0.f, -5.f);
			normalGroundPlane = glm::vec3(0, 0, 1);
			Dp = 5;

			if (box.wallCheckCollision(initial, finalV, secondFv, normalGroundPlane, oldPos, colName))
			{
				std::cout << "COLLISION!" << std::endl;
			}

			box.addRotation();
			box.draw();
		}
	}

	if (nextParticleIdx < ps.emissionRate && !ps.rigidBodyMode) {
		spawn();
	}

	if(!ps.rigidBodyMode)
	ps.updateLilSpheres();
}

void PhysicsCleanup() {
	//Exemple_PhysicsCleanup();
}



