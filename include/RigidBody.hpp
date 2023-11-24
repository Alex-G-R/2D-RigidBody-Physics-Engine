#pragma once

#include <iostream>
#include <SDL2/SDL.h>
#include <cmath>

class RigidBody {
	public:
		float mass;
		float positionX, positionY;
		float width, height;
		float velocityX, velocityY;
		float angularVelocity;
		float rotation;
		SDL_Rect rect;

		RigidBody(float mass, float posX, float posY, float width, float height);

		float getRotationDegrees();
		void update(float deltaTime);
		void applyForce(float forceX, float forceY);
		void applyTorque(float torque);
		void stopForces();
		void reset();
		void printInfo();

		void leftBarrier();
		void rightBarrier();
		void bottomBarrier();
		void topBarrier();
		void airFriction();
		void applyGravity(float deltaTime);

};