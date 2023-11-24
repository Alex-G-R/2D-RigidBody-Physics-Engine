#include "RigidBody.hpp"

RigidBody::RigidBody(float mass, float posX, float posY, float w, float h)
:mass(mass), positionX(posX), positionY(posY), width(w), height(h), velocityX(0.0f), velocityY(0.0f), angularVelocity(0.0f), rotation(0.0f),
rect{static_cast<int>(posX), static_cast<int>(posY), static_cast<int>(w), static_cast<int>(h)} {} 


void RigidBody::update(float deltaTime){
	// update position based on velocity
	positionX += velocityX * deltaTime;
	positionY += velocityY * deltaTime;

	// update rotation based on angular velocity
	rotation += angularVelocity * deltaTime;

	// Update SDL_Rect
    rect.x = static_cast<int>(positionX);
    rect.y = static_cast<int>(positionY);
    rect.w = static_cast<int>(width);
    rect.h = static_cast<int>(height);
}

float RigidBody::getRotationDegrees() {
    return rotation * 180.0f / M_PI;
}


void RigidBody::applyForce(float forceX, float forceY){
	// apply force based on mass
	velocityX += forceX / mass;
	velocityY += forceY / mass;
}

void RigidBody::applyTorque(float torque) {
    // torque based on mass
    angularVelocity += torque / mass;
}

void RigidBody::stopForces(){
	velocityX = 0;
	velocityY = 0;
	angularVelocity = 0;
}

void RigidBody::reset(){
	stopForces();
	rotation = 0;
	positionX = 10;
	positionY = 10;
}

void RigidBody::printInfo() {
    std::cout << "Position: (" << positionX << ", " << positionY << ")\n";
    std::cout << "Velocity: (" << velocityX << ", " << velocityY << ")\n";
    std::cout << "Rotation: " << rotation << "\n";
}


void RigidBody::leftBarrier(){
	positionX += width/12;
    velocityX *= -0.8f;  // Bounce back with reduced speed
    angularVelocity *= -0.8f;  // Reduce angular speed
}
void RigidBody::rightBarrier(){
	positionX -= width/12;
    velocityX *= -0.8f;  // Bounce back with reduced speed
    angularVelocity *= -0.8f;  // Reduce angular speed
}
void RigidBody::bottomBarrier(){
	positionY -= height/12;
    velocityY *= -0.8f;  // Bounce back with reduced speed
    angularVelocity *= -0.8f;  // Reduce angular speed
}
void RigidBody::topBarrier(){
	positionY += height/12;
    velocityY *= -0.8f;  // Bounce back with reduced speed
    angularVelocity *= -0.8f;  // Reduce angular speed
}
void RigidBody::airFriction(){
	velocityY *= 0.999f;  
    velocityX *= 0.999f;  
    angularVelocity *= 0.999f;
}

void RigidBody::applyGravity(float deltaTime) {
    const float gravitationalAcceleration = 9.8; // in m/s^2

    // Calculate the force of gravity
    float gravityForce = mass * gravitationalAcceleration;

    // Calculate the acceleration
    float acceleration = gravityForce / mass;

    // Update the velocity based on the acceleration and time step
    velocityY += acceleration * deltaTime;
}