#include <SDL2/SDL.h>
#include <iostream>
#include "RigidBody.hpp"
#include <vector>
#include <cmath>

// Function to check for collision between two rigid bodies
bool checkCollision(const RigidBody& body1, const RigidBody& body2) {
    return (body1.rect.x < body2.rect.x + body2.rect.w &&
            body1.rect.x + body1.rect.w > body2.rect.x &&
            body1.rect.y < body2.rect.y + body2.rect.h &&
            body1.rect.y + body1.rect.h > body2.rect.y);
}

// Helper function to calculate overlap between two colliding bodies
float calculateOverlap(const RigidBody& body1, const RigidBody& body2) {
    float overlapX = std::min(body1.rect.x + body1.rect.w, body2.rect.x + body2.rect.w) -
                     std::max(body1.rect.x, body2.rect.x);
    float overlapY = std::min(body1.rect.y + body1.rect.h, body2.rect.y + body2.rect.h) -
                     std::max(body1.rect.y, body2.rect.y);

    // Return the smaller of the two overlaps (negative if no overlap)
    return std::min(overlapX, overlapY);
}


void handleCollisionResponse(RigidBody& body1, RigidBody& body2) {
    // Calculate collision normal (unit vector pointing from body1 to body2)
    float dx = body2.positionX - body1.positionX;
    float dy = body2.positionY - body1.positionY;
    float length = std::sqrt(dx * dx + dy * dy);

    // If length is zero, the bodies are at the same position, avoid division by zero
    if (length == 0.0f) {
        return;
    }

    float collisionNormalX = dx / length;
    float collisionNormalY = dy / length;

    // Calculate relative velocity
    float relativeVelocityX = body2.velocityX - body1.velocityX;
    float relativeVelocityY = body2.velocityY - body1.velocityY;

    // Calculate relative velocity along the normal
    float relativeSpeedAlongNormal = (relativeVelocityX * collisionNormalX +
                                       relativeVelocityY * collisionNormalY);

    // If relative speed along the normal is positive, bodies are moving away, no response needed
    if (relativeSpeedAlongNormal > 0) {
        return;
    }

    // Calculate impulse (change in velocity) based on the coefficient of restitution
    float coefficientOfRestitution = 0.8f;  // Adjust as needed
    float impulse = -(1 + coefficientOfRestitution) * relativeSpeedAlongNormal /
                    (1 / body1.mass + 1 / body2.mass);

    // Apply impulse to adjust velocities
    body1.velocityX -= impulse / body1.mass * collisionNormalX;
    body1.velocityY -= impulse / body1.mass * collisionNormalY;

    body2.velocityX += impulse / body2.mass * collisionNormalX;
    body2.velocityY += impulse / body2.mass * collisionNormalY;

    // adjust the angular velocity
    if(abs(body1.angularVelocity) > abs(body2.angularVelocity)){
        body1.angularVelocity *= 0.5f;
        body2.angularVelocity = body1.angularVelocity * 0.5f;
    }
    else if(abs(body2.angularVelocity) > abs(body1.angularVelocity)){
        body2.angularVelocity *= 0.5f;
        body1.angularVelocity = body2.angularVelocity * 0.5f;
    }
    body1.angularVelocity *= 0.8f;
    body2.angularVelocity *= 0.8f;

    float overlap = calculateOverlap(body1, body2);
    float smallOverlap = 0.1f;  // Adjust as needed
    float correctionX = collisionNormalX * (std::max(overlap - smallOverlap, 0.0f) / (body1.mass + body2.mass));
    float correctionY = collisionNormalY * (std::max(overlap - smallOverlap, 0.0f) / (body1.mass + body2.mass));
    body1.positionX -= correctionX * body1.mass;
    body1.positionY -= correctionY * body1.mass;
    body2.positionX += correctionX * body2.mass;
    body2.positionY += correctionY * body2.mass;
}


int main() {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;

    SDL_Event event;

    const int windowWidth = 1280;
    const int windowHeight = 720;

    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_CreateWindowAndRenderer(windowWidth, windowHeight, 0, &window, &renderer);

    SDL_Surface* surface = SDL_CreateRGBSurface(0, 100, 100, 32, 0, 0, 0, 0);
    SDL_FillRect(surface, nullptr, SDL_MapRGBA(surface->format, 255, 255, 255, 255));
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

    // Initialize a vector of rigidBodies
    std::vector<RigidBody> rigidBodies;

    // Add multiple RigidBody objects to the vector
    rigidBodies.emplace_back(5.0f, 100.0f, 100.0f, 100.0f, 100.0f);
    rigidBodies.emplace_back(1.0f, 150.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 200.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 250.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 300.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 350.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 400.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 500.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 550.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 600.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 650.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 700.0f, 200.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 250.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 300.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 350.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 400.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 450.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 500.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 550.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 600.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 650.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 700.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 750.0f, 50.0f, 50.0f);
    rigidBodies.emplace_back(1.0f, 450.0f, 800.0f, 50.0f, 50.0f);



    bool running = true;
    while (running) {
        // eventListener?
        while (SDL_PollEvent(&event)) {
            // If window X clicked, close the program
            if (event.type == SDL_QUIT)
                running = false;
        }

        // Apply some test forces to the rigid body
        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_RIGHT]) {
            rigidBodies[0].applyForce(10.0f, 0.0f);
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_LEFT]) {
            rigidBodies[0].applyForce(-10.0f, 0.0f);
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_UP]) {
            rigidBodies[0].applyForce(0.0f, -10.0f);
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_DOWN]) {
            rigidBodies[0].applyForce(0.0f, 10.0f);
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_Q]) {
            rigidBodies[0].applyTorque(-1.0f);
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_E]) {
            rigidBodies[0].applyTorque(1.0f);
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_R]) {
            rigidBodies[0].reset();
        }

        if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_S]) {
            rigidBodies[0].stopForces();
        }


        // Update all RigidBody objects in the vector
        for (auto& body : rigidBodies) {
            body.update(0.01f);
        }

        // Check collision between all pairs of rigid bodies
        for (size_t i = 0; i < rigidBodies.size(); ++i) {
            for (size_t j = i + 1; j < rigidBodies.size(); ++j) {
                if (checkCollision(rigidBodies[i], rigidBodies[j])) {
                    handleCollisionResponse(rigidBodies[i], rigidBodies[j]);
                }       
            }
        }

        for (auto& body : rigidBodies) {
            // Left barrier
            if (body.positionX < 0) {
                body.leftBarrier();
            }

            // Right barrier
            if (body.positionX + body.width > windowWidth) {
                body.rightBarrier();
            }

            // Top barrier
            if (body.positionY < 0) {
                body.topBarrier();
            }

            // Bottom barrier
            if (body.positionY + body.height > windowHeight) {
                body.bottomBarrier();
            }
        }
        

        for (auto& body : rigidBodies) {
            // Air friction
            body.airFriction();
        }

        /*
        for (auto& body : rigidBodies) {
            // Gravity
            body.applyGravity(0.01f);
        }
        */

        // Draw a rectangle
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        for (auto& body : rigidBodies) {
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderCopyEx(renderer, texture, nullptr, &body.rect, body.getRotationDegrees(), nullptr, SDL_FLIP_NONE);
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(10);
    }

    SDL_FreeSurface(surface);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
