/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"
#include <string>
#include <vector>

const unsigned int TARGET_FPS = 50;
float dt = 1.0f / TARGET_FPS;
float time = 0;
float restitution = 0.9f;
bool isBirdSpawned = false;
float spawnRadius = 10.0f;
float slingshotRadius = 100.0f;
float birdIndex = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////     Physics Shapes     ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum PhysicsShape
{
    CIRCLE,
    RECT,
    HALF_SPACE
};

float speed = 100;
float angle = 0;

// Physics Object Base
class PhysicsObj
{
public:
    bool isStatic = false;
    Vector2 position = { 0, 0 };
    Vector2 velocity = { 0, 0 };
    Vector2 netForce = { 0, 0 };
    float mass = 1;
    float grippiness = 0.5f; // for determining coefficient of friction
    float bounciness = 0.9f; // for determining coefficient of restitution

    Color color = GREEN;

    virtual void draw()
    {
        DrawCircle(position.x, position.y, 2, color);
    }

    virtual PhysicsShape shape() = 0;
};

// Physics Object Circle
class PhysicsCircle : public PhysicsObj
{
public:
    float radius = 10;

    void draw() override
    {
        DrawCircle(position.x, position.y, radius, color);
    }

    PhysicsShape shape() override
    {
        return CIRCLE;
    }
};

class PhysicsRect : public PhysicsObj
{
public:
    Vector2 size = { 10, 10 };

    void draw() override
    {
        Vector2 offsPos = position - (size / 2);
        DrawRectangle(offsPos.x, offsPos.y, size.x, size.y, color);
    }

    PhysicsShape shape() override
    {
        return RECT;
    }
};

// Physics Object Halfspace
class PhysicsHalfspace : public PhysicsObj
{
private:
    float rotation = 0;
    Vector2 normal = { 0, -1 };

public:
    void SetRotationInDeg(float rotationInDeg)
    {
        rotation = rotationInDeg;
        normal = Vector2Rotate({ 0, -1 }, rotation * DEG2RAD);
    }

    float GetRotationInDeg()
    {
        return rotation;
    }

    Vector2 GetNormal()
    {
        return normal;
    }


    void draw() override
    {
        DrawCircle(position.x, position.y, 8, color);

        DrawLineEx(position, position + normal * 30, 1, color);

        Vector2 parallelToSurface = Vector2Rotate(normal, PI * 0.5f);
        DrawLineEx(position - parallelToSurface * 4000, position + parallelToSurface * 4000, 1, color);
    }

    PhysicsShape shape() override
    {
        return HALF_SPACE;
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////     Physics World      ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CircleCircleOverlap(PhysicsCircle* circleA, PhysicsCircle* circleB);
bool CircleCircleCollisionCheck(PhysicsCircle* circleA, PhysicsCircle* circleB);
bool CircleHalfspaceOverlap(PhysicsCircle* circle, PhysicsHalfspace* halfspace);
bool CircleHalfspaceCollisionCheck(PhysicsCircle* circle, PhysicsHalfspace* halfspace);

bool RectangleAABBCollision(PhysicsRect* rectA, PhysicsRect* rectB);
bool RectHalfspaceCollisionCheck(PhysicsRect* rect, PhysicsHalfspace* halfspace);
bool RectangleCircleCollision(PhysicsRect* rect, PhysicsCircle* circle);

class PhysicsWorld
{
public:
    std::vector<PhysicsObj*> objects;
    PhysicsObj* activeBird;
    Vector2 accelGravity = { 0, 9 };
    Vector2 startPos = { 105, 510 };

    void add(PhysicsObj* newObj)
    {
        objects.push_back(newObj);
    }

    void resetNetForces()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            PhysicsObj* object = objects[i];

            object->netForce = { 0, 0 };
        }

        if (activeBird != nullptr)
        {
            activeBird->netForce = { 0, 0 };
        }
    }

    void addGravityForces()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            PhysicsObj* object = objects[i];

            if (object->isStatic) continue;

            Vector2 FGravity = accelGravity * object->mass;
            object->netForce += FGravity;

            DrawLineEx(object->position, object->position + FGravity, 3, PURPLE);
        }

        if (activeBird != nullptr)
        {
            Vector2 FGravity = accelGravity * activeBird->mass;
            activeBird->netForce += FGravity;
        }
    }

    void applyKinematics()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            PhysicsObj* object = objects[i];

            if (object->isStatic) continue;

            object->position = object->position + object->velocity * dt;

            Vector2 acceleration = object->netForce/object->mass;

            object->velocity = object->velocity + acceleration * dt;

            DrawLineEx(object->position, object->position + object->velocity, 3, RED);
        }

        if (activeBird != nullptr)
        {
            activeBird->position = activeBird->position + activeBird->velocity * dt;

            Vector2 birdAcceleration = activeBird->netForce / activeBird->mass;

            activeBird->velocity = activeBird->velocity + birdAcceleration * dt;
        }
    }

    void update()
    {
        resetNetForces();

        addGravityForces();

        collisionCheck();

        applyKinematics();
    }

    void collisionCheck()
    {
        for (int i = 0; i < objects.size(); i++)
        {

            for (int j = i + 1; j < objects.size(); j++)
            {
                PhysicsObj* objectPointerA = objects[i];
                PhysicsObj* objectPointerB = objects[j];

                PhysicsShape shapeOfA = objectPointerA->shape();
                PhysicsShape shapeOfB = objectPointerB->shape();



                if (shapeOfA == CIRCLE && shapeOfB == CIRCLE)
                {
                    CircleCircleCollisionCheck((PhysicsCircle*)objectPointerA, (PhysicsCircle*)objectPointerB);
                }
                
                else if (shapeOfA == CIRCLE && shapeOfB == HALF_SPACE)
                {
                    CircleHalfspaceCollisionCheck((PhysicsCircle*)objectPointerA, (PhysicsHalfspace*)objectPointerB);
                }
                else if (shapeOfA == HALF_SPACE && shapeOfB == CIRCLE)
                {
                    CircleHalfspaceCollisionCheck((PhysicsCircle*)objectPointerB, (PhysicsHalfspace*)objectPointerA);
                }
                
                else if (shapeOfA == RECT && shapeOfB == RECT)
                {
                    RectangleAABBCollision((PhysicsRect*)objectPointerA, (PhysicsRect*)objectPointerB);
                }
                else if (shapeOfA == RECT && shapeOfB == HALF_SPACE)
                {
                    RectHalfspaceCollisionCheck((PhysicsRect*)objectPointerA, (PhysicsHalfspace*)objectPointerB);
                }
                else if (shapeOfA == HALF_SPACE && shapeOfB == RECT)
                {
                    RectHalfspaceCollisionCheck((PhysicsRect*)objectPointerB, (PhysicsHalfspace*)objectPointerA);
                }
                else if (shapeOfA == RECT && shapeOfB == CIRCLE)
                {
                    RectangleCircleCollision((PhysicsRect*)objectPointerA, (PhysicsCircle*)objectPointerB);
                }
                else if (shapeOfA == CIRCLE && shapeOfB == RECT)
                {
                    RectangleCircleCollision((PhysicsRect*)objectPointerB, (PhysicsCircle*)objectPointerA);
                }
            }
        }

        if (activeBird != nullptr)
        {
            for (int i = 0; i < objects.size(); i++)
            {
                PhysicsObj* objectPointer = objects[i];
                PhysicsShape objectShape = objectPointer->shape();


                if (activeBird->shape() == CIRCLE)
                {
                    PhysicsCircle* birdPointer = (PhysicsCircle*)activeBird;
                    
                    if (objectShape == CIRCLE)
                    {
                        CircleCircleCollisionCheck(birdPointer, (PhysicsCircle*)objectPointer);
                    }
                    else if (objectShape == HALF_SPACE)
                    {
                        CircleHalfspaceCollisionCheck(birdPointer, (PhysicsHalfspace*)objectPointer);
                    }
                    else if (objectShape == RECT)
                    {
                        RectangleCircleCollision((PhysicsRect*)objectPointer, birdPointer);
                    }
                }
                else if (activeBird->shape() == RECT)
                {
                    PhysicsRect* birdPointer = (PhysicsRect*)activeBird;
                    
                    if (objectShape == RECT)
                    {
                        RectangleAABBCollision(birdPointer, (PhysicsRect*)objectPointer);
                    }
                    else if (objectShape == HALF_SPACE)
                    {
                        RectHalfspaceCollisionCheck(birdPointer, (PhysicsHalfspace*)objectPointer);
                    }
                    else if (objectShape == CIRCLE)
                    {
                        RectangleCircleCollision(birdPointer, (PhysicsCircle*)objectPointer);
                    }
                }
            }
        }
    }
};

PhysicsWorld world;
PhysicsHalfspace halfspace;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////    Collision Checks    ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CircleCircleOverlap(PhysicsCircle* circleA, PhysicsCircle* circleB)
{
    Vector2 displaceAToB = circleB->position - circleA->position;
    float distance = Vector2Length(displaceAToB);
    float sumOfRadii = circleA->radius + circleB->radius;


    if (sumOfRadii > distance)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool CircleCircleCollisionCheck(PhysicsCircle* circleA, PhysicsCircle* circleB)
{
    Vector2 displaceAToB = circleB->position - circleA->position;
    float distance = Vector2Length(displaceAToB);
    float sumOfRadii = circleA->radius + circleB->radius;
    float overlap = sumOfRadii - distance;
    Vector2 normalAToB;

    if (abs(distance) == 0)
    {
        normalAToB = { 0, 1 };
    }
    else
    {
        normalAToB = displaceAToB / distance;
    }

    Vector2 mtv = normalAToB * overlap;


    if (sumOfRadii > distance)
    {
        circleA->position -= mtv * 0.5f;
        circleB->position += mtv * 0.5f;

        Vector2 velocityBRelativeToA = circleB->velocity - circleA->velocity;
        float closingVelocity1D = Vector2DotProduct(velocityBRelativeToA, normalAToB);

        if (closingVelocity1D >= 0) return true;

        float restitution = circleA->bounciness * circleB->bounciness;

        float totalMass = circleA->mass + circleB->mass;

        float impulseMagnitude = ((1.0 + restitution) * closingVelocity1D * circleA->mass * circleB->mass) / totalMass;

        Vector2 impulseB = normalAToB * -impulseMagnitude;
        Vector2 impulseA = normalAToB * impulseMagnitude;

        circleA->velocity += impulseA / circleA->mass;
        circleB->velocity += impulseB / circleB->mass;

        return true;
    }
    else
    {
        return false;
    }
}

// Halfspace Checks
bool CircleHalfspaceOverlap(PhysicsCircle* circle, PhysicsHalfspace* halfspace)
{
    Vector2 displacementToCircle = circle->position - halfspace->position;

    float dot = Vector2DotProduct(displacementToCircle, halfspace->GetNormal());
    Vector2 projectDisplacementOntoNorm = halfspace->GetNormal() * dot;

    DrawLineEx(circle->position, circle->position - projectDisplacementOntoNorm, 1, GRAY);
    Vector2 midpoint = circle->position - projectDisplacementOntoNorm * 0.5f;
    DrawText(TextFormat("D: %6.0f", dot), midpoint.x, midpoint.y, 30, GRAY);

    return dot < circle->radius ? true : false;
}

bool CircleHalfspaceCollisionCheck(PhysicsCircle* circle, PhysicsHalfspace* halfspace)
{
    Vector2 displacementToCircle = circle->position - halfspace->position;

    float dot = Vector2DotProduct(displacementToCircle, halfspace->GetNormal());
    Vector2 projectDisplacementOntoNorm = halfspace->GetNormal() * dot;
    float overlap = circle->radius - dot;

    DrawLineEx(circle->position, circle->position - projectDisplacementOntoNorm, 1, GRAY);
    Vector2 midpoint = circle->position - projectDisplacementOntoNorm * 0.5f;
    DrawText(TextFormat("D: %6.0f", dot), midpoint.x, midpoint.y, 30, GRAY);

    if (overlap > 0)
    {
        Vector2 mtv = halfspace->GetNormal() * overlap;
        circle->position += mtv;

        // Get Grav Forces
        Vector2 FGravity = world.accelGravity * circle->mass;

        // Apply Normal
        Vector2 FgPerp = halfspace->GetNormal() * Vector2DotProduct(FGravity, halfspace->GetNormal());
        Vector2 FNormal = FgPerp * -1;
        circle->netForce += FNormal;
        DrawLineEx(circle->position, circle->position + FNormal, 3, GREEN);

        // Friction
        float u = circle->grippiness * halfspace->grippiness;
        Vector2 FgPara = FGravity - FgPerp;
        float frictionMagnitude = u * Vector2Length(FNormal); // Max magnitude of force of friction

        if (frictionMagnitude > Vector2Length(FgPara))
        {
            frictionMagnitude = Vector2Length(FgPara);
        }

        Vector2 frictionDirection = Vector2Normalize(FgPara) * -1; // Direction of force of friction
        Vector2 Ffriction = frictionDirection * frictionMagnitude;

        circle->netForce += Ffriction;
        DrawLineEx(circle->position, circle->position + Ffriction, 3, ORANGE);

        // Bouncing

        float closingVelocity1D = Vector2DotProduct(circle->velocity, halfspace->GetNormal());

        if (closingVelocity1D >= 0) return true;

        float restitution = circle->bounciness * halfspace->bounciness;

        circle->velocity += halfspace->GetNormal() * closingVelocity1D * -(1.0f + restitution);

        return true;
    }
    else
    {
        return false;
    }
}

bool RectangleAABBCollision(PhysicsRect* rectA, PhysicsRect* rectB)
{
    Vector2 posA = rectA->position;
    Vector2 posB = rectB->position;
    Vector2 sizeA = rectA->size/2;
    Vector2 sizeB = rectB->size/2;

    Vector2 diff = rectB->position - rectA->position;
    Vector2 sizedist = (rectA->size + rectB->size)/2;
    
    if (abs(diff.x) < sizedist.x && abs(diff.y) < sizedist.y)
    {
        //* AABB Collision *//
        Vector2 overlap = { sizedist.x - abs(diff.x) , sizedist.y - abs(diff.y) };
        if (overlap.x < overlap.y)
        {
            overlap.y = 0;
            if (rectA->position.x > rectB->position.x)
            {
                overlap.x *= -1;
            }
            rectA->position.x -= overlap.x / 2;
            rectB->position.x += overlap.x / 2;
        }
        else if (overlap.y < overlap.x)
        {
            overlap.x = 0;
            if (rectA->position.y > rectB->position.y)
            {
                overlap.y *= -1;
            }
            rectA->position.y -= overlap.y / 2;
            rectB->position.y += overlap.y / 2;
        }



        //* Kinematic Forces *//
        Vector2 normalAToB = Vector2Normalize(overlap);
        
        Vector2 velocityBRelativeToA = rectB->velocity - rectA->velocity;
        float closingVelocity1D = Vector2DotProduct(velocityBRelativeToA, normalAToB);

        if (closingVelocity1D >= 0) return true;

        float restitution = rectA->bounciness * rectB->bounciness;
        float totalMass = rectA->mass + rectB->mass;
        float impulseMagnitude = ((1.0 + restitution) * closingVelocity1D * rectA->mass * rectB->mass) / totalMass;

        Vector2 impulseB = normalAToB * -impulseMagnitude;
        Vector2 impulseA = normalAToB * impulseMagnitude;

        rectA->velocity += impulseA / rectA->mass;
        rectB->velocity += impulseB / rectB->mass;

        return true;
    }
    else
    {
        return false;
    }
}
bool RectHalfspaceCollisionCheck(PhysicsRect* rect, PhysicsHalfspace* halfspace)
{
    Vector2 halfSize = rect->size / 2;
    Vector2 closestPoint;
    closestPoint.x = Clamp(halfspace->position.x, rect->position.x - halfSize.x, rect->position.x + halfSize.x);
    closestPoint.y = Clamp(halfspace->position.y, rect->position.y - halfSize.y, rect->position.y + halfSize.y);

    Vector2 displacementToRect = rect->position - halfspace->position;

    float dot = Vector2DotProduct(displacementToRect, halfspace->GetNormal());
    Vector2 projectDisplacementOntoNorm = halfspace->GetNormal() * dot;

    float overlap = halfSize.y - dot;


    if (overlap > 0)
    {
        Vector2 mtv = halfspace->GetNormal() * overlap;
        rect->position += mtv;

        // Get Grav Forces
        Vector2 FGravity = rect->velocity;

        // Apply Normal
        Vector2 FgPerp = halfspace->GetNormal() * Vector2DotProduct(FGravity, halfspace->GetNormal());
        Vector2 FNormal = FgPerp * -1;
        rect->netForce += FNormal;
        DrawLineEx(rect->position, rect->position + FNormal, 3, GREEN);

        // Friction
        float u = rect->grippiness * halfspace->grippiness;
        Vector2 FgPara = FGravity - FgPerp;
        float frictionMagnitude = u * Vector2Length(FNormal); // Max magnitude of force of friction

        if (frictionMagnitude > Vector2Length(FgPara))
        {
            frictionMagnitude = Vector2Length(FgPara);
        }

        Vector2 frictionDirection = Vector2Normalize(FgPara) * -1; // Direction of force of friction
        Vector2 Ffriction = frictionDirection * frictionMagnitude;

        rect->netForce += Ffriction;
        DrawLineEx(rect->position, rect->position + Ffriction, 3, ORANGE);

        // Bouncing

        float closingVelocity1D = Vector2DotProduct(rect->velocity, halfspace->GetNormal());

        if (closingVelocity1D >= 0) return true;

        float restitution = rect->bounciness * halfspace->bounciness;

        rect->velocity += halfspace->GetNormal() * closingVelocity1D * -(1.0f + restitution);

        return true;
    }
    else
    {
        return false;
    }
}
bool RectangleCircleCollision(PhysicsRect* rect, PhysicsCircle* circle)
{
    Vector2 halfSize = rect->size / 2;
    Vector2 closestPoint;
    closestPoint.x = Clamp(circle->position.x, rect->position.x - halfSize.x, rect->position.x + halfSize.x);
    closestPoint.y = Clamp(circle->position.y, rect->position.y - halfSize.y, rect->position.y + halfSize.y);

    Vector2 displacementToRect = closestPoint - circle->position;
    float dist = Vector2Length(displacementToRect);

    if (dist < circle->radius)
    {
        //* Overlap *//
        Vector2 normalCircleToRect = Vector2Normalize(displacementToRect);
        float overlap = (dist - circle->radius);

        circle->position += normalCircleToRect * overlap / 2;
        rect->position -= normalCircleToRect * overlap / 2;


        //* Kinematics *//
        Vector2 velRectRelativeToCircle = rect->velocity - circle->velocity;
        float closingVelocity1D = Vector2DotProduct(velRectRelativeToCircle, normalCircleToRect);

        if (closingVelocity1D >= 0) return true;

        float restitution = circle->bounciness * rect->bounciness;
        float totalMass = circle->mass + rect->mass;
        float impulseMagnitude = ((1.0 + restitution) * closingVelocity1D * circle->mass * rect->mass) / totalMass;

        Vector2 impulseRect = normalCircleToRect * -impulseMagnitude;
        Vector2 impulseCircle = normalCircleToRect * impulseMagnitude;

        circle->velocity += impulseCircle / circle->mass;
        rect->velocity += impulseRect / rect->mass;

        return true;
    }
    else
    {
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////        Cleanup         ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cleanup()
{
    for (int i = 0; i < world.objects.size(); i++)
    {
        PhysicsObj* object = world.objects[i];

        if (object->position.y > GetScreenHeight() || object->position.y < 0
            || object->position.x > GetScreenWidth() || object->position.x < 0)
        {
            auto iterator = (world.objects.begin() + i);
            PhysicsObj* pointerToPhysicsObj = *iterator;
            delete pointerToPhysicsObj;

            world.objects.erase(iterator);
            i--;
        }
    }

    if (world.activeBird != nullptr)
    {
        if (world.activeBird->position.y > GetScreenHeight() || world.activeBird->position.y < 0
            || world.activeBird->position.x > GetScreenWidth() || world.activeBird->position.x < 0)
        {
            delete world.activeBird;
            world.activeBird = nullptr;
            isBirdSpawned = false;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////         Update         ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool aimingBird = false;

void update()
{
    Vector2 mouse = GetMousePosition();
    dt = 1.0f / TARGET_FPS;
    time += dt;

    cleanup();
    world.update();

    if (world.activeBird != nullptr)
    {
        if (IsKeyPressed(KEY_R))
        {
            delete world.activeBird;
            world.activeBird = nullptr;
            isBirdSpawned = false;
        }
    }

    if (IsKeyPressed(KEY_SPACE))
    {
        if (birdIndex == 0)
        {
            birdIndex = 1;
        }
        else if (birdIndex == 1)
        {
            birdIndex = 0;
        }
    }

    if (!isBirdSpawned)
    {
        if (!aimingBird)
        {
            DrawCircleV(world.startPos, spawnRadius, WHITE);

            if (CheckCollisionPointCircle(mouse, world.startPos, spawnRadius))
            {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
                {
                    if (birdIndex == 0)
                    {
                        PhysicsCircle* bird = new PhysicsCircle();

                        bird->position = world.startPos;
                        bird->bounciness = restitution;
                        bird->isStatic = true;
                        world.activeBird = bird;
                    }
                    else if (birdIndex == 1)
                    {
                        PhysicsRect* bird = new PhysicsRect();
                        bird->size = { 20, 20 };

                        bird->position = world.startPos;
                        bird->bounciness = restitution;
                        bird->isStatic = true;
                        world.activeBird = bird;
                    }

                    aimingBird = true;
                }
            }
        }
        if (aimingBird && world.activeBird != nullptr)
        {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
            {
                Vector2 displacement = mouse - world.startPos;
                float dist = Vector2Length(displacement);
                if (dist > slingshotRadius)
                {
                    displacement = Vector2Normalize(displacement) * slingshotRadius;
                }
                world.activeBird->position = world.startPos + displacement;

                DrawLineEx(world.startPos, world.activeBird->position, 3, RED);
            }

            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
            {
                Vector2 displacement = world.startPos - world.activeBird->position;
                float birdMagnitude = Vector2Length(displacement) * 0.75;
                Vector2 birdDirection = Vector2Normalize(displacement);

                world.activeBird->isStatic = false;

                float launchScale = 3.0f;
                world.activeBird->velocity = birdDirection * birdMagnitude * launchScale;
                isBirdSpawned = true;
                aimingBird = false;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////          Draw          ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw()
{
    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Assignment 3", 10, float(GetScreenHeight() - 30), 20, LIGHTGRAY);

    DrawRectanglePro({ 95, 520, 50, 10 }, { 25, 5 }, 60.0f, BROWN);
    DrawRectanglePro({ 115, 520, 50, 10 }, { 25, 5 }, -60.0f, BROWN);
    DrawRectangle(100, 540, 10, 60, BROWN);

    for (int i = 0; i < world.objects.size(); i++)
    {
        world.objects[i]->draw();
    }

    if (world.activeBird != nullptr)
    {
        world.activeBird->draw();
    }

    EndDrawing();
}





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////          Main          ////////////////////////////////////////////////////////
///////////////////////////////////////////////                        ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpawnInitialObjects()
{
    float mass = 16, bounce = 0.2f, grip = 2;

    //Left beam
    PhysicsRect* rect = new PhysicsRect();
    rect->position = { 700, halfspace.position.y - 40 };
    rect->size = { 20, 80 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);
    //Middle beam
    rect = new PhysicsRect();
    rect->position = { 780, halfspace.position.y - 40 };
    rect->size = { 20, 80 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);
    //Right beam
    rect = new PhysicsRect();
    rect->position = { 860, halfspace.position.y - 40 };
    rect->size = { 20, 80 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);

    // Left Roof
    rect = new PhysicsRect();
    rect->position = { 740, halfspace.position.y - 90 };
    rect->size = { 80, 20 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);
    // Right Roof
    rect = new PhysicsRect();
    rect->position = { 820, halfspace.position.y - 90 };
    rect->size = { 80, 20 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);


    // Top Left Beam
    rect = new PhysicsRect();
    rect->position = { 740, halfspace.position.y - 140 };
    rect->size = { 20, 80 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);
    // Top Right Beam
    rect = new PhysicsRect();
    rect->position = { 820, halfspace.position.y - 140 };
    rect->size = { 20, 80 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);

    //Top Roof
    rect = new PhysicsRect();
    rect->position = { 780, halfspace.position.y - 200 };
    rect->size = { 80, 20 };
    rect->mass = mass; rect->bounciness = bounce; rect->grippiness = grip;
    world.add(rect);
}

int main()
{
    InitWindow(InitialWidth, InitialHeight, "GAME2005 Logan Medina 101538952");
    SetTargetFPS(TARGET_FPS);

    halfspace.isStatic = true;
    halfspace.position = { 300, 600 };
    halfspace.grippiness = 1;
    world.add(&halfspace);

    SpawnInitialObjects();
   
    while (!WindowShouldClose())
    {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}
