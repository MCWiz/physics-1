/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"
#include <vector>

const unsigned int TARGET_FPS = 50;
float dt = 1.0f / TARGET_FPS;
float time = 0;
float restitution = 0.9f;
bool isBirdSpawned = false;
float spawnRadius = 10.0f;
float slingshotRadius = 100.0f;

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
    float sizeX = 10;
    float sizeY = 10;

    void draw() override
    {
        DrawRectangle(position.x, position.y, sizeX, sizeY, RED);
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

class PhysicsWorld
{
public:
    std::vector<PhysicsObj*> objects;
    PhysicsCircle* activeBird;
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
            }
        }

        if (activeBird != nullptr)
        {
            for (int i = 0; i < objects.size(); i++)
            {
                PhysicsObj* objectPointer = objects[i];
                PhysicsShape objectShape = objectPointer->shape();
                PhysicsCircle* birdPointer = activeBird;

                if (objectShape == CIRCLE)
                {
                    CircleCircleCollisionCheck(birdPointer, (PhysicsCircle*)objectPointer);
                }
                else if (objectShape == HALF_SPACE)
                {
                    CircleHalfspaceCollisionCheck(birdPointer, (PhysicsHalfspace*)objectPointer);
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
void update()
{
    Vector2 mouse = GetMousePosition();
    dt = 1.0f / TARGET_FPS;
    time += dt;

    cleanup();
    world.update();

    if (IsKeyPressed(KEY_SPACE))
    {
        PhysicsCircle* bird = new PhysicsCircle();
        bird->position = world.startPos;
        bird->velocity = { speed * (float)cos(angle * DEG2RAD), speed * (float)sin(angle * DEG2RAD) };
        bird->radius = (rand() % 16) + 10;
        bird->bounciness = restitution;

        world.add(bird);
    }

    if (!isBirdSpawned)
    {
        if (CheckCollisionPointCircle(mouse, world.startPos, spawnRadius))
        {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            {
                PhysicsCircle* bird = new PhysicsCircle();
                bird->position = world.startPos;
                bird->bounciness = restitution;
                bird->isStatic = true;
                world.activeBird = bird;
            }
        }
        if (CheckCollisionPointCircle(mouse, world.startPos, slingshotRadius))
        {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
            {
                if (world.activeBird != nullptr)
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
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            if (world.activeBird != nullptr)
            {
                Vector2 displacement = world.startPos - world.activeBird->position;
                float birdMagnitude = Vector2Length(displacement);
                Vector2 birdDirection = Vector2Normalize(displacement);

                world.activeBird->isStatic = false;

                float launchScale = 3.0f;
                world.activeBird->velocity = birdDirection * birdMagnitude * launchScale;
                isBirdSpawned = true;
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
    DrawText("Logan Christopher Medina 101538952", 10, float(GetScreenHeight() - 30), 20, LIGHTGRAY);

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
int main()
{
    InitWindow(InitialWidth, InitialHeight, "GAME2005 Logan Medina 101538952");
    SetTargetFPS(TARGET_FPS);

    halfspace.isStatic = true;
    halfspace.position = { 300, 600 };
    halfspace.grippiness = 1;
    world.add(&halfspace);
   
    while (!WindowShouldClose())
    {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}
