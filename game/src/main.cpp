/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/


/*
Notes for Lab Exercise 8

AABB to AABB collision
1. to get the min and max of the AABB calculate with
objAMax = objA.pos.x + objA.size.x/2 and objAMin = objA.pos.x - objA.size.x/2

2. if the min of A's AABB is between the min and max of B's AABB or the max of A's AABB is betweem the min and max
of B's AABB they overlap on this axis, otherwise they don't overlap so we can exit collision detection early.
Overlap = (A.size/2 + B.size/2) - distance.
        a. Note that if you do A.max - B.min, if its negative they don't overlap, and if its greater than both
        objects widths added together they don't overlap. Any thing between is the overlap

3. If all axes did have an overlap, pick the shortest one. Use that as a your Collision Normal, and use that overlap
to multiply with with Normal which creates MTV e.g. if (abs(overlapX) < abs(overlapY)) 
then MTV = {sign(displacement.x), 0};

4. Move them apart by MTV. We can divide MTV in half and give each half of MTV to move by, as we did with
circle-circle, or since we now have mass, make it inversely proportional to mass.

sign(X) is equal to +1 or -1 based on if X is positive or negative.

----------------------------------------------------------------------------------------------------------------------

AABB to Circle collision
1.  if the circle center is INSIDE the AABB, they overlap
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

float mass1 = 1.0f;
float mass2 = 1.0f;
float mass3 = 1.0f;
float mass4 = 1.0f;
float mass5 = 1.0f;

float xCircle1 = 0.0f;
float xCircle2 = 0.0f;
float xCircle3 = 0.0f;
float xCircle4 = 100.0f;
float xCircle5 = 0.0f;

float yCircle1 = 0.0f;
float yCircle2 = 0.0f;
float yCircle3 = 0.0f;
float yCircle4 = 0.0f;
float yCircle5 = 0.0f;

enum PhysicsShape
{
    CIRCLE,
    RECT,
    HALF_SPACE
};

//float x = 500;
//float y = 500;
//float frequency = 1;
//float amplitude = 100;

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
        //DrawLineEx(position, position + velocity, 3, RED);

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

// Physics World

bool CircleCircleOverlap(PhysicsCircle* circleA, PhysicsCircle* circleB);
bool CircleCircleCollisionCheck(PhysicsCircle* circleA, PhysicsCircle* circleB);
bool CircleHalfspaceOverlap(PhysicsCircle* circle, PhysicsHalfspace* halfspace);
bool CircleHalfspaceCollisionCheck(PhysicsCircle* circle, PhysicsHalfspace* halfspace);

class PhysicsWorld
{
public:
    std::vector<PhysicsObj*> objects;
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
    }

    void update()
    {
        /*for (int i = 0; i < objects.size(); i++)
        {
            objects[i]->color = GREEN;
        }*/

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
                    if (CircleCircleCollisionCheck((PhysicsCircle*)objectPointerA, (PhysicsCircle*)objectPointerB))
                    {
                        /*objectPointerA->color = RED;
                        objectPointerB->color = RED;*/
                    }
                }
                else if (shapeOfA == CIRCLE && shapeOfB == HALF_SPACE)
                {
                    
                    if (CircleHalfspaceCollisionCheck((PhysicsCircle*)objectPointerA, (PhysicsHalfspace*)objectPointerB))
                    {
                        /*objectPointerA->color = RED;
                        objectPointerB->color = RED;*/
                    }
                }
                else if (shapeOfA == HALF_SPACE && shapeOfB == CIRCLE)
                {
                    if (CircleHalfspaceCollisionCheck((PhysicsCircle*)objectPointerB, (PhysicsHalfspace*)objectPointerA))
                    {
                        /*objectPointerA->color = RED;
                        objectPointerB->color = RED;*/
                    }
                }
            }
        }
    }
};

PhysicsWorld world;
PhysicsHalfspace halfspace;

// Circle Checks
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

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

// Cleanup function
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
}

// Update function
void update()
{
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

    /*if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {

    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        PhysicsCircle* bird = new PhysicsCircle();
        bird->position = world.startPos;
        bird->velocity = { speed * (float)cos(angle * DEG2RAD), speed * (float)sin(angle * DEG2RAD) };
        bird->radius = (rand() % 16) + 10;
        bird->bounciness = restitution;

        world.add(bird);
    }*/
}

// Draw function
void draw()
{
    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Logan Christopher Medina 101538952", 10, float(GetScreenHeight() - 30), 20, LIGHTGRAY);

    DrawRectanglePro({ 95, 520, 50, 10 }, { 25, 5 }, 60.0f, BROWN);
    DrawRectanglePro({ 115, 520, 50, 10 }, { 25, 5 }, -60.0f, BROWN);
    DrawRectangle(100, 540, 10, 60, BROWN);

    GuiSliderBar(Rectangle{ 55, 15, 450, 20 }, "Angle", TextFormat("Angle: %.0f Degrees", angle * -1), &angle, -180, 180);

    GuiSliderBar(Rectangle{ 680, 15, 450, 20 }, "Acceleration", TextFormat("Gravity: %.0f", world.accelGravity.y), &world.accelGravity.y, -600, 600);

    GuiSliderBar(Rectangle{ 75, 75, 1000, 20 }, "Restitution", TextFormat("%.0f", restitution), &restitution, 0, 1);

    Vector2 startPos = { world.startPos.x, world.startPos.y };
    Vector2 velocity = { speed * cos(angle * DEG2RAD), speed * sin(angle * DEG2RAD) };
    DrawLineEx(world.startPos, world.startPos + velocity, 3, RED);

    for (int i = 0; i < world.objects.size(); i++)
    {
        world.objects[i]->draw();
    }

    EndDrawing();
}

// Main
int main()
{
    InitWindow(InitialWidth, InitialHeight, "GAME2005 Logan Medina 101538952");
    SetTargetFPS(TARGET_FPS);

    halfspace.isStatic = true;
    halfspace.position = { 300, 600 };
    halfspace.grippiness = 1;
    world.add(&halfspace);

    /*PhysicsCircle* circle1 = new PhysicsCircle();
    PhysicsCircle* circle2 = new PhysicsCircle();
    PhysicsCircle* circle3 = new PhysicsCircle();
    PhysicsCircle* circle4 = new PhysicsCircle();
    PhysicsCircle* circle5 = new PhysicsCircle();

    circle1->position = { 25, 300 };
    circle2->position = { 25, 311 };
    circle3->position = { 100, 300 };
    circle4->position = { 200, 585 };
    circle5->position = { 300, 585 };

    circle1->velocity = { xCircle1, yCircle1 };
    circle2->velocity = { xCircle2, yCircle2 };
    circle3->velocity = { xCircle3, yCircle3 };
    circle4->velocity = { xCircle4, yCircle4 };
    circle5->velocity = { xCircle5, yCircle5 };

    circle1->mass = mass1;
    circle2->mass = mass2;
    circle3->mass = mass3;
    circle4->mass = mass4;
    circle5->mass = mass5;

    circle1->color = RED;
    circle2->color = GREEN;
    circle3->color = BLUE;
    circle4->color = YELLOW;
    circle5->color = PURPLE;

    circle1->bounciness = restitution;
    circle2->bounciness = restitution;
    circle3->bounciness = restitution;
    circle4->bounciness = restitution;
    circle5->bounciness = restitution;

    world.add(circle1);
    world.add(circle2);
    world.add(circle3);
    world.add(circle4);
    world.add(circle5);*/
   
    while (!WindowShouldClose())
    {
        update();
        draw();

        /*GuiSliderBar(Rectangle{ 75, 105, 100, 20 }, "Circle 1", TextFormat("Mass: %.0f", mass1), &mass1, 1, 10);
        GuiSliderBar(Rectangle{ 75, 135, 100, 20 }, "Circle 2", TextFormat("Mass: %.0f", mass2), &mass2, 1, 10);
        GuiSliderBar(Rectangle{ 75, 165, 100, 20 }, "Circle 3", TextFormat("Mass: %.0f", mass3), &mass3, 1, 10);
        GuiSliderBar(Rectangle{ 75, 195, 100, 20 }, "Circle 4", TextFormat("Mass: %.0f", mass4), &mass4, 1, 10);
        GuiSliderBar(Rectangle{ 75, 225, 100, 20 }, "Circle 5", TextFormat("Mass: %.0f", mass5), &mass5, 1, 10);

        GuiSliderBar(Rectangle{ 300, 105, 100, 20 }, "Circle 1", TextFormat("Initial X: %.0f", xCircle1), &xCircle1, 0, 100);
        GuiSliderBar(Rectangle{ 300, 135, 100, 20 }, "Circle 2", TextFormat("Initial X: %.0f", xCircle2), &xCircle2, 0, 100);
        GuiSliderBar(Rectangle{ 300, 165, 100, 20 }, "Circle 3", TextFormat("Initial X: %.0f", xCircle3), &xCircle3, 0, 100);
        GuiSliderBar(Rectangle{ 300, 195, 100, 20 }, "Circle 4", TextFormat("Initial X: %.0f", xCircle4), &xCircle4, 0, 100);
        GuiSliderBar(Rectangle{ 300, 225, 100, 20 }, "Circle 5", TextFormat("Initial X: %.0f", xCircle5), &xCircle5, 0, 100);

        GuiSliderBar(Rectangle{ 525, 105, 100, 20 }, "Circle 1", TextFormat("Initial Y: %.0f", yCircle1), &yCircle1, 0, 100);
        GuiSliderBar(Rectangle{ 525, 135, 100, 20 }, "Circle 2", TextFormat("Initial Y: %.0f", yCircle2), &yCircle2, 0, 100);
        GuiSliderBar(Rectangle{ 525, 165, 100, 20 }, "Circle 3", TextFormat("Initial Y: %.0f", yCircle3), &yCircle3, 0, 100);
        GuiSliderBar(Rectangle{ 525, 195, 100, 20 }, "Circle 4", TextFormat("Initial Y: %.0f", yCircle4), &yCircle4, 0, 100);
        GuiSliderBar(Rectangle{ 525, 225, 100, 20 }, "Circle 5", TextFormat("Initial Y: %.0f", yCircle5), &yCircle5, 0, 100);*/
    }

    CloseWindow();
    return 0;
}
