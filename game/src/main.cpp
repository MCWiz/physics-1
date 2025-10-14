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

enum PhysicsShape
{
    CIRCLE,
    HALF_SPACE
};

//float x = 500;
//float y = 500;
//float frequency = 1;
//float amplitude = 100;

float speed = 100;
float angle = 0;

class PhysicsObj
{
public:
    Vector2 position = { 500, 500 };
    Vector2 velocity = { 0, 0 };
    float mass = 1;
    Color color = GREEN;
    bool isColliding = false;

    virtual void draw()
    {
        DrawCircle(position.x, position.y, 2, color);
    }
};

class PhysicsCircle : public PhysicsObj
{
public:
    float radius;

    void draw() override
    {
        DrawLineEx(position, position + velocity, 3, RED);

        DrawCircle(position.x, position.y, radius, color);
    }
};

//class PhysicsBox : public PhysicsObj
//{
//    Vector2 size = { 5, 5 };
//};

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

class PhysicsWorld
{
public:
    std::vector<PhysicsObj*> objects;
    Vector2 accelGravity = { 0, 9 };
    Vector2 startPos = { 500, 500 };

    void add(PhysicsObj* newObj)
    {
        objects.push_back(newObj);
    }

    void update()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            PhysicsObj* object = objects[i];

            object->position = object->position + object->velocity * dt;
            object->velocity = object->velocity + accelGravity * dt;
        }

        collisionCheck();
    }

    void collisionCheck()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            for (int j = i + 1; j < objects.size(); j++)
            {
                PhysicsObj* objectPointerA = objects[i];
                PhysicsCircle* circlePointerA = (PhysicsCircle*)objectPointerA;

                PhysicsObj* objectPointerB = objects[j];
                PhysicsCircle* circlePointerB = (PhysicsCircle*)objectPointerB;

                if (CircleCircleOverlap(circlePointerA, circlePointerB))
                {
                    objectPointerA->isColliding = true;
                    objectPointerB->isColliding = true;
                }
                else
                {
                    if (objectPointerA->isColliding == true && objectPointerB->isColliding == false)
                    {
                        break;
                    }
                    else if (objectPointerA->isColliding == true && objectPointerB->isColliding == true)
                    {
                        objectPointerA->isColliding = false;
                        objectPointerB->isColliding = false;
                    }
                    else
                    {
                        objectPointerA->isColliding = false;
                        objectPointerB->isColliding = false;
                    }
                }
            }
        }

        for (int i = 0; i < objects.size(); i++)
        {
            if (objects[i]->isColliding == true)
            {
                objects[i]->color = RED;
            }
            else
            {
                objects[i]->color = GREEN;
            }
        }
    }
};

PhysicsWorld world;

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
        // Color randColor = { rand() % 256, rand() % 256, rand() % 256, 255 };

        world.add(bird);
    }

    /*x = x + (-sin(time * frequency)) * frequency * amplitude * dt;
    y = y + (cos(time * frequency)) * frequency * amplitude * dt;*/
}

void draw()
{
    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Logan Christopher Medina 101538952", 10, float(GetScreenHeight() - 30), 20, LIGHTGRAY);

    GuiSliderBar(Rectangle{ 35, 15, 1000, 20 }, "Time", TextFormat("%.2f", time), &time, 0, 240);

    GuiSliderBar(Rectangle{ 35, 45, 450, 20 }, "Speed", TextFormat("Speed: %.0f", speed), &speed, -300, 300);
    GuiSliderBar(Rectangle{ 35, 75, 450, 20 }, "Angle", TextFormat("Angle: %.0f Degrees", angle * -1), &angle, -180, 180);

    GuiSliderBar(Rectangle{ 75, 105, 1000, 20 }, "Acceleration", TextFormat("Gravity: %.0f", world.accelGravity.y), &world.accelGravity.y, -600, 600);

    GuiSliderBar(Rectangle{ 650, 45, 450, 20 }, "Start", TextFormat("X: %.0f", world.startPos.x), &world.startPos.x, 200, 1000);
    GuiSliderBar(Rectangle{ 650, 75, 450, 20 }, "Start", TextFormat("Y: %.0f", world.startPos.y), &world.startPos.y, 200, 600);

    DrawText(TextFormat("T: %.2f", time), GetScreenWidth() - 130, 10, 30, LIGHTGRAY);

    /*Vector2 startPos = { startPosX, startPosY };*/
    Vector2 velocity = { speed * cos(angle * DEG2RAD), speed * sin(angle * DEG2RAD) };

    DrawLineEx(world.startPos, world.startPos + velocity, 3, RED);

    for (int i = 0; i < world.objects.size(); i++)
    {
        world.objects[i]->draw();
    }

    /*DrawCircle(x, y, 70, RED);
    DrawCircle(500 + cos(time * frequency) * amplitude, 500 + sin(time * frequency) * amplitude, 70, GREEN);*/

    EndDrawing();
}

int main()
{
    InitWindow(InitialWidth, InitialHeight, "GAME2005 Logan Medina 101538952");
    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose())
    {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}
