/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"

const unsigned int TARGET_FPS = 50;
float dt = 1.0f / TARGET_FPS;
float time = 0;
float x = 500;
float y = 500;
float frequency = 1;
float amplitude = 100;

float speed = 100;
float angle = 0;
float startPosX = 100;
float startPosY = 100;

void update()
{
    dt = 1.0f / TARGET_FPS;
    time += dt;

    x = x + (-sin(time * frequency)) * frequency * amplitude * dt;
    y = y + (cos(time * frequency)) * frequency * amplitude * dt;
}

void draw()
{
    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Logan Christopher Medina 101538952", 10, float(GetScreenHeight() - 30), 20, LIGHTGRAY);

    GuiSliderBar(Rectangle{ 35, 15, 1000, 20 }, "Time", TextFormat("%.2f", time), &time, 0, 240);

    GuiSliderBar(Rectangle{ 35, 45, 450, 20 }, "Speed", TextFormat("Speed: %.0f", speed), &speed, -300, 300);
    GuiSliderBar(Rectangle{ 35, 75, 450, 20 }, "Angle", TextFormat("Angle: %.0f Degrees", angle), &angle, -180, 180);

    GuiSliderBar(Rectangle{ 650, 45, 450, 20 }, "Start", TextFormat("X: %.0f", startPosX), &startPosX, 200, 1000);
    GuiSliderBar(Rectangle{ 650, 75, 450, 20 }, "Start", TextFormat("Y: %.0f", startPosY), &startPosY, 200, 600);

    DrawText(TextFormat("T: %.2f", time), GetScreenWidth() - 130, 10, 30, LIGHTGRAY);

    Vector2 startPos = { startPosX, startPosY };
    Vector2 velocity = { speed * cos(angle * DEG2RAD), speed * sin(angle * DEG2RAD) };

    DrawLineEx(startPos, startPos + velocity, 3, RED);

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
