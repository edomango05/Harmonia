#include <stdbool.h>

#include "graphics.h"
#include "raylib.h"
#include "scene.h"
#include "types.h"

const Color COLOR_BG = {24, 24, 24, 255};
const Color COLOR_GRID = {50, 50, 50, 255};
const Color COLOR_NODE = {240, 240, 240, 255};
const Color COLOR_BEAM = {100, 180, 255, 255};
const Color COLOR_EXT_CONSTRAINT = {255, 160, 60, 255};
const Color COLOR_INT_CONSTRAINT = {200, 100, 255, 255};
const Color COLOR_FORCE = {255, 50, 80, 255};
const Color COLOR_REACTION = {50, 255, 100, 255};
const Color COLOR_DIAGRAM_N = {255, 200, 50, 255};
const Color COLOR_DIAGRAM_T = {50, 200, 255, 255};
const Color COLOR_DIAGRAM_M = {255, 100, 200, 255};

int main(void) {
	const int screenWidth = 1280;
	const int screenHeight = 720;

	SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
	InitWindow(screenWidth, screenHeight, "Beam Solver 2D - FEM Engine");
	SetTargetFPS(60);

	Camera2D camera = {0};
	camera.zoom = 1.0f;
	camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};

	AppScene scene = InitScene();
	InputState input = {0};

	while (!WindowShouldClose()) {
		UpdateInputState(&input, &scene, camera);
		ProcessAppInput(&scene, &input, &camera);

		BeginDrawing();
		ClearBackground(COLOR_BG);

		BeginMode2D(camera);
		RenderScene(&scene, &input);
		EndMode2D();

		DrawTopUI(scene.currentMode, scene.diagramScale, GetScreenWidth());
		EndDrawing();
	}

	FreeScene(&scene);
	CloseWindow();
	return 0;
}