#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "types.h"
#include "math_utils.h"
#include "scene.h"
#include "graphics.h"
#include "fem_solver.h"

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Beam Solver 2D - FEM Engine");

    Color bgColor = (Color){24, 24, 24, 255};
    Color gridColor = (Color){50, 50, 50, 255};
    Color nodeColor = (Color){240, 240, 240, 255};
    Color beamColor = (Color){100, 180, 255, 255};
    Color extConstraintColor = (Color){255, 160, 60, 255};
    Color intConstraintColor = (Color){200, 100, 255, 255};
    Color forceColor = (Color){255, 50, 80, 255};
    Color reactionColor = (Color){50, 255, 100, 255};
    Color diagramN = (Color){255, 200, 50, 255};
    Color diagramT = (Color){50, 200, 255, 255};
    Color diagramM = (Color){255, 100, 200, 255};

    AppMode currentMode = MODE_EDIT;
    float diagramScale = 0.5f;

    Camera2D camera = {0};
    camera.zoom = 1.0f;
    camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};

    int nodeCapacity = 10, nodeCount = 0;
    Node *nodes = (Node *)calloc(nodeCapacity, sizeof(Node));
    int beamCapacity = 10, beamCount = 0;
    Beam *beams = (Beam *)calloc(beamCapacity, sizeof(Beam));

    bool isDraggingBeam = false;
    int dragStartNodeIdx = -1;
    bool isDraggingForce = false;
    int forceNodeIdx = -1;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        Vector2 snappedPos = {roundf(mouseWorldPos.x / GRID_SIZE) * GRID_SIZE,
                              roundf(mouseWorldPos.y / GRID_SIZE) * GRID_SIZE};
        
        int hoveredNodeIdx = GetNodeUnderMouse(nodes, nodeCount, mouseWorldPos);
        bool shiftPressed = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        bool fPressed = IsKeyDown(KEY_F);
        float wheel = GetMouseWheelMove();
        int hoveredBeam = GetHoveredBeam(mouseWorldPos, nodes, beams, beamCount);

        if (IsKeyPressed(KEY_TAB)) {
            currentMode = (currentMode + 1) % MODE_COUNT;
            if (currentMode == MODE_REACTIONS)
                SolveFEM(nodes, nodeCount, beams, beamCount);
        }

        if (currentMode != MODE_EDIT) {
            if (IsKeyPressed(KEY_UP)) diagramScale *= 1.5f;
            if (IsKeyPressed(KEY_DOWN)) diagramScale /= 1.5f;
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            camera.target = Vector2Add(camera.target, Vector2Scale(GetMouseDelta(), -1.0f / camera.zoom));
        }

        if (currentMode == MODE_EDIT) {
            if (fPressed) {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hoveredNodeIdx != -1) {
                    isDraggingForce = true;
                    forceNodeIdx = hoveredNodeIdx;
                }
                
                if (isDraggingForce) {
                    Vector2 rawForce = Vector2Subtract(mouseWorldPos, nodes[forceNodeIdx].position);
                    float mag = roundf(Vector2Length(rawForce) / 10.0f) * 10.0f;
                    float ang = atan2f(rawForce.y, rawForce.x);
                    if (shiftPressed) {
                        float snapAngle = 15.0f * DEG2RAD;
                        ang = roundf(ang / snapAngle) * snapAngle;
                    }
                    nodes[forceNodeIdx].force.x = mag * cosf(ang);
                    nodes[forceNodeIdx].force.y = mag * sinf(ang);
                } else if (wheel != 0 && hoveredNodeIdx != -1 && Vector2Length(nodes[hoveredNodeIdx].force) > 0) {
                    float mag = Vector2Length(nodes[hoveredNodeIdx].force);
                    float ang = atan2f(nodes[hoveredNodeIdx].force.y, nodes[hoveredNodeIdx].force.x);
                    mag += wheel * 1.0f;
                    if (mag < 0) mag = 0;
                    nodes[hoveredNodeIdx].force.x = mag * cosf(ang);
                    nodes[hoveredNodeIdx].force.y = mag * sinf(ang);
                    wheel = 0;
                }
                
                if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && isDraggingForce) {
                    if (Vector2Length(nodes[forceNodeIdx].force) < 10.0f)
                        nodes[forceNodeIdx].force = (Vector2){0, 0};
                    isDraggingForce = false;
                    forceNodeIdx = -1;
                }

                if (hoveredBeam != -1) {
                    float load_step = 0.1f;
                    if (IsKeyPressed(KEY_UP)) beams[hoveredBeam].q_perp += load_step;
                    if (IsKeyPressed(KEY_DOWN)) beams[hoveredBeam].q_perp -= load_step;
                    if (IsKeyPressed(KEY_RIGHT)) beams[hoveredBeam].q_axial += load_step;
                    if (IsKeyPressed(KEY_LEFT)) beams[hoveredBeam].q_axial -= load_step;
                }
            } else {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hoveredNodeIdx != -1) {
                    if (shiftPressed) {
                        nodes[hoveredNodeIdx].int_constraint = (nodes[hoveredNodeIdx].int_constraint + 1) % INT_CONSTRAINT_COUNT;
                        if (nodes[hoveredNodeIdx].int_constraint == INT_CONSTRAINT_RIGID)
                            nodes[hoveredNodeIdx].int_angle = 0.0f;
                    } else {
                        nodes[hoveredNodeIdx].constraint = (nodes[hoveredNodeIdx].constraint + 1) % CONSTRAINT_COUNT;
                        if (nodes[hoveredNodeIdx].constraint == CONSTRAINT_FREE)
                            nodes[hoveredNodeIdx].angle = 0.0f;
                    }
                }
                if (wheel != 0 && hoveredNodeIdx != -1) {
                    if (shiftPressed) nodes[hoveredNodeIdx].int_angle += wheel * 15.0f;
                    else nodes[hoveredNodeIdx].angle += wheel * 15.0f;
                    wheel = 0;
                }
            }

            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                int idx = GetNodeAtGridPosition(nodes, nodeCount, snappedPos);
                if (idx == -1) {
                    if (nodeCount >= nodeCapacity) {
                        nodeCapacity *= 2;
                        nodes = (Node *)realloc(nodes, nodeCapacity * sizeof(Node));
                    }
                    nodes[nodeCount].id = nodeCount;
                    nodes[nodeCount].position = snappedPos;
                    nodes[nodeCount].constraint = CONSTRAINT_FREE;
                    nodes[nodeCount].angle = 0.0f;
                    nodes[nodeCount].int_constraint = INT_CONSTRAINT_RIGID;
                    nodes[nodeCount].int_angle = 0.0f;
                    nodes[nodeCount].force = (Vector2){0, 0};
                    idx = nodeCount++;
                }
                isDraggingBeam = true;
                dragStartNodeIdx = idx;
            }
            
            if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT) && isDraggingBeam) {
                int endIdx = GetNodeAtGridPosition(nodes, nodeCount, snappedPos);
                if (endIdx == -1) {
                    if (nodeCount >= nodeCapacity) {
                        nodeCapacity *= 2;
                        nodes = (Node *)realloc(nodes, nodeCapacity * sizeof(Node));
                    }
                    nodes[nodeCount].id = nodeCount;
                    nodes[nodeCount].position = snappedPos;
                    nodes[nodeCount].constraint = CONSTRAINT_FREE;
                    nodes[nodeCount].angle = 0.0f;
                    nodes[nodeCount].int_constraint = INT_CONSTRAINT_RIGID;
                    nodes[nodeCount].int_angle = 0.0f;
                    nodes[nodeCount].force = (Vector2){0, 0};
                    endIdx = nodeCount++;
                }
                if (dragStartNodeIdx != endIdx && !BeamExists(beams, beamCount, dragStartNodeIdx, endIdx)) {
                    if (beamCount >= beamCapacity) {
                        beamCapacity *= 2;
                        beams = (Beam *)realloc(beams, beamCapacity * sizeof(Beam));
                    }
                    beams[beamCount].node_start_idx = dragStartNodeIdx;
                    beams[beamCount].node_end_idx = endIdx;
                    
                    beams[beamCount].q_perp = 0.0;
                    beams[beamCount].q_axial = 0.0;
                    beams[beamCount].N_start = 0.0f;
                    beams[beamCount].N_end = 0.0f;
                    beams[beamCount].T_start = 0.0f;
                    beams[beamCount].T_end = 0.0f;
                    beams[beamCount].M_start = 0.0f;
                    beams[beamCount].M_end = 0.0f;
                    beams[beamCount].release_start = false;
                    beams[beamCount].release_end = false;
                    
                    beamCount++;
                }
                isDraggingBeam = false;
            }
        }

        if (wheel != 0) {
            camera.offset = GetMousePosition();
            camera.target = mouseWorldPos;
            camera.zoom += wheel * 0.125f;
            if (camera.zoom < 0.1f) camera.zoom = 0.1f;
        }

        BeginDrawing();
        ClearBackground(bgColor);
        BeginMode2D(camera);
        
        int numLines = 100;
        for (int i = -numLines; i <= numLines; i++) {
            DrawLineV((Vector2){i * GRID_SIZE, -numLines * GRID_SIZE},
                      (Vector2){i * GRID_SIZE, numLines * GRID_SIZE}, gridColor);
            DrawLineV((Vector2){-numLines * GRID_SIZE, i * GRID_SIZE},
                      (Vector2){numLines * GRID_SIZE, i * GRID_SIZE}, gridColor);
        }
        
        for (int i = 0; i < nodeCount; i++)
            DrawConstraint(nodes[i].position, nodes[i].constraint, nodes[i].angle, extConstraintColor);
            
        for (int i = 0; i < beamCount; i++) {
            beamColor = (Color){100, 180, 255, 255};
            if (currentMode == MODE_EDIT && i == hoveredBeam) {
                beamColor = RED;
            }
            DrawLineEx(nodes[beams[i].node_start_idx].position,
                       nodes[beams[i].node_end_idx].position, 4.0f, beamColor);
        }

        if (isDraggingBeam)
            DrawLineEx(nodes[dragStartNodeIdx].position, snappedPos, 4.0f, (Color){255, 255, 255, 100});

        for (int i = 0; i < nodeCount; i++) {
            DrawInternalConstraint(nodes[i].position, nodes[i].int_constraint, nodes[i].int_angle, intConstraintColor);
            DrawCircleV(nodes[i].position, 5.0f, nodeColor);
            DrawCircleLines(nodes[i].position.x, nodes[i].position.y, 5.0f, BLACK);
        }

        switch (currentMode) {
            case MODE_EDIT:
                for (int i = 0; i < nodeCount; i++)
                    DrawForceArrow(nodes[i].position, nodes[i].force, forceColor, NULL, true);
                if (!isDraggingBeam && hoveredNodeIdx == -1 && !fPressed) {
                    DrawCircleV(snappedPos, 5.0f, (Color){255, 80, 80, 150});
                }
                DrawDistributedLoads(nodes, beams, beamCount);
                break;
                
            case MODE_REACTIONS:
                for (int i = 0; i < nodeCount; i++) {
                    if (fabs(nodes[i].force.x) > 0.1f || fabs(nodes[i].force.y) > 0.1f) {
                        DrawForceArrow(nodes[i].position, nodes[i].force, forceColor, NULL, true);
                    }
                    if (nodes[i].constraint != CONSTRAINT_FREE) {
                        if (fabs(nodes[i].reaction_force.x) > 0.1f || fabs(nodes[i].reaction_force.y) > 0.1f) {
                            DrawForceArrow(nodes[i].position, nodes[i].reaction_force, reactionColor, "R", false);
                        }
                        if (fabs(nodes[i].reaction_moment) > 0.1f) {
                            DrawRing(nodes[i].position, 20.0f, 24.0f, 45, 315, 32, Fade(reactionColor, 0.7f));
                            DrawText(TextFormat("M: %.0f", nodes[i].reaction_moment),
                                     nodes[i].position.x + 28, nodes[i].position.y - 28, 14, reactionColor);
                        }
                    }
                }
                break;
                
            case MODE_N:
                for (int i = 0; i < beamCount; i++)
                    DrawBeamDiagram(nodes[beams[i].node_start_idx].position,
                                    nodes[beams[i].node_end_idx].position, beams[i].N_start,
                                    beams[i].N_end, diagramN, diagramScale);
                break;
                
            case MODE_T:
                for (int i = 0; i < beamCount; i++)
                    DrawBeamDiagram(nodes[beams[i].node_start_idx].position,
                                    nodes[beams[i].node_end_idx].position, beams[i].T_start,
                                    beams[i].T_end, diagramT, diagramScale);
                break;
                
            case MODE_M:
                for (int i = 0; i < beamCount; i++) {
                    DrawParabolicMomentDiagram(
                        nodes[beams[i].node_start_idx].position,
                        nodes[beams[i].node_end_idx].position, 
                        beams[i].M_start,
                        beams[i].M_end, 
                        beams[i].q_perp,
                        diagramM, 
                        diagramScale
                    );
                }
                break;
        }

        EndMode2D();

        DrawRectangle(0, 0, screenWidth, 40, Fade(BLACK, 0.8f));
        switch (currentMode) {
            case MODE_EDIT:
                DrawText("MODE: [ EDIT ] - Press TAB to Solve", 20, 10, 20, WHITE);
                break;
            case MODE_REACTIONS:
                DrawText("MODE: [ SUPPORT REACTIONS ] - (Up/Down to scale arrows/diagrams)", 20, 10, 20, reactionColor);
                break;
            case MODE_N:
                DrawText(TextFormat("MODE: [ AXIAL FORCE (N) ] - Scale: %.2fx", diagramScale), 20, 10, 20, diagramN);
                break;
            case MODE_T:
                DrawText(TextFormat("MODE: [ SHEAR FORCE (V) ] - Scale: %.2fx", diagramScale), 20, 10, 20, diagramT);
                break;
            case MODE_M:
                DrawText(TextFormat("MODE: [ BENDING MOMENT (M) ] - Scale: %.2fx", diagramScale), 20, 10, 20, diagramM);
                break;
        }

        EndDrawing();
    }

    free(nodes);
    free(beams);
    CloseWindow();
    return 0;
}