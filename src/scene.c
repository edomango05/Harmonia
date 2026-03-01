#include "scene.h"
#include "fem_solver.h"
#include "raymath.h"
#include <math.h>
#include <stdlib.h>

int GetNodeAtGridPosition(Node *nodes, int count, Vector2 pos) {
    for (int i = 0; i < count; i++)
        if (Vector2Distance(nodes[i].position, pos) < 1.0f)
            return i;
    return -1;
}

int GetNodeUnderMouse(Node *nodes, int count, Vector2 mouseWorldPos) {
    for (int i = 0; i < count; i++)
        if (Vector2Distance(nodes[i].position, mouseWorldPos) < 15.0f)
            return i;
    return -1;
}

bool BeamExists(Beam *beams, int count, int n1, int n2) {
    for (int i = 0; i < count; i++) {
        if ((beams[i].node_start_idx == n1 && beams[i].node_end_idx == n2) ||
            (beams[i].node_start_idx == n2 && beams[i].node_end_idx == n1))
            return true;
    }
    return false;
}

int GetHoveredBeam(Vector2 mousePos, Node *nodes, Beam *beams, int beamCount) {
    float minDist = 15.0f;
    int closestBeam = -1;

    for (int i = 0; i < beamCount; i++) {
        Vector2 p1 = nodes[beams[i].node_start_idx].position;
        Vector2 p2 = nodes[beams[i].node_end_idx].position;

        float l2 = Vector2DistanceSqr(p1, p2);
        if (l2 == 0) continue;

        float t = ((mousePos.x - p1.x) * (p2.x - p1.x) +
                   (mousePos.y - p1.y) * (p2.y - p1.y)) / l2;
        t = fmaxf(0.0f, fminf(1.0f, t));

        Vector2 projection = {p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y)};
        float dist = Vector2Distance(mousePos, projection);

        if (dist < minDist) {
            minDist = dist;
            closestBeam = i;
        }
    }
    return closestBeam;
}


AppScene InitScene(void) {
    AppScene scene = {0};
    scene.nodeCapacity = 10;
    scene.nodes = (Node *)calloc(scene.nodeCapacity, sizeof(Node));
    scene.beamCapacity = 10;
    scene.beams = (Beam *)calloc(scene.beamCapacity, sizeof(Beam));
    scene.currentMode = MODE_EDIT;
    scene.diagramScale = 0.5f;
    return scene;
}

void FreeScene(AppScene *scene) {
    free(scene->nodes);
    free(scene->beams);
}

int GetOrAddNode(AppScene *scene, Vector2 pos) {
    int idx = GetNodeAtGridPosition(scene->nodes, scene->nodeCount, pos);
    if (idx != -1) return idx;

    if (scene->nodeCount >= scene->nodeCapacity) {
        scene->nodeCapacity *= 2;
        scene->nodes = (Node *)realloc(scene->nodes, scene->nodeCapacity * sizeof(Node));
    }
    
    Node *n = &scene->nodes[scene->nodeCount];
    n->id = scene->nodeCount;
    n->position = pos;
    n->constraint = CONSTRAINT_FREE;
    n->angle = 0.0f;
    n->int_constraint = INT_CONSTRAINT_RIGID;
    n->int_angle = 0.0f;
    n->force = (Vector2){0, 0};
    
    return scene->nodeCount++;
}

void AddBeamIfValid(AppScene *scene, int startIdx, int endIdx) {
    if (startIdx == endIdx || BeamExists(scene->beams, scene->beamCount, startIdx, endIdx)) return;

    if (scene->beamCount >= scene->beamCapacity) {
        scene->beamCapacity *= 2;
        scene->beams = (Beam *)realloc(scene->beams, scene->beamCapacity * sizeof(Beam));
    }

    Beam *b = &scene->beams[scene->beamCount];
    b->node_start_idx = startIdx;
    b->node_end_idx = endIdx;
    b->q_perp = 0.0f;
    b->q_axial = 0.0f;
    b->N_start = 0.0f;
    b->N_end = 0.0f;
    b->T_start = 0.0f;
    b->T_end = 0.0f;
    b->M_start = 0.0f;
    b->M_end = 0.0f;
    b->release_start = false;
    b->release_end = false;

    scene->beamCount++;
}

void UpdateInputState(InputState *input, AppScene *scene, Camera2D camera) {
    input->mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
    input->snappedPos = (Vector2){roundf(input->mouseWorldPos.x / GRID_SIZE) * GRID_SIZE,
                                  roundf(input->mouseWorldPos.y / GRID_SIZE) * GRID_SIZE};
    input->hoveredNodeIdx = GetNodeUnderMouse(scene->nodes, scene->nodeCount, input->mouseWorldPos);
    input->hoveredBeam = GetHoveredBeam(input->mouseWorldPos, scene->nodes, scene->beams, scene->beamCount);
    input->wheel = GetMouseWheelMove();
    input->shiftPressed = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    input->fPressed = IsKeyDown(KEY_F);
}

void HandleForceDragging(AppScene *scene, InputState *input) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && input->hoveredNodeIdx != -1) {
        input->isDraggingForce = true;
        input->forceNodeIdx = input->hoveredNodeIdx;
    }
    
    if (input->isDraggingForce) {
        Vector2 rawForce = Vector2Subtract(input->mouseWorldPos, scene->nodes[input->forceNodeIdx].position);
        float mag = roundf(Vector2Length(rawForce) / 10.0f) * 10.0f;
        float ang = atan2f(rawForce.y, rawForce.x);
        
        if (input->shiftPressed) {
            float snapAngle = 15.0f * DEG2RAD;
            ang = roundf(ang / snapAngle) * snapAngle;
        }
        
        scene->nodes[input->forceNodeIdx].force.x = mag * cosf(ang);
        scene->nodes[input->forceNodeIdx].force.y = mag * sinf(ang);
    } else if (input->wheel != 0 && input->hoveredNodeIdx != -1 && Vector2Length(scene->nodes[input->hoveredNodeIdx].force) > 0) {
        float mag = Vector2Length(scene->nodes[input->hoveredNodeIdx].force);
        float ang = atan2f(scene->nodes[input->hoveredNodeIdx].force.y, scene->nodes[input->hoveredNodeIdx].force.x);
        
        mag = fmaxf(0.0f, mag + input->wheel * 1.0f);
        scene->nodes[input->hoveredNodeIdx].force.x = mag * cosf(ang);
        scene->nodes[input->hoveredNodeIdx].force.y = mag * sinf(ang);
        input->wheel = 0;
    }
    
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && input->isDraggingForce) {
        if (Vector2Length(scene->nodes[input->forceNodeIdx].force) < 10.0f) {
            scene->nodes[input->forceNodeIdx].force = (Vector2){0, 0};
        }
        input->isDraggingForce = false;
        input->forceNodeIdx = -1;
    }
}

void HandleNodeConstraints(AppScene *scene, InputState *input) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && input->hoveredNodeIdx != -1) {
        if (input->shiftPressed) {
            scene->nodes[input->hoveredNodeIdx].int_constraint = (scene->nodes[input->hoveredNodeIdx].int_constraint + 1) % INT_CONSTRAINT_COUNT;
            if (scene->nodes[input->hoveredNodeIdx].int_constraint == INT_CONSTRAINT_RIGID) {
                scene->nodes[input->hoveredNodeIdx].int_angle = 0.0f;
            }
        } else {
            scene->nodes[input->hoveredNodeIdx].constraint = (scene->nodes[input->hoveredNodeIdx].constraint + 1) % CONSTRAINT_COUNT;
            if (scene->nodes[input->hoveredNodeIdx].constraint == CONSTRAINT_FREE) {
                scene->nodes[input->hoveredNodeIdx].angle = 0.0f;
            }
        }
    }
    
    if (input->wheel != 0 && input->hoveredNodeIdx != -1) {
        if (input->shiftPressed) scene->nodes[input->hoveredNodeIdx].int_angle += input->wheel * 15.0f;
        else scene->nodes[input->hoveredNodeIdx].angle += input->wheel * 15.0f;
        input->wheel = 0;
    }
}

void ProcessAppInput(AppScene *scene, InputState *input, Camera2D *camera) {

    if (IsKeyPressed(KEY_C)) {
        scene->nodeCount = 0;
        scene->beamCount = 0;
        scene->currentMode = MODE_EDIT;
        
        input->isDraggingBeam = false;
        input->isDraggingForce = false;
    }

    if (IsKeyPressed(KEY_TAB)) {
        scene->currentMode = (scene->currentMode + 1) % MODE_COUNT;
        if (scene->currentMode == MODE_REACTIONS) {
            SolveFEM(scene->nodes, scene->nodeCount, scene->beams, scene->beamCount);
        }
    }

    if (scene->currentMode != MODE_EDIT) {
        if (IsKeyPressed(KEY_UP)) scene->diagramScale *= 1.5f;
        if (IsKeyPressed(KEY_DOWN)) scene->diagramScale /= 1.5f;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        camera->target = Vector2Add(camera->target, Vector2Scale(GetMouseDelta(), -1.0f / camera->zoom));
    }

    if (scene->currentMode == MODE_EDIT) {
        if (input->fPressed) {
            HandleForceDragging(scene, input);
            if (input->hoveredBeam != -1) {
                float load_step = 0.1f;
                if (IsKeyPressed(KEY_UP)) scene->beams[input->hoveredBeam].q_perp += load_step;
                if (IsKeyPressed(KEY_DOWN)) scene->beams[input->hoveredBeam].q_perp -= load_step;
                if (IsKeyPressed(KEY_RIGHT)) scene->beams[input->hoveredBeam].q_axial += load_step;
                if (IsKeyPressed(KEY_LEFT)) scene->beams[input->hoveredBeam].q_axial -= load_step;
            }
        } else {
            HandleNodeConstraints(scene, input);
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            input->dragStartNodeIdx = GetOrAddNode(scene, input->snappedPos);
            input->isDraggingBeam = true;
        }
        
        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT) && input->isDraggingBeam) {
            int endIdx = GetOrAddNode(scene, input->snappedPos);
            AddBeamIfValid(scene, input->dragStartNodeIdx, endIdx);
            input->isDraggingBeam = false;
        }
    }

    if (input->wheel != 0) {
        camera->offset = GetMousePosition();
        camera->target = input->mouseWorldPos;
        camera->zoom = fmaxf(0.1f, camera->zoom + input->wheel * 0.125f);
    }
}
