#pragma once

#include "types.h"
#include "raylib.h"

int GetNodeAtGridPosition(Node *nodes, int count, Vector2 pos);
int GetNodeUnderMouse(Node *nodes, int count, Vector2 mouseWorldPos);
bool BeamExists(Beam *beams, int count, int n1, int n2);
int GetHoveredBeam(Vector2 mousePos, Node *nodes, Beam *beams, int beamCount);

AppScene InitScene(void);
void FreeScene(AppScene *scene);
int GetOrAddNode(AppScene *scene, Vector2 pos);
void AddBeamIfValid(AppScene *scene, int startIdx, int endIdx);
void UpdateInputState(InputState *input, AppScene *scene, Camera2D camera);
void ProcessAppInput(AppScene *scene, InputState *input, Camera2D *camera);