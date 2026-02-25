#pragma once

#include "types.h"
#include "raylib.h"

int GetNodeAtGridPosition(Node *nodes, int count, Vector2 pos);
int GetNodeUnderMouse(Node *nodes, int count, Vector2 mouseWorldPos);
bool BeamExists(Beam *beams, int count, int n1, int n2);
int GetHoveredBeam(Vector2 mousePos, Node *nodes, Beam *beams, int beamCount);