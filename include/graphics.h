#pragma once

#include "types.h"
#include "raylib.h"

void DrawConstraint(Vector2 pos, ConstraintType type, float angle, Color color);
void DrawInternalConstraint(Vector2 pos, InternalConstraintType type, float angle, Color color);
void DrawForceArrow(Vector2 startPos, Vector2 force, Color color, const char *label, bool showValue);
void DrawBeamDiagram(Vector2 p1, Vector2 p2, float val1, float val2, Color color, float scale);
void DrawDistributedLoads(Node *nodes, Beam *beams, int beamCount);
void DrawParabolicMomentDiagram(Vector2 p1, Vector2 p2, float M_start, float M_end, float q_perp, Color color, float scale);