#pragma once

#include "raylib.h"
#include "types.h"

extern const Color COLOR_BG;
extern const Color COLOR_GRID;
extern const Color COLOR_NODE;
extern const Color COLOR_BEAM;
extern const Color COLOR_EXT_CONSTRAINT;
extern const Color COLOR_INT_CONSTRAINT;
extern const Color COLOR_FORCE;
extern const Color COLOR_REACTION;
extern const Color COLOR_DIAGRAM_N;
extern const Color COLOR_DIAGRAM_T;
extern const Color COLOR_DIAGRAM_M;

void DrawConstraint(Vector2 pos, ConstraintType type, float angle, Color color);
void DrawInternalConstraint(Vector2 pos, InternalConstraintType type, float angle, Color color);
void DrawForceArrow(
	Vector2 startPos, Vector2 force, Color color, const char *label, bool showValue);
void DrawBeamDiagram(Vector2 p1, Vector2 p2, float val1, float val2, Color color, float scale);
void DrawDistributedLoads(Node *nodes, Beam *beams, int beamCount);
void DrawParabolicMomentDiagram(
	Vector2 p1, Vector2 p2, float M_start, float M_end, float q_perp, Color color, float scale);

void DrawBackgroundGrid(void);
void RenderScene(AppScene *scene, InputState *input);
void DrawTopUI(AppMode mode, float scale, int width);