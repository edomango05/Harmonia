#pragma once

#include <stdbool.h>

#include "raylib.h"

#define GRID_SIZE 50.0f

typedef enum { MODE_EDIT = 0, MODE_REACTIONS, MODE_N, MODE_T, MODE_M, MODE_COUNT } AppMode;

typedef enum {
	CONSTRAINT_FREE = 0,
	CONSTRAINT_PINNED,
	CONSTRAINT_ROLLER,
	CONSTRAINT_SLIDER,
	CONSTRAINT_SLEEVE,
	CONSTRAINT_FIXED,
	CONSTRAINT_COUNT
} ConstraintType;

typedef enum {
	INT_CONSTRAINT_RIGID = 0,
	INT_CONSTRAINT_HINGE,
	INT_CONSTRAINT_ROLLER,
	INT_CONSTRAINT_SLIDER,
	INT_CONSTRAINT_SLEEVE,
	INT_CONSTRAINT_COUNT
} InternalConstraintType;

typedef struct {
	int id;
	Vector2 position;
	ConstraintType constraint;
	float angle;
	InternalConstraintType int_constraint;
	float int_angle;
	Vector2 force;
	Vector2 reaction_force;
	float reaction_moment;
} Node;

typedef struct {
	int node_start_idx;
	int node_end_idx;
	double q_perp;
	double q_axial;
	float N_start, N_end;
	float T_start, T_end;
	float M_start, M_end;
	bool release_start;
	bool release_end;
} Beam;

typedef struct {
	Node *nodes;
	int nodeCount;
	int nodeCapacity;
	Beam *beams;
	int beamCount;
	int beamCapacity;
	AppMode currentMode;
	float diagramScale;
} AppScene;

typedef struct {
	bool isDraggingBeam;
	int dragStartNodeIdx;
	bool isDraggingForce;
	int forceNodeIdx;
	Vector2 mouseWorldPos;
	Vector2 snappedPos;
	int hoveredNodeIdx;
	int hoveredBeam;
	float wheel;
	bool shiftPressed;
	bool fPressed;
} InputState;