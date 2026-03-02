#pragma once

#include <stdbool.h>

#include "raylib.h"

bool SolveLinearSystem(double *A, double *B, int N);
double SnapToZero(double value, double tolerance);
float PointToLineSegmentDistance(Vector2 p, Vector2 a, Vector2 b);