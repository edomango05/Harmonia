#include "math_utils.h"

#include <math.h>

bool SolveLinearSystem(double *A, double *B, int N) {
	for (int i = 0; i < N; i++) {
		int maxRow = i;
		for (int k = i + 1; k < N; k++)
			if (fabs(A[k * N + i]) > fabs(A[maxRow * N + i])) maxRow = k;
		for (int k = i; k < N; k++) {
			double tmp = A[i * N + k];
			A[i * N + k] = A[maxRow * N + k];
			A[maxRow * N + k] = tmp;
		}
		double tmp = B[i];
		B[i] = B[maxRow];
		B[maxRow] = tmp;

		if (fabs(A[i * N + i]) < 1e-10) return false;

		for (int k = i + 1; k < N; k++) {
			double factor = A[k * N + i] / A[i * N + i];
			for (int j = i; j < N; j++) A[k * N + j] -= factor * A[i * N + j];
			B[k] -= factor * B[i];
		}
	}
	for (int i = N - 1; i >= 0; i--) {
		double sum = 0.0;
		for (int j = i + 1; j < N; j++) sum += A[i * N + j] * B[j];
		B[i] = (B[i] - sum) / A[i * N + i];
	}
	return true;
}

double SnapToZero(double value, double tolerance) {
	return (fabs(value) < tolerance) ? 0.0 : value;
}

float PointToLineSegmentDistance(Vector2 p, Vector2 a, Vector2 b) {
	Vector2 ab = {b.x - a.x, b.y - a.y};
	Vector2 ap = {p.x - a.x, p.y - a.y};
	float magAB2 = ab.x * ab.x + ab.y * ab.y;

	if (magAB2 == 0.0f) return sqrtf(ap.x * ap.x + ap.y * ap.y);

	float t = (ap.x * ab.x + ap.y * ab.y) / magAB2;
	t = fmaxf(0.0f, fminf(1.0f, t));

	Vector2 closest = {a.x + t * ab.x, a.y + t * ab.y};
	return sqrtf((p.x - closest.x) * (p.x - closest.x) + (p.y - closest.y) * (p.y - closest.y));
}