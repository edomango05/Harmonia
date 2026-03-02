#include "graphics.h"

#include <math.h>
#include <time.h>

#include "raymath.h"
#include "rlgl.h"

void DrawConstraint(Vector2 pos, ConstraintType type, float angle, Color color) {
	if (type == CONSTRAINT_FREE) return;

	float size = 12.0f;
	rlPushMatrix();
	rlTranslatef(pos.x, pos.y, 0.0f);
	rlRotatef(angle, 0.0f, 0.0f, 1.0f);

	switch (type) {
		case CONSTRAINT_PINNED:
			DrawTriangle(
				(Vector2){0, 0},
				(Vector2){-size, size * 1.5f},
				(Vector2){size, size * 1.5f},
				color);
			break;
		case CONSTRAINT_ROLLER:
			DrawTriangle(
				(Vector2){0, 0},
				(Vector2){-size, size * 1.5f},
				(Vector2){size, size * 1.5f},
				color);
			DrawLineEx(
				(Vector2){-size * 1.5f, size * 1.8f},
				(Vector2){size * 1.5f, size * 1.8f},
				3.0f,
				color);
			break;
		case CONSTRAINT_SLIDER:
			DrawRectangle(-size, size * 0.8f, size * 2.0f, size * 0.8f, color);
			DrawLineEx((Vector2){0, 0}, (Vector2){0, size * 0.8f}, 4.0f, color);
			DrawLineEx(
				(Vector2){-size * 1.5f, size * 1.9f},
				(Vector2){size * 1.5f, size * 1.9f},
				3.0f,
				color);
			break;
		case CONSTRAINT_SLEEVE:
			DrawRectangle(-size, size * 0.5f, size * 2.0f, size * 1.0f, color);
			DrawLineEx(
				(Vector2){-size * 1.5f, size * 0.2f},
				(Vector2){size * 1.5f, size * 0.2f},
				2.0f,
				color);
			DrawLineEx(
				(Vector2){-size * 1.5f, size * 1.8f},
				(Vector2){size * 1.5f, size * 1.8f},
				2.0f,
				color);
			break;
		case CONSTRAINT_FIXED:
			DrawRectangle(-size, 0, size * 2.0f, size * 1.5f, color);
			DrawLineEx(
				(Vector2){-size * 1.5f, size * 1.5f},
				(Vector2){size * 1.5f, size * 1.5f},
				3.0f,
				color);
			for (int i = -1; i <= 1; i++)
				DrawLineEx(
					(Vector2){i * size, size * 1.5f},
					(Vector2){i * size - 5, size * 2.0f},
					2.0f,
					color);
			break;
		default:
			break;
	}
	rlPopMatrix();
}

void DrawInternalConstraint(Vector2 pos, InternalConstraintType type, float angle, Color color) {
	if (type == INT_CONSTRAINT_RIGID) return;

	float size = 14.0f;
	rlPushMatrix();
	rlTranslatef(pos.x, pos.y, 0.0f);
	rlRotatef(angle, 0.0f, 0.0f, 1.0f);

	switch (type) {
		case INT_CONSTRAINT_HINGE:
			DrawCircleLines(0, 0, size, color);
			DrawCircleLines(0, 0, size - 1.0f, color);
			break;
		case INT_CONSTRAINT_ROLLER:
			DrawCircleLines(0, 0, size, color);
			DrawLineEx((Vector2){-size * 1.8f, 0}, (Vector2){size * 1.8f, 0}, 3.0f, color);
			break;
		case INT_CONSTRAINT_SLIDER:
			DrawRectangleLinesEx(
				(Rectangle){-size * 1.2f, -size * 0.8f, size * 2.4f, size * 1.6f}, 3.0f, color);
			DrawLineEx((Vector2){-size * 1.8f, 0}, (Vector2){size * 1.8f, 0}, 3.0f, color);
			break;
		case INT_CONSTRAINT_SLEEVE:
			DrawLineEx(
				(Vector2){-size * 1.8f, -size * 0.5f},
				(Vector2){size * 1.8f, -size * 0.5f},
				3.0f,
				color);
			DrawLineEx(
				(Vector2){-size * 1.8f, size * 0.5f},
				(Vector2){size * 1.8f, size * 0.5f},
				3.0f,
				color);
			break;
		default:
			break;
	}
	rlPopMatrix();
}

void DrawForceArrow(
	Vector2 startPos, Vector2 force, Color color, const char *label, bool showValue) {
	if (force.x == 0.0f && force.y == 0.0f) return;

	Vector2 endPos = Vector2Add(startPos, force);
	DrawLineEx(startPos, endPos, 4.0f, color);

	float angle = atan2f(force.y, force.x);
	float headSize = 12.0f;
	Vector2 p1 = {
		endPos.x - headSize * cosf(angle - PI / 6.0f),
		endPos.y - headSize * sinf(angle - PI / 6.0f)};
	Vector2 p2 = {
		endPos.x - headSize * cosf(angle + PI / 6.0f),
		endPos.y - headSize * sinf(angle + PI / 6.0f)};

	DrawTriangle(endPos, p1, p2, color);
	DrawTriangle(endPos, p2, p1, color);

	float mag = Vector2Length(force);
	if (label) {
		DrawText(TextFormat("%s: %.1f", label, mag), endPos.x + 10, endPos.y - 10, 14, color);
	} else if (showValue) {
		float displayAngle = -angle * RAD2DEG;
		if (displayAngle <= -180.0f) displayAngle += 360.0f;
		DrawText(
			TextFormat("%.0f u | %.0f°", mag, displayAngle),
			endPos.x + 10,
			endPos.y - 10,
			14,
			color);
	}
}

void DrawBeamDiagram(Vector2 p1, Vector2 p2, float val1, float val2, Color color, float scale) {
	Vector2 dir = Vector2Normalize(Vector2Subtract(p2, p1));
	Vector2 normal = {-dir.y, dir.x};
	Vector2 p3 = Vector2Add(p2, Vector2Scale(normal, val2 * scale));
	Vector2 p4 = Vector2Add(p1, Vector2Scale(normal, val1 * scale));
	Color fillColor = Fade(color, 0.4f);

	DrawTriangle(p1, p2, p3, fillColor);
	DrawTriangle(p1, p3, p4, fillColor);
	DrawTriangle(p1, p3, p2, fillColor);
	DrawTriangle(p1, p4, p3, fillColor);

	DrawLineEx(p1, p4, 2.0f, color);
	DrawLineEx(p4, p3, 2.0f, color);
	DrawLineEx(p3, p2, 2.0f, color);

	int steps = 10;
	for (int i = 1; i < steps; i++) {
		float t = (float)i / steps;
		DrawLineV(
			(Vector2){Lerp(p1.x, p2.x, t), Lerp(p1.y, p2.y, t)},
			(Vector2){Lerp(p4.x, p3.x, t), Lerp(p4.y, p3.y, t)},
			Fade(color, 0.6f));
	}

	if (fabs(val1) > 0.1f) DrawText(TextFormat("%.1f", val1), p4.x + 5, p4.y - 10, 14, color);
	if (fabs(val2) > 0.1f) DrawText(TextFormat("%.1f", val2), p3.x + 5, p3.y - 10, 14, color);
}

void DrawDistributedLoads(Node *nodes, Beam *beams, int beamCount) {
	for (int i = 0; i < beamCount; i++) {
		float q = beams[i].q_perp;
		if (fabs(q) < 0.1f) continue;

		Vector2 p1 = nodes[beams[i].node_start_idx].position;
		Vector2 p2 = nodes[beams[i].node_end_idx].position;

		Vector2 dir = Vector2Normalize(Vector2Subtract(p2, p1));
		Vector2 normal = {-dir.y, dir.x};

		float arrow_height = 30.0f;
		float sign = (q > 0) ? -1.0f : 1.0f;

		Vector2 offsetP1 = Vector2Add(p1, Vector2Scale(normal, arrow_height * sign));
		Vector2 offsetP2 = Vector2Add(p2, Vector2Scale(normal, arrow_height * sign));

		Color loadColor = ORANGE;
		DrawLineEx(offsetP1, offsetP2, 2.0f, loadColor);

		float beam_length = Vector2Distance(p1, p2);
		int num_arrows = (int)(beam_length / 25.0f);
		if (num_arrows < 2) num_arrows = 2;

		for (int j = 0; j <= num_arrows; j++) {
			float t = (float)j / num_arrows;

			Vector2 base = {p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y)};
			Vector2 tail = Vector2Add(base, Vector2Scale(normal, arrow_height * sign));

			DrawLineEx(tail, base, 2.0f, loadColor);

			Vector2 pLeft = Vector2Add(base, Vector2Scale(dir, -4.0f));
			pLeft = Vector2Add(pLeft, Vector2Scale(normal, 6.0f * sign));

			Vector2 pRight = Vector2Add(base, Vector2Scale(dir, 4.0f));
			pRight = Vector2Add(pRight, Vector2Scale(normal, 6.0f * sign));

			DrawTriangle(base, pRight, pLeft, loadColor);
		}

		Vector2 mid = {(offsetP1.x + offsetP2.x) / 2, (offsetP1.y + offsetP2.y) / 2};
		DrawText(TextFormat("q: %.1f N/m", fabs(q)), (int)mid.x - 20, (int)mid.y - 20, 15, ORANGE);
	}
}

void DrawParabolicMomentDiagram(
	Vector2 p1, Vector2 p2, float M_start, float M_end, float q_perp, Color color, float scale) {
	float L_pixels = Vector2Distance(p1, p2);
	if (L_pixels < 0.01f) return;

	float L_phys = L_pixels / GRID_SIZE;
	Vector2 dir = Vector2Normalize(Vector2Subtract(p2, p1));
	Vector2 normal = {-dir.y, dir.x};

	int segments = 30;
	float dx_pixels = L_pixels / segments;

	Vector2 prevPt = p1;
	Vector2 prevDiagramPt = Vector2Add(p1, Vector2Scale(normal, M_start * scale));

	for (int i = 1; i <= segments; i++) {
		float x_pixels = i * dx_pixels;
		float x_phys = x_pixels / GRID_SIZE;

		float M_x = M_start * (1.0f - x_phys / L_phys) + M_end * (x_phys / L_phys) +
					q_perp * (x_phys * (L_phys - x_phys)) / 2.0f;

		Vector2 basePt = Vector2Add(p1, Vector2Scale(dir, x_pixels));
		Vector2 diagramPt = Vector2Add(basePt, Vector2Scale(normal, M_x * scale));

		DrawTriangle(prevPt, basePt, prevDiagramPt, Fade(color, 0.4f));
		DrawTriangle(basePt, diagramPt, prevDiagramPt, Fade(color, 0.4f));

		DrawLineEx(prevDiagramPt, diagramPt, 2.0f, color);

		if (i % 3 == 0) {
			DrawLineEx(basePt, diagramPt, 1.0f, Fade(color, 0.6f));
		}

		prevPt = basePt;
		prevDiagramPt = diagramPt;
	}
}

void DrawTopUI(AppMode mode, float scale, int width) {
	DrawRectangle(0, 0, width, 40, Fade(BLACK, 0.8f));
	switch (mode) {
		case MODE_EDIT:
			DrawText("MODE: [ EDIT ] - Press TAB to Solve", 20, 10, 20, WHITE);
			break;
		case MODE_REACTIONS:
			DrawText("MODE: [ SUPPORT REACTIONS ]", 20, 10, 20, COLOR_REACTION);
			break;
		case MODE_N:
			DrawText(
				TextFormat("MODE: [ AXIAL (N) ] - Scale: %.2fx", scale),
				20,
				10,
				20,
				COLOR_DIAGRAM_N);
			break;
		case MODE_T:
			DrawText(
				TextFormat("MODE: [ SHEAR (V) ] - Scale: %.2fx", scale),
				20,
				10,
				20,
				COLOR_DIAGRAM_T);
			break;
		case MODE_M:
			DrawText(
				TextFormat("MODE: [ MOMENT (M) ] - Scale: %.2fx", scale),
				20,
				10,
				20,
				COLOR_DIAGRAM_M);
			break;
	}
}

void RenderScene(AppScene *scene, InputState *input) {
	DrawBackgroundGrid();

	for (int i = 0; i < scene->nodeCount; i++) {
		DrawConstraint(
			scene->nodes[i].position,
			scene->nodes[i].constraint,
			scene->nodes[i].angle,
			COLOR_EXT_CONSTRAINT);
	}

	for (int i = 0; i < scene->beamCount; i++) {
		Color currentBeamColor =
			(scene->currentMode == MODE_EDIT && i == input->hoveredBeam) ? RED : COLOR_BEAM;
		DrawLineEx(
			scene->nodes[scene->beams[i].node_start_idx].position,
			scene->nodes[scene->beams[i].node_end_idx].position,
			4.0f,
			currentBeamColor);
	}

	if (input->isDraggingBeam) {
		DrawLineEx(
			scene->nodes[input->dragStartNodeIdx].position,
			input->snappedPos,
			4.0f,
			(Color){255, 255, 255, 100});
	}

	for (int i = 0; i < scene->nodeCount; i++) {
		DrawInternalConstraint(
			scene->nodes[i].position,
			scene->nodes[i].int_constraint,
			scene->nodes[i].int_angle,
			COLOR_INT_CONSTRAINT);
		DrawCircleV(scene->nodes[i].position, 5.0f, COLOR_NODE);
		DrawCircleLines(scene->nodes[i].position.x, scene->nodes[i].position.y, 5.0f, BLACK);
	}

	switch (scene->currentMode) {
		case MODE_EDIT:
			for (int i = 0; i < scene->nodeCount; i++) {
				DrawForceArrow(
					scene->nodes[i].position, scene->nodes[i].force, COLOR_FORCE, NULL, true);
			}
			if (!input->isDraggingBeam && input->hoveredNodeIdx == -1 && !input->fPressed) {
				DrawCircleV(input->snappedPos, 5.0f, (Color){255, 80, 80, 150});
			}
			DrawDistributedLoads(scene->nodes, scene->beams, scene->beamCount);
			break;

		case MODE_REACTIONS:
			for (int i = 0; i < scene->nodeCount; i++) {
				if (fabs(scene->nodes[i].force.x) > 0.1f || fabs(scene->nodes[i].force.y) > 0.1f) {
					DrawForceArrow(
						scene->nodes[i].position, scene->nodes[i].force, COLOR_FORCE, NULL, true);
				}
				if (scene->nodes[i].constraint != CONSTRAINT_FREE) {
					if (fabs(scene->nodes[i].reaction_force.x) > 0.1f ||
						fabs(scene->nodes[i].reaction_force.y) > 0.1f) {
						DrawForceArrow(
							scene->nodes[i].position,
							scene->nodes[i].reaction_force,
							COLOR_REACTION,
							"R",
							false);
					}
					if (fabs(scene->nodes[i].reaction_moment) > 0.1f) {
						DrawRing(
							scene->nodes[i].position,
							20.0f,
							24.0f,
							45,
							315,
							32,
							Fade(COLOR_REACTION, 0.7f));
						DrawText(
							TextFormat("M: %.0f", scene->nodes[i].reaction_moment),
							scene->nodes[i].position.x + 28,
							scene->nodes[i].position.y - 28,
							14,
							COLOR_REACTION);
					}
				}
			}
			break;

		case MODE_N:
			for (int i = 0; i < scene->beamCount; i++) {
				DrawBeamDiagram(
					scene->nodes[scene->beams[i].node_start_idx].position,
					scene->nodes[scene->beams[i].node_end_idx].position,
					scene->beams[i].N_start,
					scene->beams[i].N_end,
					COLOR_DIAGRAM_N,
					scene->diagramScale);
			}
			break;

		case MODE_T:
			for (int i = 0; i < scene->beamCount; i++) {
				DrawBeamDiagram(
					scene->nodes[scene->beams[i].node_start_idx].position,
					scene->nodes[scene->beams[i].node_end_idx].position,
					scene->beams[i].T_start,
					scene->beams[i].T_end,
					COLOR_DIAGRAM_T,
					scene->diagramScale);
			}
			break;

		case MODE_M:
			for (int i = 0; i < scene->beamCount; i++) {
				DrawParabolicMomentDiagram(
					scene->nodes[scene->beams[i].node_start_idx].position,
					scene->nodes[scene->beams[i].node_end_idx].position,
					scene->beams[i].M_start,
					scene->beams[i].M_end,
					scene->beams[i].q_perp,
					COLOR_DIAGRAM_M,
					scene->diagramScale);
			}
			break;
	}
}

void DrawBackgroundGrid(void) {
	int numLines = 100;
	for (int i = -numLines; i <= numLines; i++) {
		DrawLineV(
			(Vector2){i * GRID_SIZE, -numLines * GRID_SIZE},
			(Vector2){i * GRID_SIZE, numLines * GRID_SIZE},
			COLOR_GRID);
		DrawLineV(
			(Vector2){-numLines * GRID_SIZE, i * GRID_SIZE},
			(Vector2){numLines * GRID_SIZE, i * GRID_SIZE},
			COLOR_GRID);
	}
}