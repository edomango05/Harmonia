#include "graphics.h"
#include "rlgl.h"
#include "raymath.h"
#include <math.h>

void DrawConstraint(Vector2 pos, ConstraintType type, float angle, Color color) {
    if (type == CONSTRAINT_FREE) return;
    
    float size = 12.0f;
    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, 0.0f);
    rlRotatef(angle, 0.0f, 0.0f, 1.0f);

    switch (type) {
        case CONSTRAINT_PINNED:
            DrawTriangle((Vector2){0, 0}, (Vector2){-size, size * 1.5f},
                         (Vector2){size, size * 1.5f}, color);
            break;
        case CONSTRAINT_ROLLER:
            DrawTriangle((Vector2){0, 0}, (Vector2){-size, size * 1.5f},
                         (Vector2){size, size * 1.5f}, color);
            DrawLineEx((Vector2){-size * 1.5f, size * 1.8f},
                       (Vector2){size * 1.5f, size * 1.8f}, 3.0f, color);
            break;
        case CONSTRAINT_SLIDER:
            DrawRectangle(-size, size * 0.8f, size * 2.0f, size * 0.8f, color);
            DrawLineEx((Vector2){0, 0}, (Vector2){0, size * 0.8f}, 4.0f, color);
            DrawLineEx((Vector2){-size * 1.5f, size * 1.9f},
                       (Vector2){size * 1.5f, size * 1.9f}, 3.0f, color);
            break;
        case CONSTRAINT_SLEEVE:
            DrawRectangle(-size, size * 0.5f, size * 2.0f, size * 1.0f, color);
            DrawLineEx((Vector2){-size * 1.5f, size * 0.2f},
                       (Vector2){size * 1.5f, size * 0.2f}, 2.0f, color);
            DrawLineEx((Vector2){-size * 1.5f, size * 1.8f},
                       (Vector2){size * 1.5f, size * 1.8f}, 2.0f, color);
            break;
        case CONSTRAINT_FIXED:
            DrawRectangle(-size, 0, size * 2.0f, size * 1.5f, color);
            DrawLineEx((Vector2){-size * 1.5f, size * 1.5f},
                       (Vector2){size * 1.5f, size * 1.5f}, 3.0f, color);
            for (int i = -1; i <= 1; i++)
                DrawLineEx((Vector2){i * size, size * 1.5f},
                           (Vector2){i * size - 5, size * 2.0f}, 2.0f, color);
            break;
        default: break;
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
            DrawRectangleLinesEx((Rectangle){-size * 1.2f, -size * 0.8f, size * 2.4f, size * 1.6f}, 3.0f, color);
            DrawLineEx((Vector2){-size * 1.8f, 0}, (Vector2){size * 1.8f, 0}, 3.0f, color);
            break;
        case INT_CONSTRAINT_SLEEVE:
            DrawLineEx((Vector2){-size * 1.8f, -size * 0.5f},
                       (Vector2){size * 1.8f, -size * 0.5f}, 3.0f, color);
            DrawLineEx((Vector2){-size * 1.8f, size * 0.5f},
                       (Vector2){size * 1.8f, size * 0.5f}, 3.0f, color);
            break;
        default: break;
    }
    rlPopMatrix();
}

void DrawForceArrow(Vector2 startPos, Vector2 force, Color color, const char *label, bool showValue) {
    if (force.x == 0.0f && force.y == 0.0f) return;
    
    Vector2 endPos = Vector2Add(startPos, force);
    DrawLineEx(startPos, endPos, 4.0f, color);
    
    float angle = atan2f(force.y, force.x);
    float headSize = 12.0f;
    Vector2 p1 = {endPos.x - headSize * cosf(angle - PI / 6.0f),
                  endPos.y - headSize * sinf(angle - PI / 6.0f)};
    Vector2 p2 = {endPos.x - headSize * cosf(angle + PI / 6.0f),
                  endPos.y - headSize * sinf(angle + PI / 6.0f)};
    
    DrawTriangle(endPos, p1, p2, color);
    DrawTriangle(endPos, p2, p1, color);

    float mag = Vector2Length(force);
    if (label) {
        DrawText(TextFormat("%s: %.1f", label, mag), endPos.x + 10, endPos.y - 10, 14, color);
    } else if (showValue) {
        float displayAngle = -angle * RAD2DEG;
        if (displayAngle <= -180.0f) displayAngle += 360.0f;
        DrawText(TextFormat("%.0f u | %.0f°", mag, displayAngle), endPos.x + 10, endPos.y - 10, 14, color);
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
        DrawLineV((Vector2){Lerp(p1.x, p2.x, t), Lerp(p1.y, p2.y, t)},
                  (Vector2){Lerp(p4.x, p3.x, t), Lerp(p4.y, p3.y, t)},
                  Fade(color, 0.6f));
    }

    if (fabs(val1) > 0.1f)
        DrawText(TextFormat("%.1f", val1), p4.x + 5, p4.y - 10, 14, color);
    if (fabs(val2) > 0.1f)
        DrawText(TextFormat("%.1f", val2), p3.x + 5, p3.y - 10, 14, color);
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
        DrawText(TextFormat("q: %.0f N/m", fabs(q)), (int)mid.x - 20, (int)mid.y - 20, 15, ORANGE);
    }
}

void DrawParabolicMomentDiagram(Vector2 p1, Vector2 p2, float M_start, float M_end, float q_perp, Color color, float scale) {
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
        
        float M_x = M_start * (1.0f - x_phys / L_phys) + 
                    M_end * (x_phys / L_phys) + 
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