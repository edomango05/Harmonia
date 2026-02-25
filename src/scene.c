#include "scene.h"
#include "raymath.h"
#include <math.h>

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