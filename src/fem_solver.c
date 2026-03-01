#include "fem_solver.h"
#include "math_utils.h"
#include <math.h>
#include <stdlib.h>

static void GetBeamReleases(int c1, int c2, bool *rNs, bool *rTs, bool *rMs, bool *rNe, bool *rTe, bool *rMe) {
    *rNs = *rTs = *rMs = *rNe = *rTe = *rMe = false;
    
    if (c1 == INT_CONSTRAINT_HINGE) *rMs = true;
    else if (c1 == INT_CONSTRAINT_SLEEVE) *rNs = true;
    else if (c1 == INT_CONSTRAINT_SLIDER) *rTs = true;
    else if (c1 == INT_CONSTRAINT_ROLLER) { *rMs = true; *rNs = true; }

    if (c2 == INT_CONSTRAINT_HINGE) *rMe = true;
    else if (c2 == INT_CONSTRAINT_SLEEVE) *rNe = true;
    else if (c2 == INT_CONSTRAINT_SLIDER) *rTe = true;
    else if (c2 == INT_CONSTRAINT_ROLLER) { *rMe = true; *rNe = true; }
}

static void GetElementLocalMatrices(double L, double E, double A_sez, double I_sez, double q_p, double q_a, 
                                    int c1, int c2, double k_loc[6][6], double f_eq_loc[6]) {
    bool rNs, rTs, rMs, rNe, rTe, rMe;
    GetBeamReleases(c1, c2, &rNs, &rTs, &rMs, &rNe, &rTe, &rMe);

    for(int i=0; i<6; i++) { f_eq_loc[i] = 0; for(int j=0; j<6; j++) k_loc[i][j] = 0; }

    double EA_L = (E * A_sez) / L;
    double EI12_L3 = (12.0 * E * I_sez) / (L * L * L);
    double EI6_L2 = (6.0 * E * I_sez) / (L * L);
    double EI4_L = (4.0 * E * I_sez) / L;
    double EI2_L = (2.0 * E * I_sez) / L;
    double EI3_L3 = (3.0 * E * I_sez) / (L * L * L);
    double EI3_L2 = (3.0 * E * I_sez) / (L * L);
    double EI3_L = (3.0 * E * I_sez) / L;

    if (!rNs && !rNe) { k_loc[0][0] = EA_L; k_loc[0][3] = -EA_L; k_loc[3][0] = -EA_L; k_loc[3][3] = EA_L; }
    if (rTs || rTe) {
        double EI_L = (E * I_sez) / L; k_loc[2][2] = EI_L; k_loc[2][5] = -EI_L; k_loc[5][2] = -EI_L; k_loc[5][5] = EI_L;
    } else if (!rMs && !rMe) {
        k_loc[1][1] = EI12_L3; k_loc[1][2] = EI6_L2; k_loc[1][4] = -EI12_L3; k_loc[1][5] = EI6_L2;
        k_loc[2][1] = EI6_L2; k_loc[2][2] = EI4_L; k_loc[2][4] = -EI6_L2; k_loc[2][5] = EI2_L;
        k_loc[4][1] = -EI12_L3; k_loc[4][2] = -EI6_L2; k_loc[4][4] = EI12_L3; k_loc[4][5] = -EI6_L2;
        k_loc[5][1] = EI6_L2; k_loc[5][2] = EI2_L; k_loc[5][4] = -EI6_L2; k_loc[5][5] = EI4_L;
    } else if (rMs && !rMe) {
        k_loc[1][1] = EI3_L3; k_loc[1][4] = -EI3_L3; k_loc[1][5] = EI3_L2;
        k_loc[4][1] = -EI3_L3; k_loc[4][4] = EI3_L3; k_loc[4][5] = -EI3_L2;
        k_loc[5][1] = EI3_L2; k_loc[5][4] = -EI3_L2; k_loc[5][5] = EI3_L;
    } else if (!rMs && rMe) {
        k_loc[1][1] = EI3_L3; k_loc[1][2] = EI3_L2; k_loc[1][4] = -EI3_L3;
        k_loc[2][1] = EI3_L2; k_loc[2][2] = EI3_L; k_loc[2][4] = -EI3_L2;
        k_loc[4][1] = -EI3_L3; k_loc[4][2] = -EI3_L2; k_loc[4][4] = EI3_L3;
    }
    if (fabs(q_p) > 1e-5 || fabs(q_a) > 1e-5) {
        if (!rNs && !rNe) { f_eq_loc[0] = -(q_a * L) / 2.0; f_eq_loc[3] = -(q_a * L) / 2.0; }
        if (!rMs && !rMe) {
            f_eq_loc[1] = -(q_p * L) / 2.0; f_eq_loc[2] = -(q_p * L * L) / 12.0;
            f_eq_loc[4] = -(q_p * L) / 2.0; f_eq_loc[5] = (q_p * L * L) / 12.0;
        } else if (rMs && !rMe) {
            f_eq_loc[1] = -(3.0 / 8.0 * q_p * L); f_eq_loc[4] = -(5.0 / 8.0 * q_p * L); f_eq_loc[5] = (q_p * L * L) / 8.0;
        } else if (!rMs && rMe) {
            f_eq_loc[1] = -(5.0 / 8.0 * q_p * L); f_eq_loc[2] = -(q_p * L * L) / 8.0; f_eq_loc[4] = -(3.0 / 8.0 * q_p * L);
        } else {
            f_eq_loc[1] = -(q_p * L) / 2.0; f_eq_loc[4] = -(q_p * L) / 2.0;
        }
    }
}

static void GetTransformationMatrix(double c, double s, double T[6][6]) {
    double temp[6][6] = {
        {c, s, 0, 0, 0, 0}, {-s, c, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
        {0, 0, 0, c, s, 0}, {0, 0, 0, -s, c, 0}, {0, 0, 0, 0, 0, 1}
    };
    for(int i=0; i<6; i++) for(int j=0; j<6; j++) T[i][j] = temp[i][j];
}


void SolveFEM(Node *nodes, int nodeCount, Beam *beams, int beamCount) {
    if (nodeCount == 0 || beamCount == 0) return;
    
    int numDOFs = nodeCount * 3;
    double E = 210e6, A_sez = 0.005, I_sez = 5e-5, zero_tol = 0.5, penalty = 1e12;

    double *K_global = (double *)calloc(numDOFs * numDOFs, sizeof(double));
    double *K_orig = (double *)calloc(numDOFs * numDOFs, sizeof(double));
    double *F_global = (double *)calloc(numDOFs, sizeof(double));
    double *F_orig = (double *)calloc(numDOFs, sizeof(double));
    double *U_global = (double *)calloc(numDOFs, sizeof(double));

    // EXTERNAL NODE PAYLOAD
    for (int i = 0; i < nodeCount; i++) {
        F_global[i * 3 + 0] = (double)nodes[i].force.x;
        F_global[i * 3 + 1] = -(double)nodes[i].force.y;
        F_global[i * 3 + 2] = 0.0;
    }

    for (int b = 0; b < beamCount; b++) {
        int n1 = beams[b].node_start_idx;
        int n2 = beams[b].node_end_idx;
        double dx = (nodes[n2].position.x - nodes[n1].position.x) / GRID_SIZE;
        double dy = -(nodes[n2].position.y - nodes[n1].position.y) / GRID_SIZE;
        double L = sqrt(dx * dx + dy * dy);
        
        if (L < 0.01) continue;

        double k_loc[6][6], f_eq_loc[6], T[6][6], k_global_el[6][6] = {0}, temp[6][6] = {0};
        
        GetElementLocalMatrices(L, E, A_sez, I_sez, beams[b].q_perp, beams[b].q_axial, 
                                nodes[n1].int_constraint, nodes[n2].int_constraint, k_loc, f_eq_loc);
        GetTransformationMatrix(dx / L, dy / L, T);

        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) for (int k = 0; k < 6; k++) temp[i][j] += k_loc[i][k] * T[k][j];
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) for (int k = 0; k < 6; k++) k_global_el[i][j] += T[k][i] * temp[k][j];

        int dof_map[6] = {n1 * 3, n1 * 3 + 1, n1 * 3 + 2, n2 * 3, n2 * 3 + 1, n2 * 3 + 2};

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                K_global[dof_map[i] * numDOFs + dof_map[j]] += k_global_el[i][j];
                K_orig[dof_map[i] * numDOFs + dof_map[j]] += k_global_el[i][j];
            }
            double f_glob_i = 0;
            for (int j = 0; j < 6; j++) f_glob_i += T[j][i] * f_eq_loc[j];
            F_global[dof_map[i]] += f_glob_i;
        }
    }

    for (int i = 0; i < numDOFs; i++) F_orig[i] = F_global[i];

    // COUNTOUR CONDITION WITH PENALTY METHOD
    for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].constraint == CONSTRAINT_FREE) continue;

        double alpha_math = -nodes[i].angle * PI / 180.0;
        double n_x = -sin(alpha_math), n_y = cos(alpha_math);
        double t_x = cos(alpha_math), t_y = sin(alpha_math);
        int u = i * 3, v = i * 3 + 1, theta = i * 3 + 2;
        bool b_n = false, b_t = false, b_theta = false;

        switch (nodes[i].constraint) {
            case CONSTRAINT_PINNED: b_n = b_t = true; break;
            case CONSTRAINT_ROLLER: b_n = true; break;
            case CONSTRAINT_SLIDER: b_n = b_theta = true; break;
            case CONSTRAINT_SLEEVE: b_theta = true; break;
            case CONSTRAINT_FIXED:  b_n = b_t = b_theta = true; break;
            default: break;
        }

        if (b_n) { K_global[u*numDOFs+u] += penalty*n_x*n_x; K_global[v*numDOFs+v] += penalty*n_y*n_y; 
                   K_global[u*numDOFs+v] += penalty*n_x*n_y; K_global[v*numDOFs+u] += penalty*n_x*n_y; }
        if (b_t) { K_global[u*numDOFs+u] += penalty*t_x*t_x; K_global[v*numDOFs+v] += penalty*t_y*t_y; 
                   K_global[u*numDOFs+v] += penalty*t_x*t_y; K_global[v*numDOFs+u] += penalty*t_x*t_y; }
        if (b_theta) K_global[theta*numDOFs+theta] += penalty;
    }

    // STABILIZATION
    for (int i = 0; i < numDOFs; i++) if (fabs(K_global[i * numDOFs + i]) < 1e-12) { K_global[i * numDOFs + i] = 1.0; F_global[i] = 0.0; }


    if (!SolveLinearSystem(K_global, F_global, numDOFs)) {
        for (int i = 0; i < beamCount; i++) beams[i].N_start = beams[i].N_end = beams[i].T_start = beams[i].T_end = beams[i].M_start = beams[i].M_end = 0;
        free(K_global); free(K_orig); free(F_global); free(F_orig); free(U_global);
        return;
    }
    for (int i = 0; i < numDOFs; i++) U_global[i] = F_global[i];

    // INTERNAL STRESS
    for (int b = 0; b < beamCount; b++) {
        int n1 = beams[b].node_start_idx;
        int n2 = beams[b].node_end_idx;
        double dx = (nodes[n2].position.x - nodes[n1].position.x) / GRID_SIZE;
        double dy = -(nodes[n2].position.y - nodes[n1].position.y) / GRID_SIZE;
        double L = sqrt(dx * dx + dy * dy);
        
        double k_loc[6][6], f_eq_loc[6], T[6][6];
        GetElementLocalMatrices(L, E, A_sez, I_sez, beams[b].q_perp, beams[b].q_axial, 
                                nodes[n1].int_constraint, nodes[n2].int_constraint, k_loc, f_eq_loc);
        GetTransformationMatrix(dx / L, dy / L, T);

        double u_glob[6] = {U_global[n1*3], U_global[n1*3+1], U_global[n1*3+2], U_global[n2*3], U_global[n2*3+1], U_global[n2*3+2]};
        double u_loc[6] = {0}, f_loc_b[6] = {0};
        
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) u_loc[i] += T[i][j] * u_glob[j];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) f_loc_b[i] += k_loc[i][j] * u_loc[j];
            f_loc_b[i] -= f_eq_loc[i]; // Sottraiamo le forze nodali equivalenti!
        }

        beams[b].N_start = (float)SnapToZero(-f_loc_b[0], zero_tol); beams[b].N_end = (float)SnapToZero(f_loc_b[3], zero_tol);
        beams[b].T_start = (float)SnapToZero(f_loc_b[1], zero_tol);  beams[b].T_end = (float)SnapToZero(-f_loc_b[4], zero_tol);
        beams[b].M_start = (float)SnapToZero(-f_loc_b[2], zero_tol); beams[b].M_end = (float)SnapToZero(f_loc_b[5], zero_tol);
    }

    // CONSTRAINTS REACTIONS
    for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].constraint != CONSTRAINT_FREE) {
            double alpha = -nodes[i].angle * PI / 180.0;
            double n_x = -sin(alpha), n_y = cos(alpha), t_x = cos(alpha), t_y = sin(alpha);
            int u = i*3, v = i*3+1, theta = i*3+2;
            
            bool b_n = false, b_t = false, b_theta = false;
            switch (nodes[i].constraint) {
                case CONSTRAINT_PINNED: b_n = b_t = true; break;
                case CONSTRAINT_ROLLER: b_n = true; break;
                case CONSTRAINT_SLIDER: b_n = b_theta = true; break;
                case CONSTRAINT_SLEEVE: b_theta = true; break; 
                case CONSTRAINT_FIXED:  b_n = b_t = b_theta = true; break;
                default: break;
            }

            double Rx_raw = -F_orig[u], Ry_raw = -F_orig[v], R_mom = -F_orig[theta];
            for (int j = 0; j < numDOFs; j++) {
                Rx_raw += K_orig[u * numDOFs + j] * U_global[j];
                Ry_raw += K_orig[v * numDOFs + j] * U_global[j];
                R_mom  += K_orig[theta * numDOFs + j] * U_global[j];
            }

            double R_n = b_n ? (Rx_raw * n_x + Ry_raw * n_y) : 0.0;
            double R_t = b_t ? (Rx_raw * t_x + Ry_raw * t_y) : 0.0;
            if (!b_theta) R_mom = 0.0;

            nodes[i].reaction_force.x = (float)SnapToZero(R_n * n_x + R_t * t_x, zero_tol);
            nodes[i].reaction_force.y = (float)SnapToZero(-(R_n * n_y + R_t * t_y), zero_tol); 
            nodes[i].reaction_moment = (float)SnapToZero(R_mom, zero_tol);
        } else {
            nodes[i].reaction_force = (Vector2){0, 0}; nodes[i].reaction_moment = 0.0f;
        }
    }

    free(K_global); free(K_orig); free(F_global); free(F_orig); free(U_global);
}