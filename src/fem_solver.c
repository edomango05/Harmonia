#include "fem_solver.h"
#include "math_utils.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

void SolveFEM(Node *nodes, int nodeCount, Beam *beams, int beamCount) {
    if (nodeCount == 0 || beamCount == 0) return;
    
    int numDOFs = nodeCount * 3;

    double *K_global = (double *)calloc(numDOFs * numDOFs, sizeof(double));
    double *K_orig = (double *)calloc(numDOFs * numDOFs, sizeof(double));
    double *F_global = (double *)calloc(numDOFs, sizeof(double));
    double *F_orig = (double *)calloc(numDOFs, sizeof(double));
    double *U_global = (double *)calloc(numDOFs, sizeof(double));

    double E = 210e6;
    double A_sez = 0.005;
    double I_sez = 5e-5;

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

        double c = dx / L;
        double s = dy / L;

        bool rNs = false, rTs = false, rMs = false;
        bool rNe = false, rTe = false, rMe = false;

        switch (nodes[n1].int_constraint) {
            case INT_CONSTRAINT_HINGE: rMs = true; break;
            case INT_CONSTRAINT_SLEEVE: rNs = true; break;
            case INT_CONSTRAINT_SLIDER: rTs = true; break;
            case INT_CONSTRAINT_ROLLER: rMs = true; rNs = true; break;
            default: break;
        }

        switch (nodes[n2].int_constraint) {
            case INT_CONSTRAINT_HINGE: rMe = true; break;
            case INT_CONSTRAINT_SLEEVE: rNe = true; break;
            case INT_CONSTRAINT_SLIDER: rTe = true; break;
            case INT_CONSTRAINT_ROLLER: rMe = true; rNe = true; break;
            default: break;
        }

        double EA_L = (E * A_sez) / L;
        double EI12_L3 = (12.0 * E * I_sez) / (L * L * L);
        double EI6_L2 = (6.0 * E * I_sez) / (L * L);
        double EI4_L = (4.0 * E * I_sez) / L;
        double EI2_L = (2.0 * E * I_sez) / L;
        double EI3_L3 = (3.0 * E * I_sez) / (L * L * L);
        double EI3_L2 = (3.0 * E * I_sez) / (L * L);
        double EI3_L = (3.0 * E * I_sez) / L;

        double k_loc[6][6] = {0};

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

        double T[6][6] = {{c, s, 0, 0, 0, 0}, {-s, c, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
                          {0, 0, 0, c, s, 0}, {0, 0, 0, -s, c, 0}, {0, 0, 0, 0, 0, 1}};

        double k_global_el[6][6] = {0};
        double temp[6][6] = {0};
        
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    temp[i][j] += k_loc[i][k] * T[k][j];

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    k_global_el[i][j] += T[k][i] * temp[k][j];

        int dof_map[6] = {n1 * 3, n1 * 3 + 1, n1 * 3 + 2,
                          n2 * 3, n2 * 3 + 1, n2 * 3 + 2};

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                K_global[dof_map[i] * numDOFs + dof_map[j]] += k_global_el[i][j];
                K_orig[dof_map[i] * numDOFs + dof_map[j]] += k_global_el[i][j];
            }
        }

        if (fabs(beams[b].q_perp) > 1e-5 || fabs(beams[b].q_axial) > 1e-5) {
            double q_p = beams[b].q_perp;
            double q_a = beams[b].q_axial;
            double f_eq_loc[6] = {0};

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

            for (int i = 0; i < 6; i++) {
                double f_glob_i = 0;
                for (int j = 0; j < 6; j++)
                    f_glob_i += T[j][i] * f_eq_loc[j];
                F_global[dof_map[i]] += f_glob_i;
            }
        }
    }

    for (int i = 0; i < numDOFs; i++)
        F_orig[i] = F_global[i];

    double penalty = 1e12; 

    for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].constraint == CONSTRAINT_FREE) continue;

        double alpha_math = -nodes[i].angle * PI / 180.0;
        
        double n_x = -sin(alpha_math);
        double n_y = cos(alpha_math);
        double t_x = cos(alpha_math);
        double t_y = sin(alpha_math);

        int u = i * 3;
        int v = i * 3 + 1;
        int theta = i * 3 + 2;

        bool block_n = false;   
        bool block_t = false;  
        bool block_theta = false; 

        switch (nodes[i].constraint) {
            case CONSTRAINT_PINNED: block_n = true; block_t = true; break;
            case CONSTRAINT_ROLLER: block_n = true; break;
            case CONSTRAINT_SLIDER: block_n = true; block_theta = true; break;
            case CONSTRAINT_SLEEVE: block_theta = true; break;
            case CONSTRAINT_FIXED: block_n = true; block_t = true; block_theta = true; break;
            default: break;
        }

        if (block_n) {
            K_global[u * numDOFs + u] += penalty * n_x * n_x;
            K_global[v * numDOFs + v] += penalty * n_y * n_y;
            K_global[u * numDOFs + v] += penalty * n_x * n_y;
            K_global[v * numDOFs + u] += penalty * n_x * n_y;
        }
        if (block_t) {
            K_global[u * numDOFs + u] += penalty * t_x * t_x;
            K_global[v * numDOFs + v] += penalty * t_y * t_y;
            K_global[u * numDOFs + v] += penalty * t_x * t_y;
            K_global[v * numDOFs + u] += penalty * t_x * t_y;
        }
        if (block_theta) {
            K_global[theta * numDOFs + theta] += penalty;
        }
    }

    for (int i = 0; i < numDOFs; i++) {
        if (fabs(K_global[i * numDOFs + i]) < 1e-12) {
            K_global[i * numDOFs + i] = 1.0;
            F_global[i] = 0.0;
        }
    }

    if (!SolveLinearSystem(K_global, F_global, numDOFs)) {
        for (int i = 0; i < beamCount; i++) {
            beams[i].N_start = 0; beams[i].N_end = 0;
            beams[i].T_start = 0; beams[i].T_end = 0;
            beams[i].M_start = 0; beams[i].M_end = 0;
        }
        free(K_global); free(K_orig); free(F_global); free(F_orig); free(U_global);
        return;
    }

    for (int i = 0; i < numDOFs; i++)
        U_global[i] = F_global[i];

    double zero_tolerance = 0.5;

    for (int b = 0; b < beamCount; b++) {
        int n1 = beams[b].node_start_idx;
        int n2 = beams[b].node_end_idx;
        double dx = (nodes[n2].position.x - nodes[n1].position.x) / GRID_SIZE;
        double dy = -(nodes[n2].position.y - nodes[n1].position.y) / GRID_SIZE;
        double L = sqrt(dx * dx + dy * dy);
        double c = dx / L;
        double s = dy / L;

        bool rNs = false, rTs = false, rMs = false;
        bool rNe = false, rTe = false, rMe = false;

        switch (nodes[n1].int_constraint) {
            case INT_CONSTRAINT_HINGE: rMs = true; break;
            case INT_CONSTRAINT_SLEEVE: rNs = true; break;
            case INT_CONSTRAINT_SLIDER: rTs = true; break;
            case INT_CONSTRAINT_ROLLER: rMs = true; rNs = true; break;
            default: break;
        }
        
        switch (nodes[n2].int_constraint) {
            case INT_CONSTRAINT_HINGE: rMe = true; break;
            case INT_CONSTRAINT_SLEEVE: rNe = true; break;
            case INT_CONSTRAINT_SLIDER: rTe = true; break;
            case INT_CONSTRAINT_ROLLER: rMe = true; rNe = true; break;
            default: break;
        }

        double EA_L = (E * A_sez) / L;
        double EI12_L3 = (12.0 * E * I_sez) / (L * L * L);
        double EI6_L2 = (6.0 * E * I_sez) / (L * L);
        double EI4_L = (4.0 * E * I_sez) / L;
        double EI2_L = (2.0 * E * I_sez) / L;
        double EI3_L3 = (3.0 * E * I_sez) / (L * L * L);
        double EI3_L2 = (3.0 * E * I_sez) / (L * L);
        double EI3_L = (3.0 * E * I_sez) / L;

        double k_loc[6][6] = {0};

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

        double T[6][6] = {{c, s, 0, 0, 0, 0}, {-s, c, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
                          {0, 0, 0, c, s, 0}, {0, 0, 0, -s, c, 0}, {0, 0, 0, 0, 0, 1}};

        double u_glob[6] = {U_global[n1 * 3], U_global[n1 * 3 + 1], U_global[n1 * 3 + 2],
                            U_global[n2 * 3], U_global[n2 * 3 + 1], U_global[n2 * 3 + 2]};
        double u_loc[6] = {0};
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) u_loc[i] += T[i][j] * u_glob[j];

        double f_loc_b[6] = {0};
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) f_loc_b[i] += k_loc[i][j] * u_loc[j];

        double q_p = beams[b].q_perp;
        double q_a = beams[b].q_axial;

        if (!rNs && !rNe) { f_loc_b[0] -= -(q_a * L) / 2.0; f_loc_b[3] -= -(q_a * L) / 2.0; }
        if (!rMs && !rMe) {
            f_loc_b[1] -= -(q_p * L) / 2.0; f_loc_b[2] -= -(q_p * L * L) / 12.0;
            f_loc_b[4] -= -(q_p * L) / 2.0; f_loc_b[5] -= (q_p * L * L) / 12.0;
        } else if (rMs && !rMe) {
            f_loc_b[1] -= -(3.0 / 8.0 * q_p * L); f_loc_b[4] -= -(5.0 / 8.0 * q_p * L); f_loc_b[5] -= (q_p * L * L) / 8.0;
        } else if (!rMs && rMe) {
            f_loc_b[1] -= -(5.0 / 8.0 * q_p * L); f_loc_b[2] -= -(q_p * L * L) / 8.0; f_loc_b[4] -= -(3.0 / 8.0 * q_p * L);
        } else {
            f_loc_b[1] -= -(q_p * L) / 2.0; f_loc_b[4] -= -(q_p * L) / 2.0;
        }

        beams[b].N_start = (float)SnapToZero(-f_loc_b[0], zero_tolerance);
        beams[b].N_end = (float)SnapToZero(f_loc_b[3], zero_tolerance);
        beams[b].T_start = (float)SnapToZero(f_loc_b[1], zero_tolerance);
        beams[b].T_end = (float)SnapToZero(-f_loc_b[4], zero_tolerance);
        beams[b].M_start = (float)SnapToZero(-f_loc_b[2], zero_tolerance);
        beams[b].M_end = (float)SnapToZero(f_loc_b[5], zero_tolerance);
    }

    for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].constraint != CONSTRAINT_FREE) {
            double alpha_math = -nodes[i].angle * PI / 180.0;
            double n_x = -sin(alpha_math);
            double n_y = cos(alpha_math);
            double t_x = cos(alpha_math);
            double t_y = sin(alpha_math);

            int u = i * 3;
            int v = i * 3 + 1;
            int theta = i * 3 + 2;

            bool block_n = false;
            bool block_t = false;
            bool block_theta = false;

            switch (nodes[i].constraint) {
                case CONSTRAINT_PINNED: block_n = true; block_t = true; break;
                case CONSTRAINT_ROLLER: block_n = true; break;
                case CONSTRAINT_SLIDER: block_n = true; block_theta = true; break;
                case CONSTRAINT_SLEEVE: block_theta = true; break; 
                case CONSTRAINT_FIXED: block_n = true; block_t = true; block_theta = true; break;
                default: break;
            }

            double Rx_raw = 0.0, Ry_raw = 0.0, R_mom = 0.0;
            for (int j = 0; j < numDOFs; j++) {
                Rx_raw += K_orig[u * numDOFs + j] * U_global[j];
                Ry_raw += K_orig[v * numDOFs + j] * U_global[j];
                R_mom  += K_orig[theta * numDOFs + j] * U_global[j];
            }
            Rx_raw -= F_orig[u];
            Ry_raw -= F_orig[v];
            R_mom  -= F_orig[theta];

            double R_n = 0.0, R_t = 0.0;
            double proj_n = Rx_raw * n_x + Ry_raw * n_y;
            double proj_t = Rx_raw * t_x + Ry_raw * t_y;

            if (block_n) R_n = proj_n;
            if (block_t) R_t = proj_t;
            if (!block_theta) R_mom = 0.0;

            double Rx = R_n * n_x + R_t * t_x;
            double Ry = R_n * n_y + R_t * t_y;

            nodes[i].reaction_force.x = (float)SnapToZero(Rx, zero_tolerance);
            nodes[i].reaction_force.y = (float)SnapToZero(-Ry, zero_tolerance); 
            nodes[i].reaction_moment = (float)SnapToZero(R_mom, zero_tolerance);
            
        } else {
            nodes[i].reaction_force = (Vector2){0, 0};
            nodes[i].reaction_moment = 0.0f;
        }
    }

    free(K_global); free(K_orig); free(F_global); free(F_orig); free(U_global);
}