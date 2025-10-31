#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// ============================================================================
// STRUTTURE DATI
// ============================================================================

typedef struct {
    int x;
    int y;
    int z;
} Point3D;

typedef struct {
    char axis;           // 'X', 'Y', 'Z'
    int steps;           // Numero step (signed per direzione)
    unsigned int speed;  // Divisore FPGA (0 = max speed)
} AxisMove;

typedef struct {
    unsigned int total_steps;
    unsigned int accel_steps;
    unsigned int const_steps;
    unsigned int decel_steps;
} TrajectoryProfile;

// ============================================================================
// FUNZIONI VELOCITÀ
// ============================================================================

// Converte Hz in divisore speed per FPGA (50MHz base)
unsigned int hz_to_speed_divisor(unsigned int hz, unsigned int max_speed_hz) {
    if (hz == 0) return 65535;  // Fermo (max divisore)
    if (hz >= max_speed_hz) return 0;  // Massima velocità (speed=0)

    // freq = 50MHz / (2 * divisore)
    // divisore = 50MHz / (2 * freq)
    return (50000000) / (2 * hz);
}

// Converte divisore FPGA in Hz
unsigned int speed_divisor_to_hz(unsigned int divisor) {
    if (divisor == 0) return 10000;  // Max FPGA
    if (divisor >= 25000) return 1000;  // Min pratico
    return (50000000) / (2 * divisor);
}

// Calcola velocità istantanea Hz basata sulla distanza nel profilo
unsigned int get_velocity_hz(unsigned int distance, TrajectoryProfile* profile,
                             unsigned int min_hz, unsigned int max_hz) {
    if (profile->total_steps == 0) return max_hz;

    if (distance < profile->accel_steps) {
        // Fase accelerazione: interpolazione lineare
        unsigned int vel_range = max_hz - min_hz;
        return min_hz + (vel_range * distance) / profile->accel_steps;
    }
    else if (distance < profile->accel_steps + profile->const_steps) {
        // Fase velocità costante
        return max_hz;
    }
    else {
        // Fase decelerazione: interpolazione lineare inversa
        unsigned int decel_distance = distance - profile->accel_steps - profile->const_steps;
        unsigned int vel_range = max_hz - min_hz;
        return max_hz - (vel_range * decel_distance) / profile->decel_steps;
    }
}

// ============================================================================
// BRESENHAM 3D BATCH DIRETTO - VERSIONE OTTIMIZZATA
// ============================================================================

int bresenham_3d_batch_direct(Point3D start, Point3D end, AxisMove* moves) {
    // Calcola delta e segni
    int dx = end.x - start.x;
    int dy = end.y - start.y;
    int dz = end.z - start.z;

    int sx = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
    int sy = (dy > 0) ? 1 : (dy < 0) ? -1 : 0;
    int sz = (dz > 0) ? 1 : (dz < 0) ? -1 : 0;

    dx = abs(dx);
    dy = abs(dy);
    dz = abs(dz);

    int move_count = 0;

    // Caso speciale: movimento nullo
    if (dx == 0 && dy == 0 && dz == 0) {
        return 0;
    }

    // ========================================================================
    // CASO 1: X dominante (dx >= dy && dx >= dz)
    // ========================================================================
    if (dx >= dy && dx >= dz) {
        int err_y = dx >> 1;
        int err_z = dx >> 1;
        int batch_steps = 0;

        for (int i = 0; i < dx; i++) {
            batch_steps++;
            err_y -= dy;
            err_z -= dz;

            int need_y = (err_y < 0);
            int need_z = (err_z < 0);

            if (i == dx - 1 || need_y || need_z) {
                if (batch_steps > 0) {
                    moves[move_count].axis = 'X';
                    moves[move_count].steps = batch_steps * sx;
                    moves[move_count].speed = 0;
                    move_count++;
                    batch_steps = 0;
                }

                if (need_y) {
                    moves[move_count].axis = 'Y';
                    moves[move_count].steps = 1 * sy;
                    moves[move_count].speed = 0;
                    move_count++;
                    err_y += dx;
                }

                if (need_z) {
                    moves[move_count].axis = 'Z';
                    moves[move_count].steps = 1 * sz;
                    moves[move_count].speed = 0;
                    move_count++;
                    err_z += dx;
                }
            }
        }
    }

    // ========================================================================
    // CASO 2: Y dominante (dy >= dx && dy >= dz)
    // ========================================================================
    else if (dy >= dx && dy >= dz) {
        int err_x = dy >> 1;
        int err_z = dy >> 1;
        int batch_steps = 0;

        for (int i = 0; i < dy; i++) {
            batch_steps++;
            err_x -= dx;
            err_z -= dz;

            int need_x = (err_x < 0);
            int need_z = (err_z < 0);

            if (i == dy - 1 || need_x || need_z) {
                if (batch_steps > 0) {
                    moves[move_count].axis = 'Y';
                    moves[move_count].steps = batch_steps * sy;
                    moves[move_count].speed = 0;
                    move_count++;
                    batch_steps = 0;
                }

                if (need_x) {
                    moves[move_count].axis = 'X';
                    moves[move_count].steps = 1 * sx;
                    moves[move_count].speed = 0;
                    move_count++;
                    err_x += dy;
                }

                if (need_z) {
                    moves[move_count].axis = 'Z';
                    moves[move_count].steps = 1 * sz;
                    moves[move_count].speed = 0;
                    move_count++;
                    err_z += dy;
                }
            }
        }
    }

    // ========================================================================
    // CASO 3: Z dominante (dz >= dx && dz >= dy)
    // ========================================================================
    else {
        int err_x = dz >> 1;
        int err_y = dz >> 1;
        int batch_steps = 0;

        for (int i = 0; i < dz; i++) {
            batch_steps++;
            err_x -= dx;
            err_y -= dy;

            int need_x = (err_x < 0);
            int need_y = (err_y < 0);

            if (i == dz - 1 || need_x || need_y) {
                if (batch_steps > 0) {
                    moves[move_count].axis = 'Z';
                    moves[move_count].steps = batch_steps * sz;
                    moves[move_count].speed = 0;
                    move_count++;
                    batch_steps = 0;
                }

                if (need_x) {
                    moves[move_count].axis = 'X';
                    moves[move_count].steps = 1 * sx;
                    moves[move_count].speed = 0;
                    move_count++;
                    err_x += dz;
                }

                if (need_y) {
                    moves[move_count].axis = 'Y';
                    moves[move_count].steps = 1 * sy;
                    moves[move_count].speed = 0;
                    move_count++;
                    err_y += dz;
                }
            }
        }
    }

    return move_count;
}

// ============================================================================
// APPLICA PROFILO VELOCITÀ AI COMANDI BATCH
// ============================================================================

void apply_speed_profile(AxisMove* moves, int move_count,
                        unsigned int min_hz, unsigned int max_hz,
                        unsigned int fpga_max_hz) {
    if (move_count == 0) return;

    // Calcola profilo trapezoidale basato su numero comandi
    TrajectoryProfile profile;
    profile.total_steps = move_count;
    profile.accel_steps = move_count / 4;
    profile.decel_steps = move_count / 4;
    profile.const_steps = move_count - profile.accel_steps - profile.decel_steps;

    // Assegna velocità ad ogni comando
    for (int i = 0; i < move_count; i++) {
        unsigned int current_hz = get_velocity_hz(i, &profile, min_hz, max_hz);
        moves[i].speed = hz_to_speed_divisor(current_hz, fpga_max_hz);
    }
}

// ============================================================================
// FUNZIONI DI VALIDAZIONE
// ============================================================================

int validate_trajectory(Point3D start, Point3D end, AxisMove* moves, int move_count) {
    Point3D current = start;

    for (int i = 0; i < move_count; i++) {
        switch (moves[i].axis) {
            case 'X': current.x += moves[i].steps; break;
            case 'Y': current.y += moves[i].steps; break;
            case 'Z': current.z += moves[i].steps; break;
        }
    }

    return (current.x == end.x && current.y == end.y && current.z == end.z);
}

// ============================================================================
// MAIN - TEST SPIRALE 3D
// ============================================================================

int main(void) {
    // Parametri di velocità
    unsigned int min_speed_hz = 1000;      // 1 kHz minima (partenza/arrivo)
    unsigned int max_speed_hz = 8000;      // 8 kHz massima (crociera)
    unsigned int fpga_max_speed_hz = 10000; // Limite FPGA 10 kHz

    // Genera 32 vertici di spirale 3D + ritorno all'origine
    int num_waypoints = 33;  // 32 spirale + 1 ritorno
    Point3D waypoints[33];

    for (int i = 0; i < num_waypoints - 1; i++) {
        double angle = (2.0 * M_PI * i) / (num_waypoints - 1);
        double radius = 100.0;
        double height = 200.0 * i / (num_waypoints - 1);

        waypoints[i].x = (int)(150 + radius * cos(angle));
        waypoints[i].y = (int)(150 + radius * sin(angle));
        waypoints[i].z = (int)height;
    }

    // Ritorno all'origine
    waypoints[32].x = 0;
    waypoints[32].y = 0;
    waypoints[32].z = 0;

    printf("╔════════════════════════════════════════════════════════════════════════╗\n");
    printf("║  BRESENHAM 3D BATCH DIRETTO - SPIRALE CON PROFILO TRAPEZOIDALE        ║\n");
    printf("║  Min: %u Hz  │  Max: %u Hz  │  FPGA Max: %u Hz                      ║\n",
           min_speed_hz, max_speed_hz, fpga_max_speed_hz);
    printf("╚════════════════════════════════════════════════════════════════════════╝\n\n");

    unsigned int total_step_count = 0;
    unsigned int total_batch_count = 0;
    unsigned int total_validations = 0;

    AxisMove moves[1000];

    for (int seg = 0; seg < num_waypoints - 1; seg++) {
        Point3D start = waypoints[seg];
        Point3D end = waypoints[seg + 1];

        // Genera comandi batch con Bresenham diretto
        int move_count = bresenham_3d_batch_direct(start, end, moves);

        // Applica profilo velocità trapezoidale
        apply_speed_profile(moves, move_count, min_speed_hz, max_speed_hz, fpga_max_speed_hz);

        // Valida traiettoria
        int is_valid = validate_trajectory(start, end, moves, move_count);
        if (!is_valid) {
            printf("❌ ERRORE VALIDAZIONE segmento %d!\n", seg + 1);
            continue;
        }
        total_validations++;

        // Calcola step totali
        int step_count = 0;
        for (int i = 0; i < move_count; i++) {
            step_count += abs(moves[i].steps);
        }

        // Calcola riduzione
        float compression = (1.0 - (float)move_count / step_count) * 100.0;

        // Stampa riassunto segmento
        printf("[%2d] (%3d,%3d,%3d)→(%3d,%3d,%3d) │ Step:%3d → Batch:%2d │ Rid:%5.1f%%",
               seg + 1, start.x, start.y, start.z, end.x, end.y, end.z,
               step_count, move_count, compression);

        // Mostra primi 3 comandi
        printf(" │ ");
        for (int i = 0; i < move_count && i < 3; i++) {
            unsigned int freq = speed_divisor_to_hz(moves[i].speed);
            printf("%c:%+d@%uHz ", moves[i].axis, moves[i].steps, freq);
        }
        if (move_count > 3) printf("...");
        printf("\n");

        total_step_count += step_count;
        total_batch_count += move_count;
    }

    printf("\n╔════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                          STATISTICHE GLOBALI                           ║\n");
    printf("╠════════════════════════════════════════════════════════════════════════╣\n");
    printf("║  Segmenti processati:          %3d / %3d                               ║\n",
           total_validations, num_waypoints - 1);
    printf("║  Step totali generati:         %u                                      ║\n",
           total_step_count);
    printf("║  Comandi batch FPGA:           %u                                      ║\n",
           total_batch_count);
    printf("║  Riduzione transazioni:        %.1f%%                                  ║\n",
           (1.0 - (float)total_batch_count / total_step_count) * 100.0);
    printf("║  Handshake risparmiati:        %u                                      ║\n",
           total_step_count - total_batch_count);
    printf("║  Efficienza media:             %.1f step/comando                       ║\n",
           (float)total_step_count / total_batch_count);
    printf("╚════════════════════════════════════════════════════════════════════════╝\n");

    if (total_validations == num_waypoints - 1) {
        printf("\n✅ TUTTI I SEGMENTI VALIDATI CORRETTAMENTE!\n");
    }

    return 0;
}
