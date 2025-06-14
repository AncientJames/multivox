#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <sched.h>
#include <time.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>

#include "rammel.h"
#include "gadget.h"
#include "rotation.h"
#include "slicemap.h"
#include "gpio.h"
#include "input.h"

#ifdef VERTICAL_SCAN
    #include "colscatter.h"
    #define TRAIL_STACK 1
#else
    #define TRAIL_STACK 2
    #define HORIZONTAL_PRESLICE
    //#define SLICER_PROFILE
#endif


#define BPC_MAX 3
#define DEVELOPMENT_FEATURES

#ifdef DEVELOPMENT_FEATURES
#define DEVELOPMENT_ONLY
#else
#define DEVELOPMENT_ONLY const
#endif

static DEVELOPMENT_ONLY int sweep_trails = TRAIL_STACK - 1;
static DEVELOPMENT_ONLY int debug_panel = 0;
static DEVELOPMENT_ONLY uint32_t stop_axis = 1;

#ifdef HORIZONTAL_PRESLICE

#ifdef STINT_ON_BUFFER
// the 4 way symmetry of the slice map means we can use a 1/4 size slice buffer and still not need to clear unmapped columns
#define SLICE_BUFFER_SLICES SLICE_QUADRANT
#else
#define SLICE_BUFFER_SLICES SLICE_COUNT
#endif

static pixel_t slice_buffer[SLICE_BUFFER_SLICES][PANEL_FIELD_HEIGHT][PANEL_COUNT][PANEL_MULTIPLEX][PANEL_WIDTH] = {};
#define SLICE_BUFFER_WRAP(slice) ((slice) % (count_of(slice_buffer)))

static DEVELOPMENT_ONLY uint non_uniformity = (uint)SLICE_BRIGHTNESS_BOOSTED;

static void reset_slicemap() {
    slicemap_init((slice_brightness_t)non_uniformity);
    memset((void*)slice_buffer, 0, sizeof(slice_buffer));
}

#else

static DEVELOPMENT_ONLY uint non_uniformity = 0;
static void reset_slicemap() {}
#endif

static int volume_fd;
static voxel_double_buffer_t* volume_buffer;

static void* map_volume() {
    mode_t old_umask = umask(0);
    volume_fd = shm_open("/vortex_double_buffer", O_CREAT | O_RDWR, 0666);
    umask(old_umask);
    if (volume_fd == -1) {
        perror("shm_open");
        return NULL;
    }

    if (ftruncate(volume_fd, sizeof(voxel_double_buffer_t)) == -1) {
        perror("ftruncate");
        return NULL;
    }

    volume_buffer = mmap(NULL, sizeof(voxel_double_buffer_t), PROT_READ | PROT_WRITE, MAP_SHARED, volume_fd, 0);
    if (volume_buffer == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }

    return volume_buffer;
}

static void unmap_volume() {
    munmap(volume_buffer, sizeof(voxel_double_buffer_t));
    close(volume_fd);
}

static void clock_control() {
    static bool performance = false;

    bool need_performance = !rotation_stopped;

    if (performance != need_performance) {
        if (need_performance) {
            system("cpufreq-set -g performance");
        } else {
            system("cpufreq-set -g powersave");
        }

        performance = need_performance;
    }
}

static inline void tiny_wait(uint count) {
    for (uint i = 0; i < count; ++i) {
        gpio_set_bits(0);
    }
}

static inline void reset_panels(void) {

#ifdef ADDR_CLK
    // flush the address register
    gpio_clear_bits((1<<ADDR__EN) | (1<<ADDR_DAT));
    tiny_wait(10);
    for (int i = 0; i < PANEL_FIELD_HEIGHT; ++i) {
        gpio_set_pin(ADDR_CLK);
        tiny_wait(10);
        gpio_clear_pin(ADDR_CLK);
        tiny_wait(10);
    }
#endif

}

#ifdef DEVELOPMENT_FEATURES

static bool handle_keys() {
    int ch = getchar();

    switch (ch) {
        case 27:
            return false;

        case 'b': {
            volume_buffer->bits_per_channel = (volume_buffer->bits_per_channel % 3) + 1;
            printf("%d bpc\n", volume_buffer->bits_per_channel);
        } break;

        case 'u': {
            non_uniformity = (non_uniformity + 1) % 3;
            printf("non uniformity: %s\n", (char*[]){"uniform", "overdriven", "unlimited"}[non_uniformity]);
            reset_slicemap();
            break;
        }

        case 'x':
        case 'y':
        case 'z': {
            stop_axis = ch - 'x';
        } break;

        case '0': {
            rotation_zero = (rotation_zero + ROTATION_FULL / 64) & ROTATION_MASK;
            printf("rotation zero %d\n", rotation_zero);
        } break;
        
        case 'l': {
            rotation_lock = !rotation_lock;
            printf(rotation_lock ? "rotation lock on\n" : "rotation lock off\n");
        } break;

        case 'd':
        case 'D': {
            rotation_drift += (ch == 'd' ? 1 : -1);
            printf("rotation drift %d\n", rotation_drift);
        } break;
        
        case 't': {
            sweep_trails = (sweep_trails + 1) % TRAIL_STACK;
            printf("trails: %d\n", sweep_trails);
        } break;

        case 'p': {
            debug_panel = (debug_panel + 1) % 3;
            printf("debug panel %d\n", debug_panel);
        } break;

#ifdef DEVELOPMENT_CALIBRATE
        case '[':
        case ']':
        case '{':
        case '}': {
            printf("%g\n", eccentricity[ch>='{'] += (ch&2) ? 0.125f : -0.125f);
            reset_slicemap();
        } break;
#endif
    }

    return true;
}

#endif


typedef const pixel_t* scanline_stack_t[TRAIL_STACK][PANEL_COUNT][PANEL_MULTIPLEX];

#ifdef HORIZONTAL_PRESLICE
// Preprocess the voxel grid into display slices.
// With vertically arranged panels this isn't necessary - each scanline is a
// column and each column fits in one or two cache lines, so we can scan out
// directly from the volume.
// With horizontal panels we need to read an entire slice column by column
// and convert it to row order just ahead of the sweep.
static bool slicer_running = true;
static slice_index_t slice_angle = 0;

void* slicer_worker(void *vargp) {
    cpu_set_t cpu_mask;
    CPU_ZERO(&cpu_mask);
    for (int i = 0; i < 3; ++i) {
        CPU_SET(i, &cpu_mask);
    }
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpu_mask) != 0) {
        perror("sched_setaffinity");
    }

    int fd = shm_open("/vortex_double_buffer", O_RDWR, 0666);
    if (fd == -1) {
        perror("shm_open");
        slicer_running = false;
        return NULL;
    }

    voxel_double_buffer_t* volume_buffer = mmap(NULL, sizeof(*volume_buffer), PROT_WRITE, MAP_SHARED, fd, 0);
    if (volume_buffer == MAP_FAILED) {
        perror("mmap");
        slicer_running = false;
        return NULL;
    }

    // give ourselves room to finish the slice before it starts scanning out
    const slice_index_t slice_ahead = SLICE_QUADRANT / 2;
    slice_index_t sliceidx = 0;
    slice_index_t bufferidx = 0;

    while (slicer_running) {
        if (sliceidx == SLICE_WRAP(slice_angle + slice_ahead)) {
            usleep(100);
        } else {
#ifdef SLICER_PROFILE
            uint32_t work_start = *timer_uS;
#endif
            sliceidx = SLICE_WRAP(sliceidx + 1);
            bufferidx = SLICE_BUFFER_WRAP(sliceidx);

            voxel_2D_t* v2d;
            for (int c = 0; c < PANEL_WIDTH; ++c) {
                pixel_t* content = volume_buffer->volume[volume_buffer->page != 0];
                for (int p = 0; p < PANEL_COUNT; ++p) {
                    if (v2d = &slice_map[sliceidx][c][p], v2d->x < VOXELS_X) {
                        for (int r = 0; r < PANEL_FIELD_HEIGHT; ++r) {
                            slice_buffer[bufferidx][r][p][0][c] = content[VOXEL_INDEX(v2d->x, v2d->y, (VOXELS_Z-1) - r)];
                            slice_buffer[bufferidx][r][p][1][c] = content[VOXEL_INDEX(v2d->x, v2d->y, (VOXELS_Z-1) - r - PANEL_FIELD_HEIGHT)];
                        }
                    }
                }
            }

#ifdef SLICER_PROFILE
            uint32_t elapsed = *timer_uS - work_start;
            printf("%d uS\n", elapsed);
#endif
            usleep(1);
        }

    }

    return NULL;
}

static scanline_stack_t* horizontal_slice(uint line) {
    static slice_index_t last_scanned[PANEL_FIELD_HEIGHT] = {0};
    static const pixel_t* stack[TRAIL_STACK][PANEL_COUNT][PANEL_MULTIPLEX] = {0};

#ifdef SLICE_PRECISION
    slice_angle = SLICE_WRAP(rotation_current_angle() >> (ROTATION_PRECISION - SLICE_PRECISION));
#else
    slice_angle = SLICE_WRAP(((rotation_current_angle() >> (ROTATION_PRECISION - 10)) * SLICE_COUNT) >> 10);
#endif

    if (!rotation_stopped) {
        // if we've rotated by more than one slice since the last update, sweep trails will accumulate all the voxels
        // that we skipped over.

        int skipped = clamp(SLICE_BUFFER_WRAP(slice_angle - last_scanned[line] + SLICE_COUNT) - 1, 0, sweep_trails);
        for (int s = 0; s < TRAIL_STACK; ++s) {
            for (int p = 0; p < PANEL_COUNT; ++p) {
                for (int f = 0; f < PANEL_MULTIPLEX; ++f) {
                    stack[s][p][f] = slice_buffer[SLICE_BUFFER_WRAP(slice_angle + SLICE_COUNT - min(s, skipped))][line][p][f];
                }
            }
        }
        
        last_scanned[line] = slice_angle;

    } else {
        // when the display isn't spinning, present an orthographic view
        static uint32_t stop_seq = 0;
        static pixel_t stopped_row[PANEL_COUNT][PANEL_MULTIPLEX][PANEL_WIDTH];

        const uint voxels_mask[] = {VOXELS_X-1, VOXELS_Y-1, VOXELS_Z-1};
        stop_seq = (stop_seq + (line == 0) + 13) & voxels_mask[stop_axis];

        pixel_t* content = volume_buffer->volume[volume_buffer->page != 0];

        for (int c = 0; c < PANEL_WIDTH; ++c) {
            for (int p = 0; p < 2; ++p) {
                for (int f = 0; f < 2; ++f) {
                    switch (stop_axis) {
                        case 0:
                            stopped_row[p][f][c] = content[VOXEL_INDEX(stop_seq, c, (VOXELS_Z-1) - line - f * PANEL_FIELD_HEIGHT)];
                            break;
                        case 1:
                            stopped_row[p][f][c] = content[VOXEL_INDEX(c, stop_seq, (VOXELS_Z-1) - line - f * PANEL_FIELD_HEIGHT)];
                            break;
                        case 2:
                            stopped_row[p][f][c] = content[VOXEL_INDEX(c, PANEL_HEIGHT*3/2 - line - f * PANEL_FIELD_HEIGHT, stop_seq)];
                            break;
                    }
                }
            }
        }
        for (int s = 0; s < TRAIL_STACK; ++s) {
            for (int p = 0; p < PANEL_COUNT; ++p) {
                for (int f = 0; f < PANEL_MULTIPLEX; ++f) {
                    stack[s][p][f] = stopped_row[p][f];
                }
            }
        }
    }

    return &stack;
}
#endif

#ifdef VERTICAL_SCAN

#define ANGLE_PRECISION 10
#define SINCOS_PRECISION 12

static int16_t sincosfixed[1<<ANGLE_PRECISION][2];

static void init_angles(void) {
    for (uint i = 0; i < count_of(sincosfixed); ++i) {
        double a = ((double)i * M_PI * 2.0) / (double)(count_of(sincosfixed) - 1);
        double s = sin(a);
        double c = cos(a);
        sincosfixed[i][0] = (int)round(c * (double)(1<<SINCOS_PRECISION));
        sincosfixed[i][1] = (int)round(s * (double)(1<<SINCOS_PRECISION));
    }
}

static scanline_stack_t* vertical_slice(uint *line) {
    _Static_assert(TRAIL_STACK==1, "sweep trails haven't been implemented for vertical slicing");
    static const pixel_t* stack[TRAIL_STACK][PANEL_COUNT][PANEL_MULTIPLEX] = {0};
    static const pixel_t blank[PANEL_WIDTH] = {0};

    int angle = (rotation_current_angle() >> (ROTATION_PRECISION - ANGLE_PRECISION)) & (count_of(sincosfixed) - 1);
    
    uint column;
    bool inner;

    if (rotation_stopped || non_uniformity > 1) {
        column = *line & (PANEL_FIELD_HEIGHT-1);
        inner = true;
    } else {
        column = colscatter[*line];
        inner = (non_uniformity > 0) || (column < PANEL_FIELD_HEIGHT);
        column &= (PANEL_FIELD_HEIGHT-1);
    }

    pixel_t* content = volume_buffer->volume[volume_buffer->page != 0];

    if (!rotation_stopped) {
        int xf = sincosfixed[angle][0];
        int yf = sincosfixed[angle][1];

        for (int f = 0; f < PANEL_MULTIPLEX; ++f) {
            int r = ((1 - f) * PANEL_HEIGHT) + 1 + (column * 2);
            int x = ((PANEL_WIDTH << SINCOS_PRECISION) + xf * r) >> (SINCOS_PRECISION+1);
            int y = ((PANEL_WIDTH << SINCOS_PRECISION) + yf * r) >> (SINCOS_PRECISION+1);
            stack[0][1][f] = &content[VOXEL_INDEX(x, y, 0)];
            stack[0][0][f] = &content[VOXEL_INDEX((VOXELS_X-1) - x, (VOXELS_Y-1) - y, 0)];
        }

        if (!inner) {
            stack[0][1][1] = blank;
            stack[0][0][1] = blank;
        }
    } else {
        // orthographic view when stopped
        static uint32_t stop_seq = 0;

        stop_seq = (stop_seq + (column == 0) + 13) & (VOXELS_Y-1);
        
        for (int p = 0; p < PANEL_COUNT; ++p) {
            for (int f = 0; f < PANEL_MULTIPLEX; ++f) {
                int x = column + (1 - f) * PANEL_FIELD_HEIGHT;
                x = (p == 0) ? PANEL_HEIGHT + x : PANEL_HEIGHT - 1 - x;
                if (stop_axis == 0) {
                    stack[0][p][f] = &content[VOXEL_INDEX(x, stop_seq, 0)];
                } else {
                    stack[0][p][f] = &content[VOXEL_INDEX(stop_seq, x, 0)];
                }
            }
        }
    }

    *line = column;
    return &stack;
}


#else
    
static void init_angles(void) {}

#endif

#ifndef ADDR_CLK

// classic HUB75E direct row address
static void set_matrix_row(uint row) {

    uint a = PANEL_FIELD_HEIGHT - 1 - row;
    uint32_t rowbits = ((a & 0x01) << ROW_A)
                     | ((a & 0x02) << (ROW_B-1))
                     | ((a & 0x04) << (ROW_C-2))
                     | ((a & 0x08) << (ROW_D-3))
                     | ((a & 0x10) << (ROW_E-4));
    gpio_clear_bits(((~rowbits) & ROW_MASK));
    gpio_set_bits(rowbits & ROW_MASK);
    gpio_clear_bits(RGB_STROBE_MASK);
}

#else

// shift register address
static uint current_shift_row = 1;

static inline void address_shift_left(const uint lsb) {
    tiny_wait(CLOCK_WAITS * 2);
    gpio_set_bits((1<<ADDR_CLK) | (lsb<<ADDR_DAT));
    tiny_wait(CLOCK_WAITS * 2);
    gpio_clear_bits((1<<ADDR_CLK) | (1<<ADDR_DAT));
}

static void set_matrix_row(uint row) {
    if (row < current_shift_row) {
        int leading_zeros = PANEL_FIELD_HEIGHT - row - 1;
        int flush = leading_zeros - current_shift_row;

        for (int i = 0; i < flush; ++i) {
            address_shift_left(0);
        }
        address_shift_left(1);

        current_shift_row = 0;
    }

    for (; current_shift_row < row; ++current_shift_row) {
        address_shift_left(0);
    }

}

#endif


static bool killed = false;
void sig_handler(int sig) {
    killed = true;
}

int main(int argc, char** argv) {
    cpu_set_t cpu_mask;
    CPU_ZERO(&cpu_mask);
    int target_core = 3;
    CPU_SET(target_core, &cpu_mask);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpu_mask) != 0) {
        perror("sched_setaffinity");
        return 1;
    }

    if (!gpio_init()) {
        return -1;
    }

    voxel_double_buffer_t* buffer = map_volume();
    if (!buffer) {
        fprintf(stderr, "Can't open buffer\n");
        return -1;
    }

    buffer->bits_per_channel = 2;

    rotation_init();
    reset_panels();
    init_angles();

#ifdef HORIZONTAL_PRESLICE
    reset_slicemap();
    
    pthread_t slicer_thread;
    pthread_create(&slicer_thread, NULL, slicer_worker, NULL); 
#endif

    signal(SIGINT,  sig_handler);
    signal(SIGKILL, sig_handler);
    signal(SIGTERM, sig_handler);

#ifdef DEVELOPMENT_FEATURES
    bool interactive = isatty(fileno(stdout));

    if (interactive) {
        input_set_nonblocking();
    }

    uint32_t perf_period = 0;
    uint perf_count = 0;
#endif

    while (!killed) {
#ifdef DEVELOPMENT_FEATURES
        uint32_t frame_start = *timer_uS;

        if (interactive) {
            if (!handle_keys()) {
                break;
            }
        }
#endif
        clock_control();

        // we unblank the previous row while we're shifting in the new row. This lookup defines
        // how late that unblank happens to vary the brightness for BCM
        const int bpc = min(max(1, buffer->bits_per_channel), BPC_MAX);
        const int gamma[BPC_MAX] = {PANEL_WIDTH-120, PANEL_WIDTH-60, PANEL_WIDTH-30};
        int unblank[3] = {};
        for (int b = 0; b < bpc; ++b) {
            unblank[(b+1)%bpc] = gamma[b];
        }

#ifdef VERTICAL_SCAN
        for (uint ci = 0; ci < count_of(colscatter); ci++) {
            uint line = ci;
            scanline_stack_t *stack = vertical_slice(&line);
#else
        for (uint line = 0; line < PANEL_FIELD_HEIGHT; ++line) {
            scanline_stack_t *stack = horizontal_slice(line);
#endif

            // read the colour data from the slice buffer and clock it out to the displays
            #pragma GCC unroll 3
            for (int b = 0; b < bpc; ++b) {

                for (int c = 0; c < PANEL_WIDTH; ++c) {
                    int s;
                    pixel_t pix;
                    uint32_t rgbbits = 0;

                    if (debug_panel != 2) {
                        for (pix = 0, s = 0; s < TRAIL_STACK; ++s) {
                            pix |= (*stack)[s][0][0][PANEL_0_ORDER(c)];
                        }
                        rgbbits |= (R_MTH_BIT(pix, b) << RGB_0_R1);
                        rgbbits |= (G_MTH_BIT(pix, b) << RGB_0_G1);
                        rgbbits |= (B_MTH_BIT(pix, b) << RGB_0_B1);

                        for (pix = 0, s = 0; s < TRAIL_STACK; ++s) {
                            pix |= (*stack)[s][0][1][PANEL_0_ORDER(c)];
                        }
                        rgbbits |= (R_MTH_BIT(pix, b) << RGB_0_R2);
                        rgbbits |= (G_MTH_BIT(pix, b) << RGB_0_G2);
                        rgbbits |= (B_MTH_BIT(pix, b) << RGB_0_B2);
                    }
                    if (debug_panel != 1) {
                        for (pix = 0, s = 0; s < TRAIL_STACK; ++s) {
                            pix |= (*stack)[s][1][0][PANEL_1_ORDER(c)];
                        }
                        rgbbits |= (R_MTH_BIT(pix, b) << RGB_1_R1);
                        rgbbits |= (G_MTH_BIT(pix, b) << RGB_1_G1);
                        rgbbits |= (B_MTH_BIT(pix, b) << RGB_1_B1);

                        for (pix = 0, s = 0; s < TRAIL_STACK; ++s) {
                            pix |= (*stack)[s][1][1][PANEL_1_ORDER(c)];
                        }
                        rgbbits |= (R_MTH_BIT(pix, b) << RGB_1_R2);
                        rgbbits |= (G_MTH_BIT(pix, b) << RGB_1_G2);
                        rgbbits |= (B_MTH_BIT(pix, b) << RGB_1_B2);
                    }

                    gpio_clear_bits(((~rgbbits) & RGB_BITS_MASK) | RGB_CLOCK_MASK | ((c==unblank[b]) << RGB_BLANK));
                    gpio_set_bits(rgbbits & RGB_BITS_MASK);
                    
                    tiny_wait((CLOCK_WAITS  )/2);

                    gpio_set_pin(RGB_CLOCK);
                    
                    tiny_wait((CLOCK_WAITS+1)/2);
                }

                gpio_clear_bits(RGB_BITS_MASK | RGB_CLOCK_MASK);
                gpio_set_bits(RGB_BLANK_MASK | RGB_STROBE_MASK | ADDR__EN_MASK);

                if (b == 0) {
                    set_matrix_row(line);
                } else {
                    tiny_wait(4);
                }

                gpio_clear_bits(RGB_STROBE_MASK | ADDR__EN_MASK);
            }

#ifdef DEVELOPMENT_FEATURES
            if (line == 31) {
                ++perf_count;
            }
#endif
        }

        buffer->revolutions_per_minute = rotation_stopped ? 0 : 60000000 / rotation_period;
        
#ifdef DEVELOPMENT_FEATURES
        if (interactive) {
            perf_period += *timer_uS - frame_start;
            if (perf_count >= 4096) {
                buffer->microseconds_per_frame = perf_period / perf_count;
                uint revs = 100000000 / rotation_period;
                printf("%u uS/frame   %u frame/S      %u.%02u revs/S   %u rpm\n", perf_period / perf_count, perf_count*1000000 / perf_period, revs/100, revs%100, (revs * 6) / 10);
                perf_count = 0;
                perf_period = 0;
            }
        }
#endif
    }

    gpio_set_bits(RGB_BLANK_MASK | ADDR__EN_MASK);

#ifdef HORIZONTAL_PRESLICE
    slicer_running = false;
    pthread_join(slicer_thread, NULL);
#endif

    unmap_volume();

    return 0;
}


