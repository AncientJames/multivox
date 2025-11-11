#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <sched.h>
#include <time.h>
#include <linux/joystick.h>
#include <poll.h>
#include <pthread.h>
#include <math.h>
#include <sys/wait.h>
#include <glob.h>

#include "mathc.h"
#include "rammel.h"
#include "input.h"
#include "graphics.h"
#include "model.h"
#include "voxel.h"

#define SHOW_STATS 1

static float model_rotation[VEC3_SIZE] = {0, 0, 0};
static float model_position[VEC3_SIZE] = {0, 0, 0};
static float model_scale = 1.0f;

static model_style_t wireframe = STYLE_WIREFRAME_IF_UNDEFINED;

typedef enum {
    NAVIGATION_ORBIT,
    NAVIGATION_WALKTHROUGH,
    NAVIGATION_COUNT
} navigation_t;

static navigation_t navigation_style = NAVIGATION_ORBIT;

static void home_pose() {
    model_scale = 1.0f;
    vec3_zero(model_rotation);
    vec3_zero(model_position);
}

static int scene_count = 0;
static char** scene_list = NULL;
static int scene_current = 0;

static model_t fallback_model = {
    .vertex_count = 12,
    .vertices = (vertex_t*)(float[][5]){
        {-8.29155, -25.5192, -13.4164, 0, 0}, {0, -0, -30, 0, 0}, {21.708, -15.7716, -13.4164, 0, 0}, {21.708, 15.7716, -13.4164, 0, 0}, {-26.8328, -0, -13.4164, 0, 0}, {-8.29155, 25.5192, -13.4164, 0, 0},
        {26.8328, -0, 13.4164, 0, 0}, {8.29155, -25.5192, 13.4164, 0, 0}, {-21.708, -15.7716, 13.4164, 0, 0}, {-21.708, 15.7716, 13.4164, 0, 0}, {8.29155, 25.5192, 13.4164, 0, 0}, {0, -0, 30, 0, 0}, 
    },
    .edge_count = 30,
    .edges = (edge_t[]){
        {{ 0, 1}, HEXPIX(AAFFAA)}, {{ 1, 2}, HEXPIX(00FF00)}, {{ 0, 2}, HEXPIX(FF55AA)}, {{ 2, 3}, HEXPIX(55AA55)}, {{ 1, 3}, HEXPIX(AAFF55)},
        {{ 1, 4}, HEXPIX(55AAFF)}, {{ 0, 4}, HEXPIX(AAAA55)}, {{ 1, 5}, HEXPIX(FFFF00)}, {{ 4, 5}, HEXPIX(FF55FF)}, {{ 3, 5}, HEXPIX(555555)},
        {{ 2, 6}, HEXPIX(55FFFF)}, {{ 3, 6}, HEXPIX(AAFFFF)}, {{ 0, 7}, HEXPIX(FFAAFF)}, {{ 2, 7}, HEXPIX(FF0000)}, {{ 4, 8}, HEXPIX(55AAAA)},
        {{ 0, 8}, HEXPIX(5555AA)}, {{ 5, 9}, HEXPIX(AA55AA)}, {{ 4, 9}, HEXPIX(FF5555)}, {{ 3,10}, HEXPIX(5555FF)}, {{ 5,10}, HEXPIX(FFAA55)},
        {{ 6, 7}, HEXPIX(FFFFAA)}, {{ 7, 8}, HEXPIX(FFAAAA)}, {{ 8, 9}, HEXPIX(AAAAFF)}, {{ 9,10}, HEXPIX(AA55FF)}, {{ 6,10}, HEXPIX(55FFAA)},
        {{ 7,11}, HEXPIX(FFFF55)}, {{ 6,11}, HEXPIX(0000FF)}, {{ 8,11}, HEXPIX(AAAAAA)}, {{ 9,11}, HEXPIX(55FF55)}, {{10,11}, HEXPIX(AA5555)}, 
    }
};

static model_t* scene_model = &fallback_model;


static bool is_model(const char* filename) {
    const char* ext = strrchr(filename, '.');
    if (ext) {
        if (strcasecmp(ext, ".obj") == 0) {
            return true;
        }
    }

    return false;
}

static bool is_image(const char* filename) {
    const char* ext = strrchr(filename, '.');
    if (ext) {
        if (strcasecmp(ext, ".png") == 0)   return true;
        if (strcasecmp(ext, ".tga") == 0)   return true;
        if (strcasecmp(ext, ".jpg") == 0)   return true;
        if (strcasecmp(ext, ".jpeg") == 0)  return true;
    }

    return false;
}

static void zoom_to_fit() {
    home_pose();
    if (!scene_model) {
        return;
    }

    vec3_t centre;
    float radius;
    float height;
    model_get_bounds(scene_model, &centre, &radius, &height);

    float hscale = (float)(VOXELS_X-1) / (radius * 2.0f);
    float vscale = (float)(VOXELS_Z-1) / height;
    
    model_scale = min(hscale, vscale);

    model_position[0] = -centre.x * model_scale;
    model_position[1] = -centre.y * model_scale;
    model_position[2] = -centre.z * model_scale;

    printf("zoom %g, offset %g,%g,%g\n", model_scale, model_position[0], model_position[1], model_position[2]);
}

static model_t* load_scene(char* scene) {
    model_t* model = NULL;

    printf("loading %s\n", scene);

    if (is_model(scene)) {
        model = model_load(scene, wireframe);
    } else if (is_image(scene)) {
        model = model_load_image(scene);
    } else {
        //assume it's raw voxel data
        pixel_t* volume = voxel_buffer_get(VOXEL_BUFFER_BACK);

        bool read = false;
        FILE* fd = fopen(scene, "rb");
        if (fd) {
            read = (fread(volume, 1, sizeof(*voxel_buffer->volume), fd) == sizeof(*voxel_buffer->volume));
            fclose(fd);
        }

        if (!read) {
            voxel_buffer_clear(volume);
        }
        voxel_buffer_swap();
        memcpy(voxel_buffer_get(VOXEL_BUFFER_BACK), voxel_buffer_get(VOXEL_BUFFER_FRONT), sizeof(*voxel_buffer->volume));
    }

    return model;
}


static void vox_rotate_z(float angle) {
    pixel_t* src = voxel_buffer_get(VOXEL_BUFFER_BACK);
    pixel_t* dst = voxel_buffer_get(VOXEL_BUFFER_FRONT);

    bool flip = false;
    angle = fmodf(fmodf(angle, M_PI*2) + M_PI_2*5, M_PI*2) - M_PI_2;
    if (angle > M_PI_2) {
        angle -= M_PI;
        flip = true;
    }
    
    float shear_x = -tanf(angle * 0.5f);
    float shear_y = sinf(angle);

    for (int y = 0; y < VOXELS_Y; ++y) {
        for (int x = 0; x < VOXELS_X; ++x) {
            int xs1 = x + lround(((float)y - (float)(VOXELS_Y-1) * 0.5f) * shear_x);
            int ys = y + lround(((float)xs1 - (float)(VOXELS_X-1) * 0.5f) * shear_y);
            int xs = xs1 + lround(((float)ys - (float)(VOXELS_Y-1) * 0.5f) * shear_x);
            if (xs >= 0 && xs < VOXELS_X && ys >= 0 && ys < VOXELS_Y) {
                if (flip) {
                    xs = VOXELS_X - 1 - xs;
                    ys = VOXELS_Y - 1 - ys;
                }
                for (int z = 0; z < VOXELS_Z; ++z) {
                    dst[VOXEL_INDEX(x, y, z)] = src[VOXEL_INDEX(xs, ys, z)];
                }
            } else {
                for (int z = 0; z < VOXELS_Z; ++z) {
                    dst[VOXEL_INDEX(x, y, z)] = 0;
                }
            }
        }
    }
}

static int temperature_base = 0;
static int temperature_cpu = 0;
static bool monitor_temperature = true;

static void* temperature_worker(void *vargp) {
    while (monitor_temperature) {
        char buffer[256];
        FILE* fp;
        int millicelsius;

        millicelsius = 0;
        if ((fp = fopen("/sys/bus/w1/devices/28-8406ee086461/w1_slave", "r"))) {
            while (fgets(buffer, sizeof(buffer), fp) && monitor_temperature) {
                char* temp;
                if ((temp = strstr(buffer, "t="))) {
                    millicelsius = strtol(temp+2, NULL, 10);
                    break;
                }
            }
            fclose(fp);
        }
        temperature_base = millicelsius;

        millicelsius = 0;
        if ((fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r"))) {
            int read = fread(buffer, 1, sizeof(buffer)-1, fp);
            if (read > 0) {
                buffer[read] = 0;
            }
            millicelsius = strtol(buffer, NULL, 10);
            fclose(fp);
        }
        temperature_cpu = millicelsius;

        for (int i = 0; i < 100 && monitor_temperature; ++i) {
            usleep(100000);
        }
    }

    return NULL;
}

static void parse_arguments(int argc, char** argv) {
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));
    int file_count = 0;
    char **file_list = NULL;
    
    for (int i = 1; i < argc; ++i) {
        int ret = glob(argv[i], GLOB_TILDE, NULL, &glob_result);
        if (ret != 0) {
            printf("glob failed: %d\n", ret);
            globfree(&glob_result);
            continue;
        }
        
        file_list = realloc(file_list, (file_count + glob_result.gl_pathc) * sizeof(char*));
        if (!file_list) {
            printf("realloc failed\n");
            globfree(&glob_result);
            return;
        }
        
        for (size_t j = 0; j < glob_result.gl_pathc; ++j) {
            file_list[file_count + j] = strdup(glob_result.gl_pathv[j]);
        }
        
        file_count += glob_result.gl_pathc;
        globfree(&glob_result);
    }
    
    if (file_list) {
        scene_count = file_count;
        scene_list = file_list;
    }
}

int main(int argc, char** argv) {

    parse_arguments(argc, argv);

    if (!voxel_buffer_map()) {
        printf("waiting for voxel buffer\n");
        do {
            sleep(1);
        } while (!voxel_buffer_map());
    }

    scene_current = 0;
    if (scene_count > 0) {
        scene_model = load_scene(scene_list[scene_current]);
    }
    home_pose();

    mfloat_t centre[VEC3_SIZE] = {(VOXELS_X-1)*0.5f, (VOXELS_Y-1)*0.5f, (VOXELS_Z-1)*0.5f};

    float dscale = 0.0f;
    float deuler[VEC3_SIZE] = {0, 0, 0};
    float doffset[VEC3_SIZE] = {0, 0, 0};
    float matrix[MAT4_SIZE];
#ifdef SHOW_STATS
    int perf = 0;

    monitor_temperature = true;
    pthread_t temperature_thread;
    pthread_create(&temperature_thread, NULL, temperature_worker, NULL); 
#endif

    home_pose();
    input_set_nonblocking();

    int scene_target = scene_current;

    for (int ch = 0; ch != 27; ch = getchar()) {
        bool scene_reload = false;

        switch (ch) {
            case '[': {
                scene_target += 1;
            } break;
            case ']': {
                scene_target -= 1;
            } break;

            case 'h': {
                zoom_to_fit();
            } break;
            
            case 'x': {
                model_rotation[0] = 0;
                model_rotation[1] = 0;
                model_rotation[2] = M_PI_2;
            } break;
            case 'y': {
                model_rotation[0] = 0;
                model_rotation[1] = 0;
                model_rotation[2] = 0;
            } break;
            case 'z': {
                model_rotation[0] = -M_PI_2;
                model_rotation[1] = 0;
                model_rotation[2] = 0;
            } break;

            case 'i':
            case 'I':
            case 'j':
            case 'J':
            case 'k':
            case 'K': {
                if (ch >= 'a') {
                    deuler[ch-'i'] += 0.001f;
                } else {
                    deuler[ch-'I'] -= 0.001f;
                }
            } break;
            
            case 'f':
            case 'F': {
                if (ch >= 'a') {
                    doffset[1] += 0.001f;
                } else {
                    doffset[1] -= 0.001f;
                }
            } break;

            case 'd':
                if (scene_model) {
                    model_dump(scene_model);
                }
                break;
        }

        input_update();

        if (input_get_button(0, BUTTON_A, BUTTON_PRESSED)) {
            navigation_style = (navigation_style + 1) % NAVIGATION_COUNT;
            home_pose();
        }

        if (input_get_button(0, BUTTON_X, BUTTON_PRESSED)) {
            zoom_to_fit();
        }

        if (input_get_button(0, BUTTON_Y, BUTTON_PRESSED)) {
            wireframe = (wireframe + 1) % STYLE_COUNT;
            scene_reload = true;
        }

        if (input_get_button(0, BUTTON_LB, BUTTON_PRESSED)) {
            scene_target -= 1;
        }

        if (input_get_button(0, BUTTON_RB, BUTTON_PRESSED)) {
            scene_target += 1;
        }

        if (scene_count > 0) {
            scene_target = modulo(scene_target, scene_count);
            if (scene_current != scene_target || scene_reload) {
                scene_current = scene_target;

                model_free(scene_model);
                scene_model = load_scene(scene_list[scene_current]);
                home_pose();
            }
        }

        switch (navigation_style) {
            case NAVIGATION_ORBIT: {
                deuler[2] = input_get_axis(0, AXIS_LS_X) * -0.1f;
                deuler[0] = input_get_axis(0, AXIS_LS_Y) *  0.1f;
                deuler[1] = input_get_axis(0, AXIS_RS_X) *  0.1f;
                    
                dscale = (input_get_axis(0, AXIS_LT) - input_get_axis(0, AXIS_RT)) * 0.1f;

                doffset[0] = input_get_axis(0, AXIS_D_X) *  3.0f;
                doffset[1] = input_get_axis(0, AXIS_D_Y) * -3.0f;
                doffset[2] = input_get_axis(0, AXIS_RS_Y) * -1.0f;
            } break;

            default:
            case NAVIGATION_WALKTHROUGH: {
                deuler[2] = input_get_axis(0, AXIS_RS_X) * 0.1f;

                doffset[1] = input_get_axis(0, AXIS_LS_Y) *  5.0f / model_scale;
                doffset[0] = input_get_axis(0, AXIS_LS_X) * -5.0f / model_scale;
                doffset[2] = input_get_axis(0, AXIS_RS_Y) *  4.0f / model_scale;
                    
                dscale = (input_get_axis(0, AXIS_LT) - input_get_axis(0, AXIS_RT)) * 0.1f;
            } break;
        }



        mat4_identity(matrix);

        switch (navigation_style) {
            case NAVIGATION_ORBIT: {
                model_scale *= 1.0f + dscale;
                vec3_add(model_rotation, model_rotation, deuler);
                vec3_add(model_position, model_position, doffset);

                mat4_apply_translation(matrix, centre);
                mat4_apply_translation(matrix, model_position);
                mat4_apply_scale_f(matrix, model_scale);
                mat4_apply_rotation(matrix, model_rotation);
            } break;

            case NAVIGATION_WALKTHROUGH: {
                model_scale *= 1.0f + dscale;
                vec3_add(model_rotation, model_rotation, deuler);

                float direction[VEC3_SIZE];
                vec2_rotate(direction, doffset, -model_rotation[2]);
                direction[2] = doffset[2];
                vec3_add(model_position, model_position, direction);

                mat4_apply_translation(matrix, centre);
                mat4_apply_scale_f(matrix, model_scale);
                mat4_apply_rotation(matrix, model_rotation);
                mat4_apply_translation(matrix, model_position);
            } break;

            default:
                break;
        }

        if (scene_model) {
            pixel_t* volume = voxel_buffer_get(VOXEL_BUFFER_BACK);
            voxel_buffer_clear(volume);

            model_draw(volume, scene_model, matrix);

            voxel_buffer_swap();
        } else {
            if (fabsf(model_rotation[2]) > 0.001f) {
                vox_rotate_z(model_rotation[2]);
            }
        }

#ifdef SHOW_STATS
        int tbase = temperature_base;
        int tproc = temperature_cpu;
        if (++perf > 60) {
            perf = 0;
            printf("%u fps   %u rpm    %d.%03d° (%d.%03d°)\n", 1000000 / max(1, (uint)voxel_buffer->microseconds_per_frame), (uint)voxel_buffer->revolutions_per_minute, tbase / 1000, tbase % 1000, tproc / 1000, tproc % 1000);
            //printf("x:%g y:%g z:%g s:%g p:%g r:%g y:%g\n", model_position[0], model_position[1], model_position[2], model_scale, model_rotation[0], model_rotation[1], model_rotation[2]);
        }
#endif

        usleep(50000);
    }
#ifdef VALGRIND_HAPPY
    model_free(scene_model);
#endif

#ifdef SHOW_STATS
    monitor_temperature = false;
    pthread_join(temperature_thread, NULL);
#endif

    voxel_buffer_unmap();

    return 0;
}




