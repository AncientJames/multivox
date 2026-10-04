#include "sim.h"

#include <stddef.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <math.h>

#include "mathc.h"
#include "voxel.h"
#include "rammel.h"
#include "slicemap.h"

#include "volume_vert_glsl.h"
#include "volume_frag_glsl.h"

_Static_assert(VOXEL_Z_STRIDE==1, "simulator assumes z stride is 1");



typedef struct {
    GLuint texture;
    GLuint vao;
    GLuint vbo;
    size_t vertex_count;

    GLuint program;
    GLuint u_view;
    GLuint u_proj;
    GLuint u_bpcmask;
    GLuint u_brightness;
} draw_state_t;

typedef struct {
    GLfloat position[3];
    GLfloat texcoord[3];
    GLfloat dotcoord[2];
} volume_vertex_t;

static draw_state_t volume;

static int volume_fd;
voxel_double_buffer_t* voxel_buffer;

static int viewport_width = 800;
static int viewport_height = 600;

static float mat_view[MAT4_SIZE];
static float mat_proj[MAT4_SIZE];

static vec3_t view_position = {.x=0, .y=0, .z=-3};
static vec3_t model_rotation = {.x=-M_PI_2, .y=0, .z=0};

static float sim_brightness = 1.0f;


typedef enum {
    SCAN_RADIAL,
    SCAN_LINEAR
} scan_geometry_t;

typedef struct {
    int screen_width;
    int screen_height;
    float screen_offset[2];
    size_t screen_count;
    size_t slice_count;
    scan_geometry_t scan_geometry;
} sim_geometry_t;

static sim_geometry_t sim_geometry = {

#ifdef PANEL_0_ECCENTRICITY
    #ifdef PANEL_1_ECCENTRICITY
        .screen_offset = {(float)(PANEL_0_ECCENTRICITY)/(float)(PANEL_WIDTH/2), (float)(PANEL_1_ECCENTRICITY)/(float)(PANEL_WIDTH/2)},
        .screen_count = ((PANEL_0_ECCENTRICITY) == (PANEL_1_ECCENTRICITY) ? 1 : 2),
    #else
        .screen_offset = {(float)(PANEL_0_ECCENTRICITY)/(float)(PANEL_WIDTH/2)},
        .screen_count = 1,
    #endif
#else
        .screen_offset = {0},
        .screen_count = 1,
#endif

#ifdef VERTICAL_SCAN
    .screen_width = PANEL_HEIGHT * 2,
    .screen_height = PANEL_WIDTH,
#else
    .screen_width = PANEL_WIDTH,
    .screen_height = PANEL_HEIGHT,
#endif

    .slice_count = 360,
    .scan_geometry = SCAN_RADIAL
};

int sim_bpc = 2;

static void parse_args(int argc, char** argv) {
    bool help = false;
    bool sset = false;
    bool wset = false;

    for (int opt = 0; opt != -1; opt = getopt(argc, argv, "o:s:w:g:b:")) {
        switch(opt) {
            case 'o': {
                sim_geometry.screen_offset[0] = sim_geometry.screen_offset[1] = atof(optarg);
                if (optind < argc) {
                    sim_geometry.screen_offset[1] = atof(argv[optind]);
                    if (sim_geometry.screen_offset[1] != 0) {
                        ++optind;
                    }
                }
                sim_geometry.screen_count = 1 + (sim_geometry.screen_offset[0] != sim_geometry.screen_offset[1]);
            } break;

            case 's': {
                sim_geometry.slice_count = clampi(atoi(optarg), 1, 4096);
                sset = true;
            } break;

            case 'w': {
                if (optind < argc) {
                    int w = atoi(optarg);
                    int h = atoi(argv[optind++]);
                    if (w > 0 && h > 0) {
                        wset = true;
                        sim_geometry.screen_width = w;
                        sim_geometry.screen_height = h;
                    }
                }
            } break;

            case 'g': {
                switch (*optarg) {
                    case 'l': sim_geometry.scan_geometry = SCAN_LINEAR; break;
                    case 'r': sim_geometry.scan_geometry = SCAN_RADIAL; break;
                }
            } break;

            case 'b': {
                sim_bpc = clampi(atoi(optarg), 1, 3);
            } break;

            case '?': {
                help = true;
            } break;
        }
    }

    if (help) {
        printf("%s - multivox volumetric display simulator.\n"
               " -b X     bit depth\n"
               " -s X     slice count\n"
               " -w X X   panel resolution (width, height)\n"
               " -o X X   panel offset (front, back)\n"
               " -g X     geometry (radial, linear)\n\n",
        argv[0]);
    }

    if (sim_geometry.scan_geometry == SCAN_LINEAR) {
        if (!sset) {
            sim_geometry.slice_count = VOXELS_Z;
        }
        if (!wset) {
            sim_geometry.screen_width = VOXELS_X;
            sim_geometry.screen_height = VOXELS_Y;
        }
    }

    if (sim_geometry.screen_count > 1
     && sim_geometry.scan_geometry == SCAN_RADIAL
     && sim_geometry.screen_offset[0] != sim_geometry.screen_offset[1]
     && sim_geometry.screen_offset[0] >= 0
     && sim_geometry.screen_offset[1] >= 0) {
        sim_geometry.screen_count = 2;
    } else {
        sim_geometry.screen_count = 1;
    }

    printf("      slice count: %ld\n panel resolution: %dx%d\n", sim_geometry.slice_count, sim_geometry.screen_width, sim_geometry.screen_height);
    if (sim_geometry.screen_count > 1) {
        printf("   screen offsets: %g %g\n", sim_geometry.screen_offset[0], sim_geometry.screen_offset[1]);
    } else {
        printf("    screen offset: %g\n", sim_geometry.screen_offset[0]);
    }
    switch (sim_geometry.scan_geometry) {
        case SCAN_RADIAL: printf("         geometry: radial\n"); break;
        case SCAN_LINEAR: printf("         geometry: linear\n"); break;
    }
    printf("     colour depth: %d bpc\n", sim_bpc);
}




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

    voxel_buffer = mmap(NULL, sizeof(voxel_double_buffer_t), PROT_READ | PROT_WRITE, MAP_SHARED, volume_fd, 0);
    if (voxel_buffer == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }

    return voxel_buffer;
}

static void unmap_volume() {
    munmap(voxel_buffer, sizeof(voxel_double_buffer_t));
    close(volume_fd);
}


GLuint compile_shader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    if (!shader) {
        return 0;
    }

#ifdef HIGH_COLOUR
    glShaderSource(shader, 2, (const GLchar*[]){"#version 310 es\n#define VOXEL_565\n//", source}, NULL);
#else
    glShaderSource(shader, 1, &source, NULL);
#endif
    glCompileShader(shader);

    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint log_length = 0;

        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
        if (log_length > 1) {
            char log[log_length];
            glGetShaderInfoLog(shader, log_length, NULL, log);
            printf("failed to compile shader:\n%s\n", log);
        }

        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

GLuint create_program(const char* vert, const char* frag) {
    GLuint vshader = compile_shader(GL_VERTEX_SHADER, vert);
    GLuint fshader = compile_shader(GL_FRAGMENT_SHADER, frag);

    if (!vshader || !fshader) {
        return 0;
    }

    GLuint program = glCreateProgram();
    if (!program) {
        return 0;
    }

    glAttachShader(program, vshader);
    glAttachShader(program, fshader);

    glLinkProgram(program);

    GLint linked;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint log_length = 0;

        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
        if (log_length > 1) {
            char log[log_length];
            glGetProgramInfoLog(program, log_length, NULL, log);
            printf("failed to link shader:\n%s\n", log);
        }

        glDeleteProgram(program);
        return 0;
    }

    return program;
}

static void init_texture(void) {
    glGenTextures(1, &volume.texture);
    glBindTexture(GL_TEXTURE_3D, volume.texture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    //glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    //glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
#ifdef HIGH_COLOUR
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB565, VOXELS_Z, VOXELS_X, VOXELS_Y, 0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, voxel_buffer->volume[voxel_buffer->page]);
#else
    glTexImage3D(GL_TEXTURE_3D, 0, GL_R8UI, VOXELS_Z, VOXELS_X, VOXELS_Y, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, voxel_buffer->volume[voxel_buffer->page]);
#endif
}

static size_t create_mesh_radial() {
    // create a mesh containing quads for every slice the screens rotate through

    volume_vertex_t vertices[sim_geometry.slice_count][sim_geometry.screen_count][12];

    int scatter[sim_geometry.slice_count];
    slicemap_ebr(scatter, count_of(scatter));

    const float z = (float)VOXELS_Z / (float)VOXELS_X;
    const float radius = (float)sim_geometry.screen_width * 0.5f;

    for (int s = 0; s < sim_geometry.slice_count; ++s) {
        float angle = (float)s * M_PI * 2.0f / (float)sim_geometry.slice_count;
        vec2_t slope = {.x = cosf(angle), .y = sinf(angle)};
        
        // throttle the inner columns
        float hole = (float)scatter[s] / (float)sim_geometry.slice_count;
        hole = floorf(hole * radius) / radius;

        for (int p = 0; p < sim_geometry.screen_count; ++p) {
            float side = (p ? -1 : 1);
            vec2_t offset = {.x = slope.y * sim_geometry.screen_offset[p] * side, .y = -slope.x * sim_geometry.screen_offset[p] * side};

            vec2_t inner, outer;
            vec2_multiply_f(outer.v, slope.v, side);
            vec2_multiply_f(inner.v, outer.v, hole);

            vec2_t ends[4];
            vec2_subtract(ends[0].v, offset.v, outer.v);
            vec2_subtract(ends[1].v, offset.v, inner.v);
            vec2_add(ends[2].v, offset.v, inner.v);
            vec2_add(ends[3].v, offset.v, outer.v);

            // clip to the voxel volume
            float rim[2] = {1.0f, 1.0f};
            for (int i = 0; i < 2; ++i) {
                float box = max(fabsf(ends[3*i].x), fabsf(ends[3*i].y));
                if (box > 1.0f) {
                    rim[i] = floorf((1.0f / box) * radius) / radius;
                    hole = min(hole, rim[i]);
                    vec2_multiply_f(ends[3*i].v, ends[3*i].v, rim[i]);
                }
            }

            vertices[s][p][0] = (volume_vertex_t){.position={ends[0].x, ends[0].y, -z}, .texcoord={0, (1+ends[0].x)*0.5f, (1+ends[0].y)*0.5f}, .dotcoord={radius*rim[0], 0.0f}};
            vertices[s][p][1] = (volume_vertex_t){.position={ends[1].x, ends[1].y, -z}, .texcoord={0, (1+ends[1].x)*0.5f, (1+ends[1].y)*0.5f}, .dotcoord={radius * hole, 0.0f}};
            vertices[s][p][2] = (volume_vertex_t){.position={ends[0].x, ends[0].y,  z}, .texcoord={1, (1+ends[0].x)*0.5f, (1+ends[0].y)*0.5f}, .dotcoord={radius*rim[0], (float)sim_geometry.screen_height}};
            vertices[s][p][3] = (volume_vertex_t){.position={ends[1].x, ends[1].y,  z}, .texcoord={1, (1+ends[1].x)*0.5f, (1+ends[1].y)*0.5f}, .dotcoord={radius * hole, (float)sim_geometry.screen_height}};
            vertices[s][p][4] = vertices[s][p][2];
            vertices[s][p][5] = vertices[s][p][1];

            vertices[s][p][6] = (volume_vertex_t){.position={ends[2].x, ends[2].y, -z}, .texcoord={0, (1+ends[2].x)*0.5f, (1+ends[2].y)*0.5f}, .dotcoord={radius * hole, 0.0f}};
            vertices[s][p][7] = (volume_vertex_t){.position={ends[3].x, ends[3].y, -z}, .texcoord={0, (1+ends[3].x)*0.5f, (1+ends[3].y)*0.5f}, .dotcoord={radius*rim[1], 0.0f}};
            vertices[s][p][8] = (volume_vertex_t){.position={ends[2].x, ends[2].y,  z}, .texcoord={1, (1+ends[2].x)*0.5f, (1+ends[2].y)*0.5f}, .dotcoord={radius * hole, (float)sim_geometry.screen_height}};
            vertices[s][p][9] = (volume_vertex_t){.position={ends[3].x, ends[3].y,  z}, .texcoord={1, (1+ends[3].x)*0.5f, (1+ends[3].y)*0.5f}, .dotcoord={radius*rim[1], (float)sim_geometry.screen_height}};
            vertices[s][p][10] = vertices[s][p][8];
            vertices[s][p][11] = vertices[s][p][7];

        }
    }

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    return sizeof(vertices) / sizeof(volume_vertex_t);
}

static size_t create_mesh_linear() {
    volume_vertex_t vertices[sim_geometry.slice_count][6];

    const float zaspect = ((float)VOXELS_Z / (float)VOXELS_X);

    for (int s = 0; s < sim_geometry.slice_count; ++s) {
        float z = ((((float)s + 0.5f) / (float)(sim_geometry.slice_count)) * 2.0f - 1.0f);

        vertices[s][0] = (volume_vertex_t){.position={ -1, -1, z * zaspect}, .texcoord={(1+z)*0.5, 0, 0}, .dotcoord={-(float)sim_geometry.screen_width*0.5f,-(float)sim_geometry.screen_height*0.5f}};
        vertices[s][1] = (volume_vertex_t){.position={  1, -1, z * zaspect}, .texcoord={(1+z)*0.5, 1, 0}, .dotcoord={ (float)sim_geometry.screen_width*0.5f,-(float)sim_geometry.screen_height*0.5f}};
        vertices[s][2] = (volume_vertex_t){.position={ -1,  1, z * zaspect}, .texcoord={(1+z)*0.5, 0, 1}, .dotcoord={-(float)sim_geometry.screen_width*0.5f, (float)sim_geometry.screen_height*0.5f}};
        vertices[s][3] = (volume_vertex_t){.position={  1,  1, z * zaspect}, .texcoord={(1+z)*0.5, 1, 1}, .dotcoord={ (float)sim_geometry.screen_width*0.5f, (float)sim_geometry.screen_height*0.5f}};
        vertices[s][4] = vertices[s][2];
        vertices[s][5] = vertices[s][1];
    }

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    return sizeof(vertices) / sizeof(volume_vertex_t);
}

static void init_mesh(void) {
    glGenVertexArrays(1, &volume.vao);
    glGenBuffers(1, &volume.vbo);

    glBindVertexArray(volume.vao);

    glBindBuffer(GL_ARRAY_BUFFER, volume.vbo);

    if (sim_geometry.scan_geometry == SCAN_LINEAR) {
        volume.vertex_count = create_mesh_linear();
    } else {
        volume.vertex_count = create_mesh_radial();
    }

    GLint a_position = glGetAttribLocation(volume.program, "a_position");
    glVertexAttribPointer(a_position, 3, GL_FLOAT, GL_FALSE, sizeof(volume_vertex_t), (void*)offsetof(volume_vertex_t, position));
    glEnableVertexAttribArray(a_position);
    
    GLint a_texcoord = glGetAttribLocation(volume.program, "a_texcoord");
    glVertexAttribPointer(a_texcoord, 3, GL_FLOAT, GL_FALSE, sizeof(volume_vertex_t), (void*)offsetof(volume_vertex_t, texcoord));
    glEnableVertexAttribArray(a_texcoord);

    GLint a_dotcoord = glGetAttribLocation(volume.program, "a_dotcoord");
    glVertexAttribPointer(a_dotcoord, 2, GL_FLOAT, GL_FALSE, sizeof(volume_vertex_t), (void*)offsetof(volume_vertex_t, dotcoord));
    glEnableVertexAttribArray(a_dotcoord);
}


bool sim_init(int argc, char** argv) {
    parse_args(argc, argv);

    if (!map_volume()) {
        return false;
    }
    voxel_buffer->bits_per_channel = sim_bpc;

    volume.program = create_program(volume_vert, volume_frag);
    if (!volume.program) {
        return false;
    }
    volume.u_view = glGetUniformLocation(volume.program, "u_view");
    volume.u_proj = glGetUniformLocation(volume.program, "u_proj");
    volume.u_bpcmask = glGetUniformLocation(volume.program, "u_bpcmask");
    volume.u_brightness = glGetUniformLocation(volume.program, "u_brightness");

    GLuint u_dotlock = glGetUniformLocation(volume.program, "u_dotlock");
    glUseProgram(volume.program);
    glUniform1i(u_dotlock, sim_geometry.scan_geometry == SCAN_RADIAL);

    init_texture();
    init_mesh();

    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);

    mat4_perspective(mat_proj, to_radians(35), (float)viewport_width / (float)viewport_height, 1, 100);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glViewport(0, 0, viewport_width, viewport_height);

    return true;
}

void sim_close(void) {
    unmap_volume();
}

void sim_resize(int w, int h) {
    viewport_width = w;
    viewport_height = h;
    glViewport(0, 0, viewport_width, viewport_height);
}

void sim_drag(int button, float dx, float dy) {
    switch (button) {
        case 1:
            model_rotation.x = clampf(model_rotation.x + dy * 3, -M_PI*0.95f, -M_PI*0.05f);
            model_rotation.z = fmodf(model_rotation.z + dx * 3, M_PI * 2);
            break;

        case 2:
            view_position.x = clampf(view_position.x + dx, -1.0f, 1.0f);
            view_position.y = clampf(view_position.y - dy, -1.0f, 1.0f);
            break;
        
    }
}

void sim_zoom(float d) {
    view_position.z = clampf(view_position.z + d, -10.0f, -0.1f);
}

void sim_draw(void) {
    float matrix[MAT4_SIZE];
    
    mat4_identity(matrix);
    mat4_rotation_z(matrix, model_rotation.z);

    mat4_identity(mat_view);
    mat4_rotation_x(mat_view, model_rotation.x);
    mat4_multiply(mat_view, mat_view, matrix);

    mat4_translate(mat_view, mat_view, view_position.v);

    mat4_perspective(mat_proj, to_radians(35), (float)viewport_width / (float)viewport_height, 0.1f, 100.0f);

    glBindTexture(GL_TEXTURE_3D, volume.texture);

    for (int y = 0; y < VOXELS_Y; ++y) {
        // splitting up the texture upload causes less flicker with simultaneous page flips
#ifdef HIGH_COLOUR
        glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, y, VOXELS_Z, VOXELS_X, 1, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, &voxel_buffer->volume[voxel_buffer->page][VOXEL_INDEX(0,y,0)]);
#else
        glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, y, VOXELS_Z, VOXELS_X, 1, GL_RED_INTEGER, GL_UNSIGNED_BYTE, &voxel_buffer->volume[voxel_buffer->page][VOXEL_INDEX(0,y,0)]);
#endif
    }

    glUseProgram(volume.program);

    glUniformMatrix4fv(volume.u_proj, 1, GL_FALSE, (float*)&mat_proj);
    glUniformMatrix4fv(volume.u_view, 1, GL_FALSE, (float*)&mat_view);

    const int bpcmask[4] = {0b11111111, 0b10010010, 0b11011011, 0b11111111};
    glUniform1i(volume.u_bpcmask, bpcmask[voxel_buffer->bits_per_channel & 3]);

    glUniform1f(volume.u_brightness, sim_brightness);

    glBindVertexArray(volume.vao);
    glBindBuffer(GL_ARRAY_BUFFER, volume.vbo);

    glDrawArrays(GL_TRIANGLES, 0, volume.vertex_count);
}

