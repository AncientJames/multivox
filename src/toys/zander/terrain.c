#include "terrain.h"

#include "mathc.h"
#include "rammel.h"
#include "graphics.h"
#include "model.h"
#include "zander.h"
#include "zsintable.h"
#include "particles.h"
#include "timer.h"

#define STEEP_SLOPES
#define SMOOTH_COASTLINE
#define CHEESY_WAVES
//#define HEIGHT_DITHER

static const int32_t TILE_SIZE = 0x01000000;
static const int32_t LAND_MID_HEIGHT = 0x05000000;
static const int32_t SEA_LEVEL = 0x05500000;
static const int32_t LAUNCHPAD_ALTITUDE = 0x03500000;
static const int32_t LAUNCHPAD_SIZE = 8;
//static const int32_t UNDERCARRIAGE_Y = 0x00640000;

static const float TILE_SCALE = 1.0f / (float)TILE_SIZE;

static const float terrain_max_height = 10.0f;

static pixel_t current_tile_colour = 0;
[[maybe_unused]] static const pixel_t colour_sea = HEXPIX(3F3FFF);
[[maybe_unused]] static const pixel_t colour_sand = HEXPIX(FFFF00);
[[maybe_unused]] static const pixel_t colour_launchpad = HEXPIX(7F7F7F);

static uint16_t bit_reverse(uint16_t n) {
    n = ((n & 0xAAAA) >>  1) | ((n & 0x5555) << 1);
    n = ((n & 0xCCCC) >>  2) | ((n & 0x3333) << 2);
    n = ((n & 0xF0F0) >>  4) | ((n & 0x0F0F) << 4);
    n = ((n & 0xFF00) >>  8) | ((n & 0x00FF) << 8);
    return n;
}

int32_t zsin(int32_t v) {
    return zsinTable[(v >> 22) & 1023];
}

static int32_t GetLandscapeAltitude(int32_t x, int32_t z) {
    int32_t r;

     r = zsin(     x -  2 * z) / 128;
    r += zsin( 4 * x +  3 * z) / 128;
    r += zsin( 3 * z -  5 * x) / 128;
    r += zsin( 7 * x +  5 * z) / 128;
    r += zsin( 5 * x + 11 * z) / 256;
    r += zsin(10 * x +  7 * z) / 256;

    return LAND_MID_HEIGHT - r;
}

static pixel_t GetLandscapeTileColour(int32_t altitude) {
#ifdef SMOOTH_COASTLINE
    if (altitude >= SEA_LEVEL - TILE_SIZE/8) {
        return colour_sand;
    }
#else
    if (altitude == SEA_LEVEL) {
        return colour_sea;
    }
#endif

    if (altitude == LAUNCHPAD_ALTITUDE) {
        return colour_launchpad;
    }

    int r = ((altitude>>2)&1) * 128;
    int g = ((altitude>>3)&1) * 128 + 64;
    
    return (RGBPIX(r, g, 0) | 0b00000100);
}

float bilerp(const float corners[2][2], const float local[2]) {
    float a0 = corners[0][0] * (1.0f - local[0]) + corners[1][0] * local[0];
    float a1 = corners[0][1] * (1.0f - local[0]) + corners[1][1] * local[0];
    return a0 * (1.0f - local[1]) + a1 * local[1];
}

static const int bayer_4x4[4][4] = {{ 0, 8, 2,10}, {12, 4,14, 6}, { 3,11, 1, 9}, {15, 7,13, 5}};

bool depth_dither(int32_t depth, float* position) {
    int x = (int)(position[0] * world_scale);
    int y = (int)(position[1] * world_scale);
    return depth*2 > -bayer_4x4[x&3][y&3];
}

float terrain_get_altitude_raw(float x, float y) {
    int32_t altitude = GetLandscapeAltitude((int)roundf(x*1024) * (TILE_SIZE/1024), (int)roundf(y*1024) * (TILE_SIZE/1024));
    return (SEA_LEVEL - altitude) * TILE_SCALE;
}

float terrain_get_altitude(float x, float y) {
    static float heights[2][2] = {};
    static int32_t cached[VEC2_SIZE] = {INT32_MAX, INT32_MAX};

    int32_t tile[VEC2_SIZE] = {floorf(x), floorf(y)};

    int32_t altitude;
    if (cached[0] != tile[0] || cached[1] != tile[1]) {
        static int32_t altitudes[2][2] = {};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                int32_t t[VEC2_SIZE] = {tile[0]+i, tile[1]+j};
                if ((uint32_t)t[0] < LAUNCHPAD_SIZE && (uint32_t)t[1] < LAUNCHPAD_SIZE) {
                    altitude = LAUNCHPAD_ALTITUDE;
                } else {
                    altitude = GetLandscapeAltitude(t[0]*TILE_SIZE, t[1]*TILE_SIZE);
                    if (altitude > SEA_LEVEL) {
                        altitude = SEA_LEVEL;
                    }
                }

                altitudes[i][j] = altitude;
                heights[i][j] = (SEA_LEVEL - altitude) * TILE_SCALE;
            }
        }

        current_tile_colour = GetLandscapeTileColour(altitudes[1][0]);
        memcpy(cached, tile, sizeof(tile));
    }

    float local[VEC2_SIZE] = {x - tile[0], y - tile[1]};
    return bilerp(heights, local);
}


void draw_ground(pixel_t* volume) {
#ifdef HEIGHT_DITHER
    const float height_fuzz = 0.25f;
    uint fuzz_origin = ((((int)world_position.x)&1)<<1) | (((int)world_position.y)&1);
#endif

    for (int y = 0; y < VOXELS_Y; ++y) {
        for (int x = 0; x < VOXELS_X; ++x) {
            vec3_t pos = {.x=((float)x - (VOXELS_X-1)*0.5f) / world_scale + world_position.x, .y=((float)y - (VOXELS_Y-1)*0.5f) / world_scale + world_position.y, 0};

            float altitude = terrain_get_altitude(pos.x, pos.y);

#ifdef HEIGHT_DITHER
            float dither = ((float)((((x&1)<<1)|(y&1))^fuzz_origin) - 1.5f) * height_fuzz;
#else
            const float dither = 0;
#endif
            int32_t z = ((altitude - world_position.z) * world_scale) + dither;
            height_map[y][x][0] = height_map[y][x][1] = clamp(z, -127, 127);

            if (z < 0 && depth_dither(z, pos.v)) {
                z = 0;
            }

#ifdef STEEP_SLOPES
            if (z >= 0) {
                int zinf = z;
                int zsup = z;

                if (y > 0) {
                    zinf = min(zinf, height_map[y-1][x][0]);
                    zsup = max(zsup, height_map[y-1][x][0]);
                }
                if (x > 0) {
                    zinf = min(zinf, height_map[y][x-1][0]);
                    zsup = max(zsup, height_map[y][x-1][0]);
                }
                
                zinf = max(0, zinf + 1);
                zsup = min(VOXELS_Z-1, zsup - 1);

                for (z = zinf; z <= zsup; ++z) {
                    volume[VOXEL_INDEX(x, y, z)] = current_tile_colour;
                }
            }
#endif

            pixel_t colour = current_tile_colour;

            if (terrain_is_water(altitude)) {
#ifdef SMOOTH_COASTLINE
                colour = colour_sea;
#endif
#ifdef CHEESY_WAVES
                //if ((x^y)&1) {
                    #define WAVE_PERIOD_MS (2500)
                    #define WAVE_HEIGHT (0.25f)
                    #define CREST_HEIGHT (WAVE_HEIGHT * 0.95f)
                    float depth = terrain_get_altitude_raw(pos.x, pos.y);
                    float crest = depth + sinf((((timer_frame_time + (int32_t)(pos.y*2000)) % (WAVE_PERIOD_MS * 256)) * 2 * M_PI / WAVE_PERIOD_MS) + depth*depth*16);
                    if (crest > 0) {
                        crest = powf(crest, 0.1f) * WAVE_HEIGHT;
                        if (crest > CREST_HEIGHT) {
                            //particles_add(pos.v, (float[3]){0,0,0}, PARTICLE_SPRAY);
                            colour = HEXPIX(BFBFFF);
                        }
                        
                        z = ((crest - world_position.z) * world_scale) + dither;
                    }
                //}
#endif
            }

            if ((uint32_t)z < VOXELS_Z) {
                volume[VOXEL_INDEX(x, y, z)] = colour;
            }
        }
    }
}
extern vec3_t ship_position;

void draw_stars(pixel_t* volume) {
    vec2i_t tile0 = {.x=floorf((-(VOXELS_X-1)*0.5f) / world_scale + world_position.x),
                     .y=floorf((-(VOXELS_Y-1)*0.5f) / world_scale + world_position.y)};

    int tiles = (int)ceilf((float)VOXELS_X / world_scale) + 1;

    vec3_t star = {.x=tile0.x, .y=tile0.y};

    for (int y = 0; y < tiles; ++y) {
        for (int x = 0; x < tiles; ++x) {
            star.x = tile0.x + x;
            star.y = tile0.y + y;
            star.z = sqrtf(fabsf(fmodf((float)bit_reverse(tile0.x+x) * 3.0f + (float)bit_reverse(tile0.y+y) * 7.0f, 1987))) + terrain_max_height;

            vec3i_t voxel;
            voxel_from_world(voxel.v, star.v);

            if ((uint32_t)voxel.x < VOXELS_X && (uint32_t)voxel.y < VOXELS_Y && (uint32_t)voxel.z < VOXELS_Z) {
                volume[VOXEL_INDEX(voxel.x, voxel.y, voxel.z)] = HEXPIX(FFFFFF);
            }

        }
    }
}

void terrain_init(void) {
}

void terrain_draw(pixel_t* volume) {

    draw_ground(volume);
    draw_stars(volume);
}
