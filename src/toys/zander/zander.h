#ifndef _ZANDER_H_
#define _ZANDER_H_

#include <math.h>
#include "mathc.h"
#include "voxel.h"

static const float world_gravity = 4.0f;

extern float world_scale;
extern struct vec3 world_position;

extern int8_t height_map[VOXELS_Y][VOXELS_X][2];
#define HEIGHT_MAP_OBJECT(x, y) (height_map[y][x][1])
#define HEIGHT_MAP_TERRAIN(x, y) (height_map[y][x][0])


static inline void foxel_from_world(float* voxel, const float* position) {
    voxel[0] = ((position[0] - world_position.x) * world_scale) + (VOXELS_X / 2);
    voxel[1] = ((position[1] - world_position.y) * world_scale) + (VOXELS_Y / 2);
    voxel[2] = ((position[2] - world_position.z) * world_scale);
}

static inline void voxel_from_world(int32_t* voxel, const float* position) {
    float foxel[3];
    foxel_from_world(foxel, position);
    for (int i = 0; i < 3; ++i) {
        voxel[i] = (int)(roundf(foxel[i]));

    }
}

void world_from_voxel(float* position, const int32_t* voxel);

float rand_range(float inf, float sup);

void zander_reset(void);


#endif
