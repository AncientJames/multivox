#ifndef _PARTICLES_H_
#define _PARTICLES_H_

#include <stdbool.h>
#include "voxel.h"

typedef enum {
    PARTICLE_COOL_DOWN =  0x01,
    PARTICLE_IS_ROCK =    0x02,
    PARTICLE_SPLASHES =   0x04,
    PARTICLE_BOUNCES =    0x08,
    PARTICLE_DROPS =      0x10,
    PARTICLE_DESTROYS =   0x20,
    PARTICLE_BIG_SPLASH = 0x40,
    PARTICLE_EXPLODES =   0x80,
} particle_flags_t;

typedef enum {
    PARTICLE_BULLET,
    PARTICLE_EXHAUST,
    PARTICLE_SMOKE,
    PARTICLE_DEBRIS,
    PARTICLE_SPARK,
    PARTICLE_SPRAY,
    PARTICLE_ROCK
} particle_type_t;

void particles_add(const float* position, const float* velocity, particle_type_t type);
void particles_add_splash(const float* position, bool big_splash);
void particles_add_explosion(const float* position, int clusters);

void particles_init(void);
void particles_update(float dt);
void particles_draw(pixel_t* volume);

#endif

