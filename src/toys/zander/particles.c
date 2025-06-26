#include "particles.h"

#include <stdlib.h>
#include <stdio.h>

#include "rammel.h"
#include "graphics.h"
#include "terrain.h"
#include "zander.h"
#include "objects.h"

typedef struct {
    vec3_t position;
    vec3_t velocity;
    float lifespan;
    pixel_t colour;
    uint8_t flags;
} particle_t;

particle_t particles[4096];
size_t particle_count = 0;

static const float tick_scale = 1.0f / 12.0f;
static const float velocity_scale = 8.0f / ((float)0x01000000);

void randomise_velocity(float* velocity, float amount) {
    float direction[VEC3_SIZE];
    do {
        for (int i = 0; i < 3; ++i) {
            direction[i] = ((float)rand() / (float)(RAND_MAX)) * 2.0f - 1.0f;
        }
    } while (vec3_length_squared(direction) > 1.0f);

    vec3_multiply_f(direction, direction, amount);
    vec3_add(velocity, velocity, direction);
}

void particles_init(void) {
    particle_count = 0;
}

void particles_add(const float* position, const float* velocity, particle_type_t type) {
    if (particle_count >= count_of(particles)) {
        return;
    }

    particle_t* particle = &particles[particle_count++];
    vec3_assign(particle->position.v, position);
    vec3_assign(particle->velocity.v, velocity);

    switch (type) {
        case PARTICLE_BULLET: {
            particle->colour = HEXPIX(FFFFAA);
            particle->lifespan = 20 * tick_scale;
            particle->flags = PARTICLE_SPLASHES | PARTICLE_BOUNCES /*| PARTICLE_DROPS*/ | PARTICLE_DESTROYS | PARTICLE_BIG_SPLASH | PARTICLE_EXPLODES;
        } break;

        case PARTICLE_EXHAUST: {
            particle->colour = HEXPIX(FFFFFF);
            particle->lifespan = rand_range(8, 8+8) * tick_scale;
            particle->flags = PARTICLE_COOL_DOWN | PARTICLE_SPLASHES | PARTICLE_BOUNCES | PARTICLE_DROPS;
            randomise_velocity(particle->velocity.v, 0x400000 * velocity_scale);
        } break;

        case PARTICLE_DEBRIS: {
            uint8_t r = (1 + (rand() % 3)) * 64;
            uint8_t g = (1 + (rand() % 3)) * 64;
            uint8_t b = (1 + (rand() % 3)) * 64;
            particle->colour = RGBPIX(r, g, b);
            particle->lifespan = rand_range(15, 15+64) * tick_scale;
            particle->flags = PARTICLE_SPLASHES | PARTICLE_BOUNCES | PARTICLE_DROPS;
            randomise_velocity(particle->velocity.v, 0x400000 * velocity_scale);
        } break;

        case PARTICLE_SPARK: {
            particle->colour = HEXPIX(FFFFFF);
            particle->lifespan = rand_range(8 , 8+8) * tick_scale;
            particle->flags = PARTICLE_COOL_DOWN | PARTICLE_SPLASHES | PARTICLE_BOUNCES | PARTICLE_DROPS;
            randomise_velocity(particle->velocity.v, 0x1000000 * velocity_scale);
        } break;

        case PARTICLE_SPRAY: {
            uint8_t b = ((rand() & 1) + 2) * 64;
            uint8_t rg = (rand() % b);
            particle->colour = RGBPIX(rg, rg, b);
            particle->lifespan = rand_range(20, 20+64) * tick_scale;
            particle->flags = PARTICLE_DROPS;
            randomise_velocity(particle->velocity.v, 0x400000 * velocity_scale);
        } break;

        case PARTICLE_SMOKE: {
            uint8_t rgb = (1 + (rand() % 3)) * 64;
            particle->colour = RGBPIX(rgb, rgb, rgb);
            particle->lifespan = rand_range(15, 15+128) * tick_scale;
            particle->flags = PARTICLE_BOUNCES;
            particle->velocity.z = 0x80000 * velocity_scale;
            randomise_velocity(particle->velocity.v, 0x80000 * velocity_scale);
        } break;

        case PARTICLE_ROCK: {
            particle->colour = HEXPIX(FFFF55);
            particle->lifespan = rand_range(170, 170+32) * tick_scale;
            particle->flags = PARTICLE_SPLASHES | PARTICLE_BOUNCES | PARTICLE_DROPS | PARTICLE_DESTROYS | PARTICLE_BIG_SPLASH | PARTICLE_EXPLODES;
            randomise_velocity(particle->velocity.v, 0x400000 * velocity_scale);
        } break;
    }
}

void particles_add_splash(const float* position, bool big_splash) {
    int splash_count = big_splash ? 256 : 16;
    splash_count = min(splash_count, (count_of(particles) - particle_count) / 2);
    for (int i = 0; i < splash_count; ++i) {
        particles_add(position, (float[3]){0, 0, 1}, PARTICLE_SPRAY);
    }
}

void particles_add_explosion(const float* position, int clusters) {
    clusters = min(clusters, (count_of(particles) - particle_count) / 4);

    for (int i = 0; i < clusters; ++i) {
        particles_add(position, (float[3]){0, 0, 0}, PARTICLE_SPARK);
        particles_add(position, (float[3]){0, 0, 0}, PARTICLE_DEBRIS);
        particles_add(position, (float[3]){0, 0, 0}, PARTICLE_SMOKE);
        particles_add(position, (float[3]){0, 0, 0}, PARTICLE_SPARK);
    }
}

void particles_delete(size_t particle) {
    if (particle >= particle_count) {
        return;
    }
    if (particle == --particle_count) {
        return;
    }

    memcpy(&particles[particle], &particles[particle_count], sizeof(*particles));
}

void particles_update(float dt) {
    //printf("\r%d particles       ", particle_count);
    
    for (int p = 0; p < particle_count; ++p) {
        particle_t* particle = &particles[p];

        particle->lifespan -= dt;
        if (particle->lifespan < 0) {
            particles_delete(p--);
            continue;
        }
        
        float dpos[VEC3_SIZE];
        vec3_multiply_f(dpos, particle->velocity.v, dt);
        vec3_add(particle->position.v, particle->position.v, dpos);

        if (particle->flags & PARTICLE_DROPS) {
            particle->velocity.z -= world_gravity * dt;
        }

        float ground = terrain_get_altitude(particle->position.x, particle->position.y);
        if (particle->position.z <= ground) {
            particle->position.z = ground;

            if (terrain_is_water(ground) && (particle->flags & PARTICLE_SPLASHES)) {
                particles_add_splash(particle->position.v, (particle->flags & PARTICLE_BIG_SPLASH) != 0);
                particles_delete(p--);
                continue;
            }

            if (!(particle->flags & PARTICLE_BOUNCES)) {
                particles_delete(p--);
                continue;
            }

            if (particle->flags & PARTICLE_EXPLODES) {
                particles_add_explosion(particle->position.v, 3*4);
                particles_delete(p--);
                continue;
            }


            particle->velocity.z = fabsf(particle->velocity.z) * 0.5f;
            vec2_multiply_f(particle->velocity.v, particle->velocity.v, 0.75f);
        }

        if (particle->flags & PARTICLE_DESTROYS) {
            if (objects_hit_and_destroy(particle->position.v)) {
                particles_delete(p--);
                continue;
            }
        }

        if (particle->flags & PARTICLE_COOL_DOWN) {
            int r = 255;
            int g = min((int)(particle->lifespan * (20/tick_scale)), 255);
            int b = min((int)(particle->lifespan * (10/tick_scale)), 255);
            particle->colour = RGBPIX(r, g, b);
        }
    }


}

void particles_draw(pixel_t* volume) {
    for (int p = 0; p < particle_count; ++p) {
        particle_t* particle = &particles[p];

        float head[VEC3_SIZE];
        foxel_from_world(head, particle->position.v);

        int32_t voxel[VEC3_SIZE] = {(int)roundf(head[0]), (int)roundf(head[1]), (int)roundf(head[2])};
        if ((uint32_t)voxel[0] < VOXELS_X && (uint32_t)voxel[1] < VOXELS_Y && (uint32_t)voxel[2] < VOXELS_Z) {

            float tail[VEC3_SIZE];
            vec3_multiply_f(tail, particle->velocity.v, 0.05f * world_scale);

            if (vec3_length_squared(tail) > 1.0f) {
                vec3_subtract(tail, head, tail);
                graphics_draw_line(volume, head, tail, particle->colour);
            } else {
                volume[VOXEL_INDEX(voxel[0], voxel[1], voxel[2])] = particle->colour;
            }

            // shadow
            int8_t surface = max(0, HEIGHT_MAP_OBJECT(voxel[0], voxel[1]));
            if (voxel[2] > surface) {
                uint32_t idx = VOXEL_INDEX(voxel[0], voxel[1], surface);
                volume[idx] = (volume[idx]&0b10010010)>>1;
            }

        }
    }
}
