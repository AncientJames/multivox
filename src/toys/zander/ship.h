#ifndef _SHIP_H_
#define _SHIP_H_

#include "mathc.h"
#include "voxel.h"

extern struct vec3 ship_position;

void ship_init(void);
void ship_update(float dt);
void ship_draw(pixel_t* volume);

#endif