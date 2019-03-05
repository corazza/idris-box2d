#ifndef BOX2DA_H__
#define BOX2DA_H__

#include <idris_rts.h>

#ifdef __cplusplus
extern "C" {
#endif

void *createWorld(double x, double y);

void destroyWorld(void *world);

void *createGroundBody(void *world, double posx, double posy, double dimx, double dimy);

void *createBox(void *world, double posx, double posy, double dimx, double dimy,
                double angle, double density, double friction);

void step(void *world, double timeStep, int velocityIterations, int positionIterations);

double getPosx(void* body);
double getPosy(void* body);
double getAngle(void* body);

#ifdef __cplusplus
}
#endif

#endif
