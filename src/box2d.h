#ifndef BOX2DA_H__
#define BOX2DA_H__

#include <idris_rts.h>

#ifdef __cplusplus
extern "C" {
#endif

void *createWorld(double x, double y);

void destroyWorld(void *world);

void *createWall(void *world, double posx, double posy, double dimx, double dimy);

void *createBox(void *world, double posx, double posy, double dimx, double dimy,
                double angle, double density, double friction);

void step(void *world, double timeStep, int velocityIterations, int positionIterations);

void applyImpulse(void *body_, double x, double y);

double getMass(void *body);
double getPosx(void *body);
double getPosy(void *body);
double getAngle(void *body);
double getVelx(void *body);
double getVely(void *body);

#ifdef __cplusplus
}
#endif

#endif
