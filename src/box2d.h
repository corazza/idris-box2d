#ifndef BOX2DA_H__
#define BOX2DA_H__

#include <idris_rts.h>

#ifdef __cplusplus
extern "C" {
#endif

// struct world_data {
//   int id_counter;
//
//   world_data() : id_counter(0){}
// };
//
// world_data single;

void *createWorld(double x, double y);

void destroyWorld(void *world);

void *createWall(void *world, double posx, double posy, double dimx, double dimy);

void *createBox(void *world, double posx, double posy, double dimx, double dimy,
                double angle, double density, double friction);

void destroy(void *world, void *body);

// disgusting concepts
void *topCollision();
void popCollision();
void *getBodyOne(void *collision);
void *getBodyTwo(void *collision);
int getIdOne(void *collision);
int getIdTwo(void *collision);
double getColVelx(void *collision_, int selector);
double getColVely(void *collision_, int selector);
int getStart(void *collision);

void step(void *world, double timeStep, int velocityIterations, int positionIterations);

void applyImpulse(void *body_, double x, double y);

int getId(void *body);
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
