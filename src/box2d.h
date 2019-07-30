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

void *createBody(void *world, int type, double posx, double posy, double angle, int fixedRotation, int bullet);
void *createFixture(void *body, void *shape, double density, double friction, double restitution,
                    int groupIndex, int categoryBits, int maskBits);
void *createFixtureCircle(void *body, double r, double offx, double offy, double angle,
                          double density, double friction, double restitution,
                          int groupIndex, int categoryBits, int maskBits);
void *createFixtureBox(void *body, double w, double h, double offx, double offy, double angle,
                       double density, double friction, double restitution,
                       int groupIndex, int categoryBits, int maskBits);
void *createRevoluteJoint(void *world_, void *bodyA, void *bodyB, int collideConnected,
                          double localAnchorAx, double localAnchorAy,
                          double localAnchorBx, double localAnchorBy);
void destroy(void *world, void *body);


// disgusting concepts
void queryAABB(void *world_, int query_id, double lx, double ly, double ux, double uy);
void *topQueryResult(void *world_);
void popQuery(void *world_);
int getQueryId(void *result_);
int getQueryBodyId(void *result_);
void *getQueryBody(void *result_);

// collisions
void *topCollision(void *world);
void popCollision(void *world);
void *getBodyOne(void *collision);
void *getBodyTwo(void *collision);
int getIdOne(void *collision);
int getIdTwo(void *collision);
double getColVelx(void *collision_, int selector);
double getColVely(void *collision_, int selector);
int getStart(void *collision);

void step(void *world, double timeStep, int velocityIterations, int positionIterations);

void applyImpulse(void *body, double x, double y);
void applyForce(void *body, double x, double y, double offx, double offy);

int getId(void *body);
double getMass(void *body);
double getPosx(void *body);
double getPosy(void *body);
double getWCx(void *body);
double getWCy(void *body);
double getAngle(void *body);
double getVelx(void *body);
double getVely(void *body);

#ifdef __cplusplus
}
#endif

#endif
