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
void *createFixture(void *world_, void *body, void *shape, double density, double friction, double restitution,
                    int groupIndex, int categoryBits, int maskBits, const char *name);
void *createFixtureCircle(void *world_, void *body, double r, double offx, double offy, double angle,
                          double density, double friction, double restitution,
                          int groupIndex, int categoryBits, int maskBits, const char *name);
void *createFixtureBox(void *world_, void *body, double w, double h, double offx, double offy, double angle,
                       double density, double friction, double restitution,
                       int groupIndex, int categoryBits, int maskBits, const char *name);
void *createFixturePolygon(void *world_, void *body,
                           const char *serialized, int n_vertices,
                           double density, double friction, double restitution,
                           int groupIndex, int categoryBits, int maskBits,
                           const char *name);
void *createFixtureChain(void *world_, void *body,
                         const char *serialized, int n_vertices,
                         double density, double friction, double restitution,
                         int groupIndex, int categoryBits, int maskBits,
                         const char *name);
void *createRevoluteJoint(void *world_, void *bodyA, void *bodyB, int collideConnected,
                          double localAnchorAx, double localAnchorAy,
                          double localAnchorBx, double localAnchorBy);
void destroy(void *world, void *body);
void setFilter(void *body_, const char *fixtureName, int all, int filterData, int which);
void setFilterBit(void *body_, const char *fixtureName, int all, int filterData, int which);
void unsetFilterBit(void *body_, const char *fixtureName, int all, int filterData, int which);

// disgusting concepts
void queryAABB(void *world_, const char *query_initiator, const char *name,
               double lx, double ly, double ux, double uy);
void *topQueryResult(void *world_);
void popQuery(void *world_);
const char *getQueryId(void *result_);
const char *getQueryName(void *result_);
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
const char* getColFixtureName(void *collision_, int selector);
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
