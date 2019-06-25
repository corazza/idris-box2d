#include "box2d.h"

#include <Box2D/Box2D.h>
#include <queue>
#include <iostream>

// held in body.user_data
struct body_data {
  int id;
};

struct collision_data {
  bool start;

  // collision data for a single body
  struct for_body {
    b2Body *body;
    double velx, vely; // velocity on collision

    for_body(b2Body *body) : body(body) {
      velx = body->GetLinearVelocity().x;
      vely = body->GetLinearVelocity().y;
    }

    int id() {
      body_data* data = (body_data *) body->GetUserData();
      return data->id;
    }
  } one, two;

  collision_data(bool start, b2Body *one, b2Body *two) :
    start(start), one(one), two(two) {}
};

struct MyContactListener : public b2ContactListener {
  std::queue<collision_data> collisions;

  void process(b2Contact *contact, bool start) {
    b2Body *one = contact->GetFixtureA()->GetBody();
    b2Body *two = contact->GetFixtureB()->GetBody();

    collisions.push(collision_data(start, one, two));
  }

  void BeginContact(b2Contact* contact) {
    process(contact, true);
  }

  void EndContact(b2Contact* contact) {
    process(contact, false);
  }
};

// wraps a world pointer and user data
struct world_data {
  b2World *world;
  int id_counter;
  MyContactListener *contactListener;

  world_data(b2World *world)
    : world(world), id_counter(0), contactListener(new MyContactListener) {
    world->SetContactListener(contactListener);
  }

  ~world_data() {
    delete world;
    delete contactListener;
  }
};

void *createWorld(double x, double y) {
  b2Vec2 gravity(x, y);
  b2World *box2dWorld = new b2World(gravity);
  world_data *world = new world_data(box2dWorld);
  return world;
}

void destroyWorld(void *world_) {
  world_data *world = (world_data *) world_;
  delete world;
}

void *createBody(void *world_, int type, double posx, double posy,
                 double angle, int fixedRotation, int bullet) {
  world_data *world = (world_data *) world_;
  b2BodyDef bodyDef;

  switch (type) {
  case 0:
    bodyDef.type = b2_staticBody;
    break;
  case 1:
    bodyDef.type = b2_dynamicBody;
    break;
  case 2:
    bodyDef.type = b2_kinematicBody;
    break;
  }

  bodyDef.position.Set(posx, posy);
  bodyDef.angle = angle;
  bodyDef.fixedRotation = fixedRotation;
  bodyDef.bullet = bullet;

  auto bodyData = new body_data;
  bodyData->id = world->id_counter++;
  bodyDef.userData = bodyData;

  return world->world->CreateBody(&bodyDef);
}

void *createFixture(void *body_, void *shape, double density, double friction, double restitution) {
  b2FixtureDef fixtureDef;
  fixtureDef.shape = shape;
  fixtureDef.density = density;
  fixtureDef.friction = friction;
  fixtureDef.restitution = restitution;
  b2Body *body = (b2Body *) body_;
  body->CreateFixture(&fixtureDef);
}

void *createFixtureCircle(void *body, double r, double offx, double offy, double angle,
                          double density, double friction, double restitution) {
  b2CircleShape circleShape;
  circleShape.m_p.Set(offx, offy);
  circleShape.m_radius = r;
  return createFixture(body, &circleShape, density, friction, restitution);
}

void *createFixtureBox(void *body, double w, double h, double offx, double offy,
                       double angle, double density, double friction, double restitution) {
  b2PolygonShape box;
  box.SetAsBox(w, h, b2Vec2(offx, offy), angle);
  return createFixture(body, &box, density, friction, restitution);
}

void destroy(void *world_, void *body_) {
  world_data *world = (world_data *) world_;
  b2Body *body = (b2Body *) body_;
  world->world->DestroyBody(body);
}

void step(void *world_, double timeStep, int velocityIterations, int positionIterations) {
  world_data *world = (world_data *) world_;
  world->world->Step(timeStep, velocityIterations, positionIterations);
}

void applyImpulse(void *body_, double x, double y) {
  b2Body *body = (b2Body *) body_;
  b2Vec2 impulse(x, y);
  body->ApplyLinearImpulse(impulse, body->GetWorldCenter(), true);
}

int getId(void *body_) {
  b2Body *body = (b2Body *) body_;
  body_data *bodyData = (body_data*) body->GetUserData();
  return bodyData->id;
}

double getMass(void *body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetMass();
}

double getPosx(void* body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetPosition().x;
}

double getPosy(void* body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetPosition().y;
}

double getAngle(void* body_) {
  b2Body *body = (b2Body *) body_;
  return ((b2Body *) body)->GetAngle();
}

double getVelx(void* body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetLinearVelocity().x;
}

double getVely(void* body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetLinearVelocity().y;
}

void *topCollision(void *world_) {
  world_data *world = (world_data *) world_;
  if (!world->contactListener->collisions.empty())
    return &world->contactListener->collisions.front();
  else return nullptr;
}

void popCollision(void *world_) {
  world_data *world = (world_data *) world_;
  world->contactListener->collisions.pop();
}

void *getBodyOne(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->one.body;
}

void *getBodyTwo(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->two.body;
}

int getIdOne(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->one.id();
}

int getIdTwo(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->two.id();
}

double getColVelx(void *collision_, int selector) {
  assert(selector == 1 || selector == 2);
  collision_data *collision = (collision_data *) collision_;
  return selector==1 ? collision->one.velx : collision->two.velx;
}

double getColVely(void *collision_, int selector) {
  assert(selector == 1 || selector == 2);
  collision_data *collision = (collision_data *) collision_;
  return selector==1 ? collision->one.vely : collision->two.vely;
}

int getStart(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->start ? 1 : 0;
}











// void *createWall(void *world_, double posx, double posy, double dimx, double dimy) {
//   b2World *world = (b2World *) world_;
//   b2BodyDef groundBodyDef;
//   groundBodyDef.position.Set(posx, posy);
//   auto bodyData = new body_data;
//   bodyData->id = wglobal_id++;
//   groundBodyDef.userData = bodyData;
//   b2Body* groundBody = world->CreateBody(&groundBodyDef);
//   b2PolygonShape groundBox;
//   groundBox.SetAsBox(dimx, dimy);
//   groundBody->CreateFixture(&groundBox, 0.0f);
//   return groundBody;
// }
//
// void *createBox(void *world_, double posx, double posy, double dimx, double dimy,
//                 double angle, double density, double friction) {
//   b2World *world = (b2World *) world_;
//   b2BodyDef bodyDef;
//   bodyDef.type = b2_dynamicBody;
//   bodyDef.position.Set(posx, posy);
//   bodyDef.angle = angle;
//   bodyDef.bullet = true;
//   auto bodyData = new body_data;
//   bodyData->id = wglobal_id++;
//   bodyDef.userData = bodyData;
//   b2Body* body = world->CreateBody(&bodyDef);
//   b2PolygonShape dynamicBox;
//   dynamicBox.SetAsBox(dimx, dimy);
//   b2FixtureDef fixtureDef;
//   fixtureDef.shape = &dynamicBox;
//   fixtureDef.density = density;
//   fixtureDef.friction = friction;
//   body->CreateFixture(&fixtureDef);
//   return body;
// }
