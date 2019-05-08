#include "box2d.h"

#include <Box2D/Box2D.h>
#include <queue>
#include <iostream>

struct body_data {
  int id;
};

struct collision_data {
  bool start;
  b2Body *one, *two;
  int id_one, id_two;

  collision_data(bool start, b2Body *one, b2Body *two, int id_one, int id_two) :
    start(start), one(one), two(two), id_one(id_one), id_two(id_two) {}
};

struct MyContactListener : public b2ContactListener {
  std::queue<collision_data> collisions;

  void process(b2Contact *contact, bool start) {
    b2Body *one = contact->GetFixtureA()->GetBody();
    b2Body *two = contact->GetFixtureB()->GetBody();
    body_data* oneData = (body_data *) one->GetUserData();
    body_data* twoData = (body_data *) two->GetUserData();
    collisions.push(collision_data(start, one, two, oneData->id, twoData->id));
  }

  void BeginContact(b2Contact* contact) {
    process(contact, true);
  }

  void EndContact(b2Contact* contact) {
    process(contact, false);
  }
};

// bad TODO fix
int wglobal_id;
MyContactListener *contactListener;

void *topCollision() {
  if (!contactListener->collisions.empty())
    return &contactListener->collisions.front();
  else return nullptr;
}

void popCollision() {
  contactListener->collisions.pop();
}

void *getBodyOne(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->one;
}

void *getBodyTwo(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->two;
}

int getIdOne(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->id_one;
}

int getIdTwo(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->id_two;
}

int getStart(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->start ? 1 : 0;
}

void *createWorld(double x, double y) {
  wglobal_id = 0;
  b2Vec2 gravity(x, y);
  b2World *world = new b2World(gravity);
  contactListener = new MyContactListener;
  world->SetContactListener(contactListener);
  return world;
}

void destroyWorld(void *world_) {
  b2World *world = (b2World *) world_;
  delete world;
  delete contactListener;
}

void *createWall(void *world_, double posx, double posy, double dimx, double dimy) {
  b2World *world = (b2World *) world_;
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(posx, posy);
  auto bodyData = new body_data;
  bodyData->id = wglobal_id++;
  groundBodyDef.userData = bodyData;
  b2Body* groundBody = world->CreateBody(&groundBodyDef);
  b2PolygonShape groundBox;
  groundBox.SetAsBox(dimx, dimy);
  groundBody->CreateFixture(&groundBox, 0.0f);
  return groundBody;
}

void *createBox(void *world_, double posx, double posy, double dimx, double dimy,
                double angle, double density, double friction) {
  b2World *world = (b2World *) world_;
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position.Set(posx, posy);
  bodyDef.angle = angle;
  auto bodyData = new body_data;
  bodyData->id = wglobal_id++;
  bodyDef.userData = bodyData;
  b2Body* body = world->CreateBody(&bodyDef);
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox(dimy, dimy);
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;
  fixtureDef.density = density;
  fixtureDef.friction = friction;
  body->CreateFixture(&fixtureDef);
  return body;
}

void step(void *world_, double timeStep, int velocityIterations, int positionIterations) {
  b2World *world = (b2World *) world_;
  world->Step(timeStep, velocityIterations, positionIterations);
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

// TODO destroying bodies
