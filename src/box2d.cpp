#include "box2d.h"

#include <Box2D/Box2D.h>

void *createWorld(double x, double y) {
  b2Vec2 gravity(x, y);
  return new b2World(gravity);
}

void destroyWorld(void *world_) {
  b2World *world = (b2World *) world_;
  delete world;
}

void *createGroundBody(void *world_, double posx, double posy, double dimx, double dimy) {
  b2World *world = (b2World *) world_;
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(posx, posy);
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
