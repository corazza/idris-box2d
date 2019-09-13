#include "box2d.h"

#include <Box2D/Box2D.h>
#include <queue>
#include <set>
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
    b2Fixture *fixture;
    double velx, vely; // velocity on collision

    for_body(b2Fixture *fixture) : fixture(fixture) {
      body = fixture->GetBody();
      velx = body->GetLinearVelocity().x;
      vely = body->GetLinearVelocity().y;
    }

    int id() {
      body_data* data = (body_data *) body->GetUserData();
      return data->id;
    }
  } one, two;

  collision_data(bool start, b2Fixture *one, b2Fixture *two) :
    start(start), one(one), two(two) {}
};

struct MyContactListener : public b2ContactListener {
  std::queue<collision_data> collisions;

  void process(b2Contact *contact, bool start) {
    b2Fixture *one = contact->GetFixtureA();
    b2Fixture *two = contact->GetFixtureB();
    collisions.push(collision_data(start, one, two));
  }

  void BeginContact(b2Contact* contact) {
    process(contact, true);
  }

  void EndContact(b2Contact* contact) {
    process(contact, false);
  }
};

struct query_result {
  int query_id;
  b2Body *body;

  query_result(int query_id, b2Body *body) :
    query_id(query_id), body(body) {}
};

class QueryCallback : public b2QueryCallback {
public:
  int query_id;
  std::vector<b2Body *> found;
  std::set<b2Body *> already_found;

  QueryCallback(int query_id) : query_id(query_id) {}

  bool ReportFixture(b2Fixture* fixture) {
    auto body = fixture->GetBody();
    if (already_found.find(body) == already_found.end()) {
      already_found.insert(body);
      found.push_back(body);
    }
    return true; //keep going to find all fixtures in the query area
  }
};

struct fixture_data {
  std::string name;

  fixture_data(const char* name) : name(name) {}
};

// wraps a world pointer and user data
struct world_data {
  b2World *world;
  int id_counter;
  MyContactListener *contactListener;
  // std::vector<QueryCallback*> queries;
  std::set<b2Body *> query_results_set;
  std::queue<query_result> query_results;
  std::vector<fixture_data*> fixtures_data;

  world_data(b2World *world)
    : world(world), id_counter(0), contactListener(new MyContactListener) {
    world->SetContactListener(contactListener);
  }

  ~world_data() {
    delete world;
    delete contactListener;
  }

  // returns pointer used for setting the user data of a fixture
  void *addFixtureName(const char *name) {
    fixture_data *data = new fixture_data(name);
    fixtures_data.push_back(data);
    return data;
  }

  void clearFixtureData(b2Fixture* f) {
    std::vector<fixture_data*>::iterator it;
    it = find(fixtures_data.begin(), fixtures_data.end(), f->GetUserData());
    if (it != fixtures_data.end()) {
      fixture_data *ptr = *it;
      delete ptr;
      fixtures_data.erase(it);
    }
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

void queryAABB(void *world_, int query_id, double lx, double ly, double ux, double uy) {
  world_data *world = (world_data *) world_;
  QueryCallback queryCallback = QueryCallback(query_id);
  b2AABB aabb;
  aabb.lowerBound = b2Vec2(lx, ly);
  aabb.upperBound = b2Vec2(ux, uy);
  world->world->QueryAABB(&queryCallback, aabb);
  for (int j = 0; j < queryCallback.found.size(); ++j) {
    world->query_results.push(query_result(query_id, queryCallback.found[j]));
  }
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

void *createFixture(void *world_, void *body_, void *shape, double density, double friction,
                    double restitution, int groupIndex, int categoryBits,
                    int maskBits, const char *name) {
  world_data *world = (world_data *) world_;
  void* fixture_name_ptr = world->addFixtureName(name);

  b2FixtureDef fixtureDef;
  fixtureDef.shape = shape;
  fixtureDef.density = density;
  fixtureDef.friction = friction;
  fixtureDef.restitution = restitution;
  fixtureDef.filter.groupIndex = groupIndex;
  fixtureDef.filter.categoryBits = categoryBits;
  fixtureDef.filter.maskBits = maskBits;
  fixtureDef.userData = fixture_name_ptr;
  b2Body *body = (b2Body *) body_;
  body->CreateFixture(&fixtureDef);
}

// 0 - groupIndex, 1 - categoryBits, 2 - maskBits
void setFilter(void *body_, const char *fixtureName, int all, int filterData, int which) {
  b2Body *body = (b2Body *) body_;
  for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
    fixture_data *data = f->GetUserData();
    if (all || data->name == fixtureName) {
      b2Filter filter = f->GetFilterData();
      switch(which) {
        case 0:
        filter.groupIndex = filterData;
        break;

        case 1:
        filter.categoryBits = filterData;
        break;

        case 2:
        filter.maskBits = filterData;
        break;
      }
      f->SetFilterData(filter);
    }
  }
}

void setFilterBit(void *body_, const char *fixtureName, int all, int filterData, int which) {
  b2Body *body = (b2Body *) body_;
  for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
    fixture_data *data = f->GetUserData();
    if (all || data->name == fixtureName) {
      b2Filter filter = f->GetFilterData();
      switch(which) {
        case 1:
        filter.categoryBits |= filterData;
        break;

        case 2:
        filter.maskBits |= filterData;
        break;
      }
      f->SetFilterData(filter);
    }
  }
}

void unsetFilterBit(void *body_, const char *fixtureName, int all, int filterData, int which) {
  b2Body *body = (b2Body *) body_;
  for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
    fixture_data *data = f->GetUserData();
    if (all || data->name == fixtureName) {
      b2Filter filter = f->GetFilterData();
      switch(which) {
        case 1:
        filter.categoryBits &= ~filterData;
        break;

        case 2:
        filter.maskBits &= ~filterData;
        break;
      }
      f->SetFilterData(filter);
    }
  }
}

void *createRevoluteJoint(void *world_, void *bodyA, void *bodyB, int collideConnected,
                          double localAnchorAx, double localAnchorAy,
                          double localAnchorBx, double localAnchorBy) {
  world_data *world = (world_data *) world_;
  b2RevoluteJointDef revoluteJointDef;
  revoluteJointDef.bodyA = bodyA;
  revoluteJointDef.bodyB = bodyB;
  revoluteJointDef.collideConnected = collideConnected;
  revoluteJointDef.localAnchorA.Set(localAnchorAx, localAnchorAy);
  revoluteJointDef.localAnchorB.Set(localAnchorBx, localAnchorBy);
  return world->world->CreateJoint(&revoluteJointDef);
}

void *createFixtureCircle(void *world_, void *body, double r, double offx, double offy, double angle,
                          double density, double friction, double restitution,
                          int groupIndex, int categoryBits, int maskBits,
                          const char *name) {
  b2CircleShape circleShape;
  circleShape.m_p.Set(offx, offy);
  circleShape.m_radius = r;
  return createFixture(world_, body, &circleShape, density, friction, restitution,
                       groupIndex, categoryBits, maskBits, name);
}

void *createFixtureBox(void *world_, void *body, double w, double h, double offx, double offy,
                       double angle, double density, double friction, double restitution,
                       int groupIndex, int categoryBits, int maskBits,
                       const char *name) {
  b2PolygonShape box;
  box.SetAsBox(w, h, b2Vec2(offx, offy), angle);
  return createFixture(world_, body, &box, density, friction, restitution,
                       groupIndex, categoryBits, maskBits, name);
}

void destroy(void *world_, void *body_) {
  world_data *world = (world_data *) world_;
  b2Body *body = (b2Body *) body_;

  for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
    world->clearFixtureData(f);
  }

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

void applyForce(void *body_, double x, double y, double offx, double offy) {
  b2Body *body = (b2Body *) body_;
  b2Vec2 force(x, y);
  b2Vec2 offset(offx, offy);
  body->ApplyForce(force, body->GetWorldPoint(offset), true);
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

double getWCx(void* body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetWorldCenter().x;
}

double getWCy(void* body_) {
  b2Body *body = (b2Body *) body_;
  return body->GetWorldCenter().y;
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

void *topQueryResult(void *world_) {
  world_data *world = (world_data *) world_;
  if (!world->query_results.empty())
    return &world->query_results.front();
  else return nullptr;
}

void popQuery(void *world_) {
  world_data *world = (world_data *) world_;
  world->query_results.pop();
}

int getQueryId(void *result_) {
  query_result *result = (query_result *) result_;
  return result->query_id;
}

int getQueryBodyId(void *result_) {
  query_result *result = (query_result *) result_;
  body_data* data = (body_data *) result->body->GetUserData();
  return data->id;
}

void *getQueryBody(void *result_) {
  query_result *result = (query_result *) result_;
  return result->body;
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

const char* getColFixtureName(void *collision_, int selector) {
  assert(selector == 1 || selector == 2);
  collision_data *collision = (collision_data *) collision_;
  b2Fixture *fixture = selector==1 ? collision->one.fixture : collision->two.fixture;
  fixture_data *data = fixture->GetUserData();
  return data->name.c_str();
}

int getStart(void *collision_) {
  collision_data *collision = (collision_data *) collision_;
  return collision->start ? 1 : 0;
}
