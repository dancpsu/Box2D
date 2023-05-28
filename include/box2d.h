#ifndef BOX2D_H
#define BOX2D_H

#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define NUM_BODIES 100
#define NUM_ARBITERS 500
#define NUM_JOINTS 100

typedef struct {
  float x, y;
} B2Vec2;

extern B2Vec2 B2Vec2Make(float x, float y);
extern B2Vec2 B2Vec2MakeEmpty(void);
extern void B2Vec2Set(B2Vec2* v, float x, float y);
extern B2Vec2 B2Vec2Invert(const B2Vec2 v);
extern B2Vec2 B2Vec2Add(const B2Vec2 v1, const B2Vec2 v2);
extern float B2Vec2AddF(const B2Vec2 v1, float a);
extern B2Vec2 B2Vec2Sub(const B2Vec2 v1, const B2Vec2 v2);
extern B2Vec2 B2Vec2Mult(const B2Vec2 v1, const B2Vec2 v2);
extern B2Vec2 B2Vec2MultF(const B2Vec2 v1, float a);
extern float B2Vec2GetLength(const B2Vec2 v1);
extern float B2Vec2Dot(B2Vec2 v1, B2Vec2 v2);
extern float B2Vec2Cross(B2Vec2 v1, B2Vec2 v2);
extern B2Vec2 B2Vec2CrossF(B2Vec2 v1, float s);
extern B2Vec2 B2Vec2FCross(float s, B2Vec2 v1);

typedef struct {
  B2Vec2 col1;
  B2Vec2 col2;
} B2Mat22;

extern B2Mat22 B2Mat22Make(B2Vec2 v1, B2Vec2 v2);
extern B2Mat22 B2Mat22MakeWithAngle(float angle);
extern B2Mat22 B2Mat22MakeEmpty(void);
extern B2Mat22 B2Mat22Transpose(B2Mat22 m);
extern B2Mat22 B2Mat22Invert(B2Mat22 m);
extern B2Mat22 B2Mat22Mult(B2Mat22 m1, B2Mat22 m2);
extern B2Vec2 B2Mat22MultVec(B2Mat22 m1, B2Vec2 v1);
extern B2Mat22 B2Mat22Add(B2Mat22 m1, B2Mat22 m2);

extern float B2Abs(float a);
extern B2Vec2 B2Vec2Abs(B2Vec2 v1);
extern B2Mat22 B2Mat22Abs(B2Mat22 m1);

extern float B2Sign(float a);
extern float B2Min(float a, float b);
extern float B2Max(float a, float b);
extern float B2Clamp(float a, float low, float high);

extern void B2Swap(void** a, void** b);

typedef struct {
  // State
  B2Vec2 position;
  float rotation;
  B2Vec2 velocity;
  float angularVelocity;
  
  // Properties
  B2Vec2 width;
  float AABBHalfSize;
  float friction;
  float mass, invMass;
  float I, invI;
  
  // Applied forces
  B2Vec2 force;
  float torque;
  
  // The index of this Body
  uint16_t index; 

  // Is it breaking? (0 for not, value for frames until gone)
  char breaking;
  
  // Is rotation fixed? (1 for don't rotate)
  char fixedRot;
  
  // The sprite index to use
  char sprite;
  
  // Whether to horizontally flip the sprite
  char flip;

  // Reference to world
  void* world;
} B2Body;

extern void B2BodyCreate(B2Body *body);
extern void B2BodySet(B2Body* body, const B2Vec2 w, float m);
extern void B2BodyAddForce(B2Body* body, const B2Vec2 f);

typedef struct {
  B2Mat22 M;
  B2Vec2 localAnchor1, localAnchor2;
  B2Vec2 r1, r2;
  B2Vec2 bias;
  B2Vec2 P;    // accumulated impulse
  B2Body* body1;
  B2Body* body2;
  float biasFactor;
  float softness;
  
  // Reference to world
  void* world;
} B2Joint;

typedef union {
  struct B2Edges {
    char inEdge1;
    char outEdge1;
    char inEdge2;
    char outEdge2;
  } e;
  uint32_t value;
} B2FeaturePair;

typedef struct {
  B2Vec2 position;
  B2Vec2 normal;
  B2Vec2 r1, r2;
  float separation;
  float Pn;  // accumulated normal impulse
  float Pt;  // accumulated tangent impulse
  float Pnb;  // accumulated normal impulse for position bias
  float massNormal, massTangent;
  float bias;
  B2FeaturePair feature;
} B2Contact;

#ifndef MAX_ARBITER_POINTS
#define MAX_ARBITER_POINTS 2
#endif

typedef struct {
  // Connectivity
  B2Body* body1;
  B2Body* body2;

  // Combined friction
  float friction;
  
  // Run-time data
  int numContacts;
  B2Contact contacts[MAX_ARBITER_POINTS];
  
  // Reference to world
  void* world;
} B2Arbiter;

typedef struct {
  B2Vec2 gravity;
  int iterations;
  float pixelScale;
  
  B2Body* bodies;
  int bodies_count;
  B2Joint* joints;
  int joints_count;
  B2Arbiter* arbiters;
  int arbiter_count;
} B2World;

extern B2World* B2WorldCreate(B2Vec2 gravity, int iterations);

extern B2Body* B2WorldAddBody(int index);
extern B2Body* B2WorldGetBody(int i);
extern void B2WorldRemoveBody(int index);
extern B2Joint* B2WorldAddJoint(int index, B2Body* b1, B2Body* b2, float x, float y);
extern B2Joint* B2WorldGetJoint(int i);
extern void B2WorldRemoveJoint(int index);
extern void B2WorldClear();
extern void B2WorldStep(float dt);
extern B2Body* B2WorldGetBody(int i);
extern B2Joint* B2WorldGetJoint(int i);
extern void B2WorldBroadphase();
extern int B2WorldNumberOfContactsBetweenBodies(B2Body* body1, B2Body* body2);

extern void B2JointCreate(B2Joint *joint, B2Body* b1, B2Body* b2, const B2Vec2 anchor);
extern void B2JointCreateEmpty(B2Joint* joint);
extern void B2JointFree(B2Joint* body);
extern void B2JointPreStep(B2Joint* joint, float inv_dt);
extern void B2JointApplyImpulse(B2Joint* joint);

extern void B2ArbiterCreate(B2Arbiter* arbiter, B2Body* body1, B2Body* body2);
extern void B2ArbiterFree(B2Arbiter* arbiter);
extern void B2ArbiterUpdate(B2Arbiter* arbiter, B2Contact* newContacts, int numNewContacts);
extern void B2ArbiterPreStep(B2Arbiter* arbiter, float inv_dt);
extern void B2ArbiterApplyImpulse(B2Arbiter* arbiter);

extern int B2Collide(B2Contact* contacts, B2Body* body1, B2Body* body2);

#ifdef BOX2D_IMPLEMENTATION

// Vector

inline B2Vec2 B2Vec2Make(float x, float y) {
  return (B2Vec2){.x = x, .y = y};
}

inline B2Vec2 B2Vec2MakeEmpty(void) {
  return (B2Vec2){.x = 0.0f, .y = 0.0f};
}

inline void B2Vec2Set(B2Vec2* v, float x, float y) {
  v->x = x;
  v->y = y;
}

inline B2Vec2 B2Vec2Invert(const B2Vec2 v) {
  return (B2Vec2){.x = -(v.x), .y = -(v.y)};
}

inline B2Vec2 B2Vec2Add(const B2Vec2 v1, const B2Vec2 v2) {
  return (B2Vec2){.x = (v1.x + v2.x), .y = (v1.y + v2.y)};
}

inline B2Vec2 B2Vec2Sub(const B2Vec2 v1, const B2Vec2 v2) {
  return (B2Vec2){.x = (v1.x - v2.x), .y = (v1.y - v2.y)};
}

inline B2Vec2 B2Vec2Mult(const B2Vec2 v1, const B2Vec2 v2) {
  return (B2Vec2){.x = (v1.x * v2.x), .y = (v1.y * v2.y)};
}

inline B2Vec2 B2Vec2MultF(const B2Vec2 v1, float a) {
  return (B2Vec2){.x = (v1.x * a), .y = (v1.y * a)};
}

inline float B2Vec2GetLength(const B2Vec2 v) {
  return sqrtf(v.x * v.x + v.y * v.y);
}

inline float B2Vec2Dot(B2Vec2 a, B2Vec2 b) {
  return a.x * b.x + a.y * b.y;
}

inline float B2Vec2Cross(B2Vec2 a, B2Vec2 b) {
  return a.x * b.y - a.y * b.x;
}

inline B2Vec2 B2Vec2CrossF(B2Vec2 v, float s) {
  return (B2Vec2){.x = s * v.y, .y = -s * v.x};
}

inline B2Vec2 B2Vec2FCross(float s, B2Vec2 v) {
  return (B2Vec2){.x = -s * v.y, .y = s * v.x};
}


// Matrix

inline B2Mat22 B2Mat22Make(B2Vec2 v1, B2Vec2 v2) {
  return (B2Mat22){ .col1 = v1, .col2 = v2 };
}

inline B2Mat22 B2Mat22MakeWithAngle(float angle) {
  float c = cosf(angle), s = sinf(angle);
  return (B2Mat22){ .col1.x = c, .col1.y = s, .col2.x = -s, .col2.y = c };
}

inline B2Mat22 B2Mat22MakeEmpty(void) {
  return (B2Mat22){ .col1 = B2Vec2MakeEmpty(), .col2 = B2Vec2MakeEmpty() };
}

inline B2Mat22 B2Mat22Transpose(B2Mat22 m1) {
  return B2Mat22Make(
      B2Vec2Make(m1.col1.x, m1.col2.x),
      B2Vec2Make(m1.col1.y, m1.col2.y)
      );
}

inline B2Mat22 B2Mat22Invert(B2Mat22 m1) {
  float a = m1.col1.x, b = m1.col2.x, c = m1.col1.y, d = m1.col2.y;
  float det = a * d - b * c;
  det = 1.0f / det;

  return B2Mat22Make(
      B2Vec2Make(det * d, -det * c),
      B2Vec2Make(-det * b, det * a)
      );
}

inline B2Mat22 B2Mat22Mult(B2Mat22 m1, B2Mat22 m2) {
  return B2Mat22Make(B2Mat22MultVec(m1, m2.col1), B2Mat22MultVec(m1, m2.col2));
}

inline B2Vec2 B2Mat22MultVec(B2Mat22 m, B2Vec2 v) {
  return B2Vec2Make(m.col1.x * v.x + m.col2.x * v.y, m.col1.y * v.x + m.col2.y * v.y);
}

inline B2Mat22 B2Mat22Add(B2Mat22 m1, B2Mat22 m2) {
  return B2Mat22Make(B2Vec2Add(m1.col1, m2.col1), B2Vec2Add(m1.col2, m2.col2));
}


// Math

inline float B2Abs(float a) {
  return a > 0.0f ? a : -a;
}

inline B2Vec2 B2Vec2Abs(B2Vec2 v1) {
  return B2Vec2Make(fabsf(v1.x), fabsf(v1.y));
}

inline B2Mat22 B2Mat22Abs(B2Mat22 m1) {
  return B2Mat22Make(B2Vec2Abs(m1.col1), B2Vec2Abs(m1.col2));
}

inline float B2Sign(float a) {
  return a < 0.0f ? -1.0f : 1.0f;
}

inline float B2Min(float a, float b) {
  return a < b ? a : b;
}

inline float B2Max(float a, float b) {
  return a > b ? a : b;
}

inline float B2Clamp(float a, float low, float high) {
  return B2Max(low, B2Min(a, high));
}

inline void B2Swap(void** a, void** b) {
  void* tmp = *a;
  *a = *b;
  *b = tmp;
}

void B2BodyCreate(B2Body* body) {
  body->position = B2Vec2MakeEmpty();
  body->rotation = 0.0f;
  body->velocity = B2Vec2MakeEmpty();
  body->angularVelocity = 0.0f;
  body->fixedRot = 0;
  body->force = B2Vec2MakeEmpty();
  body->friction = 0.2f;
  body->width = B2Vec2Make(1.0f, 1.0f);
  body->AABBHalfSize = B2Vec2GetLength(body->width) * 0.5f;
  body->mass = FLT_MAX;
  body->invMass = 0.0f;
  body->I = FLT_MAX;
  body->invI = 0.0f;
}

void B2BodySet(B2Body* body, const B2Vec2 w, float m) {
  body->position = B2Vec2MakeEmpty();
  body->rotation = 0.0f;
  body->velocity = B2Vec2MakeEmpty();
  body->angularVelocity = 0.0f;
  body->fixedRot = 0;
  body->force = B2Vec2MakeEmpty();
  body->torque = 0.0f;
  body->friction = 0.2f;

  body->width = w;
  body->AABBHalfSize = B2Vec2GetLength(body->width) * 0.5f;
  body->mass = m;

  if(m < FLT_MAX) {
    body->invMass = 1.0f / m;
    body->I = m * (w.x * w.x + w.y * w.y) / 12.0f;
    body->invI = 1.0f / body->I;
  }
  else {
    body->invMass = 0.0f;
    body->I = FLT_MAX;
    body->invI = 0.0f;
  }
}

void B2BodyAddForce(B2Body* body, const B2Vec2 f) {
  body->force = B2Vec2Add(body->force, f);
}

void B2ArbiterCreate(B2Arbiter *arbiter, B2Body* b1, B2Body* b2) {
  memset(arbiter, 0, sizeof(B2Arbiter));

  if(b1 == NULL || b2 == NULL) {
    puts("box2d: B2Arbiter: attempting to create arbiter with NULL body\n");
  }

  if(b1 < b2) {
    arbiter->body1 = b1;
    arbiter->body2 = b2;
  }
  else {
    arbiter->body1 = b2;
    arbiter->body2 = b1;
  }

  arbiter->numContacts = B2Collide(arbiter->contacts, arbiter->body1, arbiter->body2);
  arbiter->friction = sqrtf(arbiter->body1->friction * arbiter->body2->friction);
}

void B2ArbiterUpdate(B2Arbiter* arbiter, B2Contact* newContacts, int numNewContacts) {
  B2Contact mergedContacts[2];

  for(int i = 0; i < numNewContacts; i++) {
    B2Contact* cNew = newContacts + i;
    int k = -1;
    for(int j = 0; j < arbiter->numContacts; ++j) {
      B2Contact* cOld = arbiter->contacts + j;
      if(cNew->feature.value == cOld->feature.value) {
        k = j;
        break;
      }
    }

    if(k > -1) {
	  B2Contact* cOld = arbiter->contacts + k;
      B2Contact* c = mergedContacts + i;
      *c = *cNew;
      c->Pn = cOld->Pn;
      c->Pt = cOld->Pt;
	  c->Pnb = cOld->Pnb;
    }
    else {
      mergedContacts[i] = newContacts[i];
    }
  }

  for(int i = 0; i < numNewContacts; ++i) {
    arbiter->contacts[i] = mergedContacts[i];
  }

  arbiter->numContacts = numNewContacts;
}

void B2ArbiterPreStep(B2Arbiter* arbiter, float inv_dt) {
  const float k_allowedPenetration = 0.00000001f;
  float k_biasFactor = 0.1f;  // B2PositionCorrection bias

  for(int i = 0; i < arbiter->numContacts; i++) {
    B2Contact* c = arbiter->contacts + i;

    B2Vec2 r1 = B2Vec2Sub(c->position, arbiter->body1->position);
    B2Vec2 r2 = B2Vec2Sub(c->position, arbiter->body2->position);

    // Precompute normal mass, tangent mass, and bias.
    float rn1 = B2Vec2Dot(r1, c->normal);
    float rn2 = B2Vec2Dot(r2, c->normal);
    float kNormal = arbiter->body1->invMass + arbiter->body2->invMass;
    kNormal += arbiter->body1->invI * (B2Vec2Dot(r1, r1) - rn1 * rn1) + arbiter->body2->invI * (B2Vec2Dot(r2, r2) - rn2 * rn2);
    c->massNormal = 1.0f / kNormal;

    B2Vec2 tangent = B2Vec2CrossF(c->normal, 1.0f);
    float rt1 = B2Vec2Dot(r1, tangent);
    float rt2 = B2Vec2Dot(r2, tangent);
    float kTangent = arbiter->body1->invMass + arbiter->body2->invMass;
    kTangent += arbiter->body1->invI * (B2Vec2Dot(r1, r1) - rt1 * rt1) + arbiter->body2->invI * (B2Vec2Dot(r2, r2) - rt2 * rt2);
    c->massTangent = 1.0f /  kTangent;

    c->bias = -k_biasFactor * inv_dt * B2Min(0.0f, c->separation + k_allowedPenetration);

    // Apply normal + friction impulse
    B2Vec2 P = B2Vec2Add(B2Vec2MultF(c->normal, c->Pn), B2Vec2MultF(tangent, c->Pt));

    arbiter->body1->velocity = B2Vec2Sub(arbiter->body1->velocity, B2Vec2MultF(P, arbiter->body1->invMass));
    if(!arbiter->body1->fixedRot) arbiter->body1->angularVelocity -= arbiter->body1->invI * B2Vec2Cross(r1, P);

    arbiter->body2->velocity = B2Vec2Add(arbiter->body2->velocity, B2Vec2MultF(P, arbiter->body2->invMass));
    if(!arbiter->body2->fixedRot) arbiter->body2->angularVelocity += arbiter->body2->invI * B2Vec2Cross(r2, P);
  }
}

void B2ArbiterApplyImpulse(B2Arbiter* arbiter) {
  B2Body* b1 = arbiter->body1;
  B2Body* b2 = arbiter->body2;

  for(int i = 0; i < arbiter->numContacts; ++i) {
    B2Contact* c = arbiter->contacts + i;
    c->r1 = B2Vec2Sub(c->position, b1->position);
    c->r2 = B2Vec2Sub(c->position, b2->position);

    // Relative velocity at contact
    B2Vec2 dv = B2Vec2Sub(B2Vec2Sub(B2Vec2Add(b2->velocity, B2Vec2FCross(b2->angularVelocity, c->r2)), b1->velocity), B2Vec2FCross(b1->angularVelocity, c->r1));

    // Compute normal impulse
    float vn = B2Vec2Dot(dv, c->normal);

    float dPn = c->massNormal * (-vn + c->bias);

    // Clamp the accumulated impulse
    float Pn0 = c->Pn;
    c->Pn = B2Max(Pn0 + dPn, 0.0f);
    dPn = c->Pn - Pn0;


    // Apply contact impulse
    B2Vec2 Pn = B2Vec2MultF(c->normal, dPn);

    b1->velocity = B2Vec2Sub(b1->velocity, B2Vec2MultF(Pn, b1->invMass));
    if (!b1->fixedRot) b1->angularVelocity -= b1->invI * B2Vec2Cross(c->r1, Pn);

    b2->velocity = B2Vec2Add(b2->velocity, B2Vec2MultF(Pn, b2->invMass));
    if (!b2->fixedRot) b2->angularVelocity += b2->invI * B2Vec2Cross(c->r2, Pn);

    // Relative velocity at contact
    dv = B2Vec2Sub(B2Vec2Sub(B2Vec2Add(b2->velocity, B2Vec2FCross(b2->angularVelocity, c->r2)), b1->velocity), B2Vec2FCross(b1->angularVelocity, c->r1));

    B2Vec2 tangent = B2Vec2CrossF(c->normal, 1.0f);
    float vt = B2Vec2Dot(dv, tangent);
    float dPt = c->massTangent * (-vt);

    // Compute friction impulse
    float maxPt = arbiter->friction * c->Pn;

    // Clamp friction
    float oldTangentImpulse = c->Pt;
    c->Pt = B2Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
    dPt = c->Pt - oldTangentImpulse;

    // Apply contact impulse
    B2Vec2 Pt = B2Vec2MultF(tangent, dPt);

    b1->velocity = B2Vec2Sub(b1->velocity, B2Vec2MultF(Pt, b1->invMass));
    if (!b1->fixedRot) b1->angularVelocity -= b1->invI * B2Vec2Cross(c->r1, Pt);

    b2->velocity = B2Vec2Add(b2->velocity, B2Vec2MultF(Pt, b2->invMass));
    if (!b2->fixedRot) b2->angularVelocity += b2->invI * B2Vec2Cross(c->r2, Pt);
  }
}

void B2JointCreate(B2Joint *joint, B2Body* b1, B2Body* b2, const B2Vec2 anchor) {
  memset(joint, 0, sizeof(B2Joint));

  joint->body1 = b1;
  joint->body2 = b2;

  B2Mat22 Rot1T = B2Mat22Transpose(B2Mat22MakeWithAngle(joint->body1->rotation));
  B2Mat22 Rot2T = B2Mat22Transpose(B2Mat22MakeWithAngle(joint->body2->rotation));

  joint->localAnchor1 = B2Mat22MultVec(Rot1T, B2Vec2Sub(anchor, joint->body1->position));
  joint->localAnchor2 = B2Mat22MultVec(Rot2T, B2Vec2Sub(anchor, joint->body2->position));

  joint->P.x = 0.0f;
  joint->P.y = 0.0f;

  joint->softness = 0.0f;
  joint->biasFactor = 0.2f;
}

void B2JointCreateEmpty(B2Joint* joint) {
  memset(joint, 0, sizeof(B2Joint));
  joint->biasFactor = 0.2f;
}

void B2JointPreStep(B2Joint* joint, float inv_dt) {
  B2Body* body1 = joint->body1;
  B2Body* body2 = joint->body2;

  B2Mat22 Rot1 = B2Mat22MakeWithAngle(body1->rotation);
  B2Mat22 Rot2 = B2Mat22MakeWithAngle(body2->rotation);

  joint->r1 = B2Mat22MultVec(Rot1, joint->localAnchor1);
  joint->r2 = B2Mat22MultVec(Rot2, joint->localAnchor2);

  B2Mat22 K1;
  K1.col1.x = body1->invMass + body2->invMass; K1.col2.x = 0.0f;
  K1.col1.y = 0.0f; K1.col2.y = body1->invMass + body2->invMass;

  B2Mat22 K2;
  K2.col1.x =  body1->invI * joint->r1.y * joint->r1.y; K2.col2.x = -body1->invI * joint->r1.x * joint->r1.y;
  K2.col1.y = -body1->invI * joint->r1.x * joint->r1.y; K2.col2.y =  body1->invI * joint->r1.x * joint->r1.x;

  B2Mat22 K3;
  K3.col1.x =  body2->invI * joint->r2.y * joint->r2.y; K3.col2.x = -body2->invI * joint->r2.x * joint->r2.y;
  K3.col1.y = -body2->invI * joint->r2.x * joint->r2.y; K3.col2.y =  body2->invI * joint->r2.x * joint->r2.x;

  B2Mat22 K = B2Mat22Add(B2Mat22Add(K1, K2), K3);
  K.col1.x += joint->softness;
  K.col2.y += joint->softness;

  joint->M = B2Mat22Invert(K);

  B2Vec2 p1 = B2Vec2Add(body1->position, joint->r1);
  B2Vec2 p2 = B2Vec2Add(body2->position, joint->r2);
  B2Vec2 dp = B2Vec2Sub(p2, p1);

  joint->bias = B2Vec2MultF(dp, -joint->biasFactor * inv_dt);

  joint->P.x = 0.0f;
  joint->P.y = 0.0f;
}

void B2JointApplyImpulse(B2Joint* joint) {
  B2Body* body1 = joint->body1;
  B2Body* body2 = joint->body2;

  B2Vec2 dv = B2Vec2Sub(B2Vec2Sub(B2Vec2Add(body2->velocity, B2Vec2FCross(body2->angularVelocity, joint->r2)), body1->velocity), B2Vec2FCross(body1->angularVelocity, joint->r1));

  B2Vec2 impulse = B2Mat22MultVec(joint->M, B2Vec2Sub(B2Vec2Sub(joint->bias, dv), B2Vec2MultF(joint->P, joint->softness)));

  body1->velocity = B2Vec2Sub(body1->velocity, B2Vec2MultF(impulse, body1->invMass));
  if (!body1->fixedRot) body1->angularVelocity -= body1->invI * B2Vec2Cross(joint->r1, impulse);

  body2->velocity = B2Vec2Add(body2->velocity, B2Vec2MultF(impulse, body2->invMass));
  if (!body2->fixedRot) body2->angularVelocity += body2->invI * B2Vec2Cross(joint->r2, impulse);

  joint->P = B2Vec2Add(joint->P, impulse);
}


// Vertex and edge numbering:
//
//    ^ y
//    |
//    e1
//  v2 ---- v1
//   |    |
//  e2 |    | e4  --> x
//   |    |
//  v3 ---- v4
//    e3


typedef enum {
  FACE_A_X,
  FACE_A_Y,
  FACE_B_X,
  FACE_B_Y
} B2Axis;

typedef enum {
  NO_EDGE = 0,
  EDGE1,
  EDGE2,
  EDGE3,
  EDGE4
} B2EdgeNumbers;

typedef struct {
  B2Vec2 v;
  B2FeaturePair fp;
} B2ClipVertex;

static int B2ClipSegmentToLine(B2ClipVertex vOut[2], B2ClipVertex vIn[2], const B2Vec2 normal, float offset, char clipEdge) {
  // Start with no output points
  int numOut = 0;

  // Calculate the distance of end points to the line
  float distance0 = B2Vec2Dot(normal, vIn[0].v) - offset;
  float distance1 = B2Vec2Dot(normal, vIn[1].v) - offset;

  // If the points are behind the plane
  if(distance0 <= 0.0f) { vOut[numOut++] = vIn[0]; }
  if(distance1 <= 0.0f) { vOut[numOut++] = vIn[1]; }

  // If the points are on different sides of the plane
  if(distance0 * distance1 < 0.0f) {
    // Find intersection point of edge and plane
    float interp = distance0 / (distance0 - distance1);
    vOut[numOut].v = B2Vec2Add(vIn[0].v, B2Vec2MultF(B2Vec2Sub(vIn[1].v, vIn[0].v), interp));
    if(distance0 > 0.0f) {
      vOut[numOut].fp = vIn[0].fp;
      vOut[numOut].fp.e.inEdge1 = clipEdge;
      vOut[numOut].fp.e.inEdge2 = NO_EDGE;
    }
    else {
      vOut[numOut].fp = vIn[1].fp;
      vOut[numOut].fp.e.outEdge1 = clipEdge;
      vOut[numOut].fp.e.outEdge2 = NO_EDGE;
    }
    numOut++;
  }

  return numOut;
}

static void ComputeIncidentEdge(B2ClipVertex c[2], const B2Vec2 h, const B2Vec2 pos, const B2Mat22 Rot, const B2Vec2 normal) {
  // The normal is from the reference box. Convert it
  // to the incident box's frame and flip sign.
  B2Mat22 RotT = B2Mat22Transpose(Rot);
  B2Vec2 n = B2Vec2Invert(B2Mat22MultVec(RotT, normal));
  B2Vec2 nAbs = B2Vec2Abs(n);

  if(nAbs.x > nAbs.y) {
    if(B2Sign(n.x) > 0.0f) {
      B2Vec2Set(&c[0].v, h.x, -h.y);
      c[0].fp.e.inEdge2 = EDGE3;
      c[0].fp.e.outEdge2 = EDGE4;

      B2Vec2Set(&c[1].v, h.x, h.y);
      c[1].fp.e.inEdge2 = EDGE4;
      c[1].fp.e.outEdge2 = EDGE1;
    }
    else {
      B2Vec2Set(&c[0].v, -h.x, h.y);
      c[0].fp.e.inEdge2 = EDGE1;
      c[0].fp.e.outEdge2 = EDGE2;

      B2Vec2Set(&c[1].v, -h.x, -h.y);
      c[1].fp.e.inEdge2 = EDGE2;
      c[1].fp.e.outEdge2 = EDGE3;
    }
  }
  else {
    if(B2Sign(n.y) > 0.0f) {
      B2Vec2Set(&c[0].v, h.x, h.y);
      c[0].fp.e.inEdge2 = EDGE4;
      c[0].fp.e.outEdge2 = EDGE1;

      B2Vec2Set(&c[1].v, -h.x, h.y);
      c[1].fp.e.inEdge2 = EDGE1;
      c[1].fp.e.outEdge2 = EDGE2;
    }
    else {
      B2Vec2Set(&c[0].v, -h.x, -h.y);
      c[0].fp.e.inEdge2 = EDGE2;
      c[0].fp.e.outEdge2 = EDGE3;

      B2Vec2Set(&c[1].v, h.x, -h.y);
      c[1].fp.e.inEdge2 = EDGE3;
      c[1].fp.e.outEdge2 = EDGE4;
    }
  }

  c[0].v = B2Vec2Add(pos, B2Mat22MultVec(Rot, c[0].v));
  c[1].v = B2Vec2Add(pos, B2Mat22MultVec(Rot, c[1].v));
}

int B2Collide(B2Contact* contacts, B2Body* bodyA, B2Body* bodyB) {
  // Early discard with a simple AABB
  float AABBSum = bodyA->AABBHalfSize + bodyB->AABBHalfSize;
  if ( B2Abs(bodyA->position.x-bodyB->position.x)>AABBSum || B2Abs(bodyA->position.y-bodyB->position.y)>AABBSum ) 
    return 0;

  // Setup
  B2Vec2 hA = B2Vec2MultF(bodyA->width, 0.5f);
  B2Vec2 hB = B2Vec2MultF(bodyB->width, 0.5f);

  B2Vec2 posA = bodyA->position;
  B2Vec2 posB = bodyB->position;

  B2Mat22 RotA = B2Mat22MakeWithAngle(bodyA->rotation);
  B2Mat22 RotB = B2Mat22MakeWithAngle(bodyB->rotation);

  B2Mat22 RotAT = B2Mat22Transpose(RotA);
  B2Mat22 RotBT = B2Mat22Transpose(RotB);

  B2Vec2 dp = B2Vec2Sub(posB, posA);
  B2Vec2 dA = B2Mat22MultVec(RotAT, dp);
  B2Vec2 dB = B2Mat22MultVec(RotBT, dp);

  B2Mat22 C = B2Mat22Mult(RotAT, RotB);
  B2Mat22 absC = B2Mat22Abs(C);
  B2Mat22 absCT = B2Mat22Transpose(absC);

  // Box A faces
  B2Vec2 faceA = B2Vec2Sub(B2Vec2Sub(B2Vec2Abs(dA), hA), B2Mat22MultVec(absC, hB));
  if(faceA.x > 0.0f || faceA.y > 0.0f)
    return 0;

  // Box B faces
  B2Vec2 faceB = B2Vec2Sub(B2Vec2Sub(B2Vec2Abs(dB), B2Mat22MultVec(absCT, hA)), hB);
  if(faceB.x > 0.0f || faceB.y > 0.0f)
    return 0;

  // Find best axis
  B2Axis axis;
  float separation;
  B2Vec2 normal;

  // Box A faces
  axis = FACE_A_X;
  separation = faceA.x;
  normal = dA.x > 0.0f ? RotA.col1 : B2Vec2Invert(RotA.col1);

  const float relativeTol = 0.9995f;
  const float absoluteTol = 0.0005f;

  if(faceA.y > relativeTol * separation + absoluteTol * hA.y) {
    axis = FACE_A_Y;
    separation = faceA.y;
    normal = dA.y > 0.0f ? RotA.col2 : B2Vec2Invert(RotA.col2);
  }

  // Box B faces
  if(faceB.x > relativeTol * separation + absoluteTol * hB.x) {
    axis = FACE_B_X;
    separation = faceB.x;
    normal = dB.x > 0.0f ? RotB.col1 : B2Vec2Invert(RotB.col1);
  }

  if(faceB.y > relativeTol * separation + absoluteTol * hB.y) {
    axis = FACE_B_Y;
    separation = faceB.y;
    normal = dB.y > 0.0f ? RotB.col2 : B2Vec2Invert(RotB.col2);
  }

  // Setup clipping plane data based on the separating axis
  B2Vec2 frontNormal, sideNormal;
  B2ClipVertex incidentEdge[2];
  float front, negSide, posSide;
  char negEdge, posEdge;

  memset(incidentEdge, 0, sizeof(incidentEdge));

  // Compute the clipping lines and the line segment to be clipped.
  switch(axis) {
    case FACE_A_X: {
             frontNormal = normal;
             front = B2Vec2Dot(posA, frontNormal) + hA.x;
             sideNormal = RotA.col2;
             float side = B2Vec2Dot(posA, sideNormal);
             negSide = -side + hA.y;
             posSide =  side + hA.y;
             negEdge = EDGE3;
             posEdge = EDGE1;
             ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
           }
           break;

    case FACE_A_Y: {
             frontNormal = normal;
             front = B2Vec2Dot(posA, frontNormal) + hA.y;
             sideNormal = RotA.col1;
             float side = B2Vec2Dot(posA, sideNormal);
             negSide = -side + hA.x;
             posSide =  side + hA.x;
             negEdge = EDGE2;
             posEdge = EDGE4;
             ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
           }
           break;

    case FACE_B_X: {
             frontNormal = B2Vec2Invert(normal);
             front = B2Vec2Dot(posB, frontNormal) + hB.x;
             sideNormal = RotB.col2;
             float side = B2Vec2Dot(posB, sideNormal);
             negSide = -side + hB.y;
             posSide =  side + hB.y;
             negEdge = EDGE3;
             posEdge = EDGE1;
             ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
           }
           break;

    case FACE_B_Y: {
             frontNormal = B2Vec2Invert(normal);
             front = B2Vec2Dot(posB, frontNormal) + hB.y;
             sideNormal = RotB.col1;
             float side = B2Vec2Dot(posB, sideNormal);
             negSide = -side + hB.x;
             posSide =  side + hB.x;
             negEdge = EDGE2;
             posEdge = EDGE4;
             ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
           }
           break;
  }

  // clip other face with 5 box planes (1 face plane, 4 edge planes)

  B2ClipVertex clipPoints1[2];
  B2ClipVertex clipPoints2[2];
  int np;

  memset(clipPoints1, 0, sizeof(clipPoints1));
  memset(clipPoints2, 0, sizeof(clipPoints2));

  // Clip to box side 1
  np = B2ClipSegmentToLine(clipPoints1, incidentEdge, B2Vec2Invert(sideNormal), negSide, negEdge);

  if(np < 2) {
    return 0;
  }

  // Clip to negative box side 1
  np = B2ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

  if(np < 2) {
    return 0;
  }

  // Now clipPoints2 contains the clipping points.
  // Due to roundoff, it is possible that clipping removes all points.

  int numContacts = 0;
  for(int i = 0; i < 2; ++i) {
    separation = B2Vec2Dot(frontNormal, clipPoints2[i].v) - front;

    if(separation <= 0) {
      contacts[numContacts].separation = separation;
      contacts[numContacts].normal = normal;
      // slide contact point onto reference face (easy to cull)
      contacts[numContacts].position = B2Vec2Sub(clipPoints2[i].v, B2Vec2MultF(frontNormal, separation));
      contacts[numContacts].feature = clipPoints2[i].fp;
      if(axis == FACE_B_X || axis == FACE_B_Y) {
        B2FeaturePair fp = contacts[numContacts].feature;
        char tmp = fp.e.inEdge2;
        fp.e.inEdge2 = fp.e.inEdge1;
        fp.e.inEdge1 = fp.e.inEdge2;
        tmp = fp.e.outEdge2;
        fp.e.outEdge2 = fp.e.outEdge1;
        fp.e.outEdge1 = fp.e.outEdge2;
      }
      numContacts++;
    }
  }

  return numContacts;
}


int B2WorldFindArbiter(B2Body* body1, B2Body* body2);
int B2WorldFindFirstArbiterForBody(B2Body* body);
int B2WorldFindFirstJointForBody(B2Body* body);

B2World this_world, *world;

B2Body bodies[NUM_BODIES]; 
B2Joint joints[NUM_JOINTS];
B2Arbiter arbiters[NUM_ARBITERS];


B2World* B2WorldCreate(B2Vec2 gravity, int iterations) {
  world = &this_world;
  memset(world, 0, sizeof(B2World));

  world->gravity = gravity;
  world->iterations = iterations;

  memset(bodies, 0, sizeof(B2Body) * NUM_BODIES);
  memset(joints, 0, sizeof(B2Joint) * NUM_JOINTS);
  world->bodies = bodies;
  world->joints = joints;

  memset(arbiters, 0, sizeof(B2Arbiter) * NUM_ARBITERS);
  world->arbiters = arbiters;

  return world;
}

B2Body* B2WorldAddBody(int index) {
  B2Body *body = &bodies[index];
  B2BodyCreate(body);
  body->index = index;
  body->world = world;
  world->bodies_count++;
  return body;
}

void B2WorldRemoveBody(int index) {
  if (index < 0 || index >= NUM_BODIES) return;
  B2Body *body = &bodies[index];
  body->index = -1;
  body->world = NULL; 

  // Remove all related arbiters
  int i = B2WorldFindFirstArbiterForBody(body);
  while(i != -1) {
    memset(&(world->arbiters[i]), 0, sizeof(B2Arbiter));
    i = B2WorldFindFirstArbiterForBody(body);
  }

  // Remove all related joints
  i = B2WorldFindFirstJointForBody(body);
  while(i != -1) {
    memset(&(world->joints[i]), 0, sizeof(B2Joint));
    i = B2WorldFindFirstJointForBody(body);
  }
  world->bodies_count--;
}

B2Joint* B2WorldAddJoint(int index, B2Body* b1, B2Body* b2, float x, float y) {
  B2Joint *joint = &joints[index];
  B2JointCreate(joint, b1, b2, B2Vec2Make(x, y));
  joint->world = world;
  world->joints_count++;
  return joint;
}

void B2WorldRemoveJoint(int index) {
  B2Joint *joint = &joints[index];
  joint->world = NULL;
  memset(joint, 0, sizeof(B2Joint));
  world->joints_count--;  
}

void B2WorldClear() {
  int i;
  for (i = 0; i < NUM_BODIES; i++) {
    B2WorldRemoveBody(i);
  }
  world->bodies_count = 0;
  world->joints_count = 0;
  world->arbiter_count = 0;
}

inline B2Body* B2WorldGetBody(int i) {
  return &bodies[i];
}

inline B2Joint* B2WorldGetJoint(int i) {
  return &joints[i];
}

int B2WorldFindFirstArbiterForBody(B2Body* body) {
  if(world->arbiter_count == 0) {
    return -1;
  }

  for(int i = 0; i < NUM_ARBITERS; i++) {
    B2Arbiter* arbiter = &arbiters[i];
    if (arbiter->world == NULL) continue;
    if(arbiter->body1 == body || arbiter->body2 == body) {
      return i;
    }
  }

  return -1;
}

int B2WorldFindFirstJointForBody(B2Body* body) {
  if(world->joints_count == 0) {
    return -1;
  }

  for(int i = 0; i < NUM_JOINTS; i++) {
    B2Joint* joint = B2WorldGetJoint(i);
    if (joint->world == NULL) continue;
    if(joint->body1 == body || joint->body2 == body) {
      return i;
    }
  }

  return -1;
}

int B2WorldFindArbiter(B2Body* body1, B2Body* body2) {
  if(world->arbiter_count == 0) {
    return -1;
  }

  B2Body* b1;
  B2Body* b2;
  if(body1 < body2) {
    b1 = body1;
    b2 = body2;
  }
  else {
    b1 = body2;
    b2 = body1;
  }

  for(int i = 0; i < NUM_ARBITERS; i++) {
    B2Arbiter* arbiter = &arbiters[i];
    if (arbiter->world == NULL) continue;
    if(arbiter->body1 == b1 && arbiter->body2 == b2) {
      return i;
    }
  }

  return -1;
}

int B2WorldNumberOfContactsBetweenBodies(B2Body* body1, B2Body* body2) {
  if(world->arbiter_count == 0) {
    return 0;
  }

  B2Body* b1;
  B2Body* b2;
  if(body1 < body2) {
    b1 = body1;
    b2 = body2;
  }
  else {
    b1 = body2;
    b2 = body1;
  }

  for(int i = 0; i < NUM_ARBITERS; i++) {
    B2Arbiter* arbiter = &arbiters[i];
    if (arbiter->world == NULL) continue;
    if(arbiter->body1 == b1 && arbiter->body2 == b2) {
      return arbiter->numContacts;
    }
  }

  return 0;
}

void B2WorldStep(float dt) {
  float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

  // Determine overlapping bodies and update contact points.
  B2WorldBroadphase(world);

  // Integrate forces.
  for(int i = 0; i < NUM_BODIES; ++i) {
    B2Body* b = B2WorldGetBody(i);
    if (b->world == NULL) continue;

    if(b->invMass == 0.0f) {
      continue;
    }

    b->velocity = B2Vec2Add(b->velocity, B2Vec2MultF(B2Vec2Add(world->gravity, B2Vec2MultF(b->force, b->invMass)), dt));
    if (!b->fixedRot) b->angularVelocity += dt * b->invI * b->torque;
  }

  // Perform pre-steps.
  for(int i = 0; i < NUM_ARBITERS; i++) {
    B2Arbiter* arbiter = &arbiters[i];
    if (arbiter->world == NULL) continue;
    B2ArbiterPreStep(arbiter, inv_dt);
  }

  for(int i = 0; i < NUM_JOINTS; i++) {
    B2Joint* joint = B2WorldGetJoint(i);
    if (joint->world == NULL) continue;
    B2JointPreStep(joint, inv_dt);
  }

  // Perform iterations
  for(int i = 0; i < world->iterations; i++) {
    for(int j = 0; j < NUM_ARBITERS; j++) {
      B2Arbiter* arbiter = &arbiters[j];
      if (arbiter->world != NULL) B2ArbiterApplyImpulse(arbiter);
    }

    for(int j = 0; j < NUM_JOINTS; j++) {
      B2Joint* joint = B2WorldGetJoint(j);
      if (joint->world == NULL) continue;
      B2JointApplyImpulse(joint);
    }
  }

  // Integrate Velocities
  for(int i = 0; i < NUM_BODIES; i++) {
    B2Body* b = B2WorldGetBody(i);
    if (b->world == NULL) continue;

    b->position = B2Vec2Add(b->position, B2Vec2MultF(b->velocity, dt));
    if (!b->fixedRot) b->rotation += dt * b->angularVelocity;

    b->force.x = 0.0f;
    b->force.y = 0.0f;
    b->torque = 0.0f;
  }
}

int B2WorldFindAvailableArbiter() {
  for (int i = 0; i < NUM_ARBITERS; i++) {
    if (arbiters[i].world == NULL) {
      return i;
    }
  }
  return -1;
}

void B2WorldBroadphase() {
  B2Arbiter new_arbiter;
  // O(n^2) broad-phase
  for(int i = 0; i < NUM_BODIES; i++) {
    B2Body* bi = B2WorldGetBody(i);
    if (bi->world == NULL) continue;

    for(int j = i + 1; j < NUM_BODIES; j++) {
      B2Body* bj = B2WorldGetBody(j);
      if (bj->world == NULL) continue;

      if(bi->invMass == 0.0f && bj->invMass == 0.0f) {
        continue;
      }

      int existing_arbiter_i = B2WorldFindArbiter(bi, bj);
      if(existing_arbiter_i == -1) {
        int aindex = B2WorldFindAvailableArbiter();
        if (aindex != -1) {
          B2ArbiterCreate(&arbiters[aindex], bi, bj);
          arbiters[aindex].world = world;
          if(arbiters[aindex].numContacts > 0) {
            arbiters[aindex].world = world;
            if (world->arbiter_count < aindex) {
              world->arbiter_count = aindex;
            }
          }
        }
      }
      else {
        B2Arbiter* arb = &arbiters[existing_arbiter_i];
        B2ArbiterCreate(&new_arbiter, bi, bj);
        B2ArbiterUpdate(arb, new_arbiter.contacts, new_arbiter.numContacts);
      }
      if(existing_arbiter_i != -1 && arbiters[existing_arbiter_i].numContacts == 0) {
        arbiters[existing_arbiter_i].world = NULL;
      }
    }
  }
}

#endif

#endif