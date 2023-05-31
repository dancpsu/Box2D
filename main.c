#include <stdio.h>
#include <stdlib.h>
#ifdef TCC
#define _MATH_H_
#define REMATH(x)   double __cdecl x( double f ); float x##f(float v) { return x(v); }

REMATH( cos );
REMATH( sin );
REMATH( tan );
REMATH( sqrt );
#define fabsf(f) f > 0.0f ? f : -f
#else
#include <math.h>
#include <float.h>  
#endif

#include "os_generic.h"

#define CNFG3D
#define CNFG_IMPLEMENTATION
#include "CNFG.h"

#define BOX2D_IMPLEMENTATION
#include "box2d.h"

unsigned frames = 0;
unsigned long iframeno = 0;
B2World *world = NULL;
double Now = 0;

int ox, oy;

void HandleKey( int keycode, int bDown )
{
  if( keycode == 65307 ) exit( 0 );
  printf( "Key: %d -> %d\n", keycode, bDown );
}

void HandleButton( int x, int y, int button, int bDown )
{
  B2Body *box;
  printf( "Button: %d,%d (%d) -> %d\n", x, y, button, bDown );
  if (bDown == 1) {
    ox = x;
    oy = y;
  } else if (bDown == 0) {
    // Add new box
    box = B2WorldAddBody(world->bodies_count + 7);
    B2BodySet(box, B2Vec2Make(0.2, 0.2), 200);
    B2Vec2Set(&box->position, x / world->pixelScale, y / world->pixelScale);
    B2BodyAddForce(box, B2Vec2Make((x - ox) * 1000.0f, (y - oy) * 1000.0f));
  }
}

void HandleMotion( int x, int y, int mask )
{
}

void HandleDestroy()
{
}

void drawBoxes() 
{
  B2Body *box;
  B2Mat22 R;
  B2Vec2 x, h, v1, v2, v3, v4;
  RDPoint points[6];
  for (int i = 0; i < 100; i++) {
    CNFGColor(0x00ffff);
    if (i > 1) CNFGColor(0xffffaa);
    box = B2WorldGetBody(i);
    if (box->world == NULL) continue;
    R = B2Mat22MakeWithAngle(box->rotation);
    x = box->position;
    h = B2Vec2MultF(box->width, 0.5f);
    v1 = B2Vec2Add(x, B2Mat22MultVec(R, B2Vec2Make(-h.x, -h.y)));
    v2 = B2Vec2Add(x, B2Mat22MultVec(R, B2Vec2Make( h.x, -h.y)));
    v3 = B2Vec2Add(x, B2Mat22MultVec(R, B2Vec2Make( h.x,  h.y)));
    v4 = B2Vec2Add(x, B2Mat22MultVec(R, B2Vec2Make(-h.x,  h.y)));
    points[0].x = v1.x * world->pixelScale;
    points[0].y = v1.y * world->pixelScale;
    points[1].x = v3.x * world->pixelScale;
    points[1].y = v3.y * world->pixelScale;
    points[2].x = v2.x * world->pixelScale;
    points[2].y = v2.y * world->pixelScale;
    points[3].x = v1.x * world->pixelScale;
    points[3].y = v1.y * world->pixelScale;
    points[4].x = v4.x * world->pixelScale;
    points[4].y = v4.y * world->pixelScale;
    points[5].x = v3.x * world->pixelScale;
    points[5].y = v3.y * world->pixelScale;
    CNFGTackPoly( points, 6 );
  }
}

void drawJoints() 
{
  for (int i = 0; i < 100; i++) {
    B2Joint *joint = B2WorldGetJoint(i);
    if (joint->world == NULL) continue;
    RDPoint points[2];
    B2Body* b1 = joint->body1;
    B2Body* b2 = joint->body2;

    B2Mat22 R1 = B2Mat22MakeWithAngle(b1->rotation);
    B2Mat22 R2 = B2Mat22MakeWithAngle(b2->rotation);

    B2Vec2 x1 = b1->position;
    B2Vec2 p1 = B2Vec2Add(x1, B2Mat22MultVec(R1, joint->localAnchor1));

    B2Vec2 x2 = b2->position;
    B2Vec2 p2 = B2Vec2Add(x2, B2Mat22MultVec(R2, joint->localAnchor2));
    points[0].x = p1.x * world->pixelScale;
    points[0].y = p1.y * world->pixelScale;
    points[1].x = x2.x * world->pixelScale;
    points[1].y = x2.y * world->pixelScale;
    CNFGColor(0x880088);
    CNFGTackSegment(points[0].x, points[0].y, points[1].x, points[1].y);
  }
}

int main()
{
  int width = 800, height = 600;
  int boxsize, boxes;
  double ThisTime;
  double LastFPSTime = OGGetAbsoluteTime();
  double LastFrameTime = OGGetAbsoluteTime();
  double StartTime = OGGetAbsoluteTime();
  double SecToWait;
  B2Body *floor = NULL;

  CNFGSetup( "Box2D Example", width, height );

  world = B2WorldCreate(B2Vec2Make(0.0, 9.81), 10);
  world->pixelScale = (float)(width)/5.5f;
  
  // Add floor
  floor = B2WorldAddBody(0);
  B2BodySet(floor, B2Vec2Make(24.0, 0.5), FLT_MAX);
  B2Vec2Set(&floor->position, 2.5, 4.0);
  floor->friction = 1.0f;

  // Add ceiling
  floor = B2WorldAddBody(1);
  B2BodySet(floor, B2Vec2Make(24.0, 0.5), FLT_MAX);
  B2Vec2Set(&floor->position, 2.5, 0.0);
  floor->friction = 1.0f;
  
  // Add pendulum
  B2Body *platform = B2WorldAddBody(2);
  B2BodySet(platform, B2Vec2Make(1.0, 0.2), 100);
  B2Vec2Set(&platform->position, 2.5, 1.5);
  B2WorldAddJoint(0, floor, platform, 2.5, 0.1);

  // Add boxes
  boxes = 36;
  for (int i = 0; i < boxes; i++) {
    boxsize = i % 4 > 1 ? 1 : 0;
    B2Body *box = B2WorldAddBody(i + 3);
    B2Vec2 size = B2Vec2Make(0.2 + (0.5 * (float)boxsize), 0.2);
    B2BodySet(box, size, 200 - ((float)(i / 5) * 15.0));
    B2Vec2Set(&box->position, 0.5 * (i % 6) + 1.5f + (0.2 * boxsize), 3.6 - (0.2 * (float)(i / 6)));
    printf("Box %d position: (%.2f, %.2f)\n", i+1, box->position.x, box->position.y);
    box->friction = 1.0;
    i += boxsize;
  } 
  
  while(1)
  {
	iframeno++;

    if (iframeno % 5 == 0) CNFGHandleInput();

    CNFGClearFrame();

    drawBoxes();
    drawJoints();

    ThisTime = OGGetAbsoluteTime();
    if (ThisTime > StartTime + 1.2f) {
      B2WorldStep(1.0/100.0);
    }
    for (int i = 0; i < 100; i++) {
      B2Body *b = B2WorldGetBody(i);
      if (b->world != NULL) {
        if(b->position.y > 6.0f) {
          B2WorldRemoveBody(b->index); // Remove body if outside the world
        }
      }
    }

    frames++;
    CNFGSwapBuffers();

    if( ThisTime > LastFPSTime + 1 )
    {
      frames = 0;
      LastFPSTime+=1;
    }

    SecToWait = .016 - ( ThisTime - LastFrameTime );
    LastFrameTime += .016;
    if( SecToWait > 0 )
      OGUSleep( (int)( SecToWait * 1000000 ) );
  }

  return(0);
}

