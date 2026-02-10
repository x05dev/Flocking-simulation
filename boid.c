#include <SDL2/SDL2_gfxPrimitives.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "boid.h"

#define PERCEPTION_RADIUS 35
#define MAX_SPEED 3.0
#define MIN_SPEED 1.0
#define MAX_FORCE 0.12
#define MAX_SEPARATION_FORCE 0.2

tri_vect2 boid_behaviour(boid boids[], int size, int k);
void edges(boid *bd, int W, int H);

void boid_update(boid boids[], int size, int W, int H)
{
  for(int i = 0; i < size; i++) {
    edges(&boids[i], W, H);

    tri_vect2 main = boid_behaviour(boids, size, i);

    vect2 steering = {
      main.alignment.x + main.cohesion.x + main.separation.x, 
      main.alignment.y + main.cohesion.y + main.separation.y};

    boids[i].acc.x = steering.x;
    boids[i].acc.y = steering.y;

    boids[i].vel.x += boids[i].acc.x;
    boids[i].vel.y += boids[i].acc.y;

    float speed = sqrt(boids[i].vel.x * boids[i].vel.x + boids[i].vel.y * boids[i].vel.y);
    if (speed > MAX_SPEED) {
      boids[i].vel.x = (boids[i].vel.x / speed) * MAX_SPEED;
      boids[i].vel.y = (boids[i].vel.y / speed) * MAX_SPEED;
    }else if(speed < MIN_SPEED && speed > 0){
      boids[i].vel.x = (boids[i].vel.x / speed) * MAX_SPEED;
      boids[i].vel.y = (boids[i].vel.y / speed) * MAX_SPEED;
    }

    boids[i].pos.x += boids[i].vel.x;
    boids[i].pos.y += boids[i].vel.y;

    boids[i].acc.x = 0.0;
    boids[i].acc.y = 0.0;
  }
}

void boid_draw(boid boids[], int size, SDL_Renderer *ren)
{
  for(int i = 0; i < size; i++) {
    float theta = atan2(boids[i].vel.y, boids[i].vel.x);
    float cost = cos(theta);
    float sint = sin(theta);

    vect2 p1 = {-3, -3};
    vect2 p2 = {-3, 3};
    vect2 p3 = {6, 0};

    vect2 np1, np2, np3;

    np1.x = (p1.x * cost - p1.y * sint) + boids[i].pos.x;
    np1.y = (p1.x * sint + p1.y * cost) + boids[i].pos.y;

    np2.x = (p2.x * cost - p2.y * sint) + boids[i].pos.x;
    np2.y = (p2.x * sint + p2.y * cost) + boids[i].pos.y;

    np3.x = (p3.x * cost - p3.y * sint) + boids[i].pos.x;
    np3.y = (p3.x * sint + p3.y * cost) + boids[i].pos.y;

    trigonRGBA(ren, np1.x, np1.y, np2.x, np2.y, np3.x, np3.y, 200, 200, 200, 255);
  }
}

void boid_init(boid boids[], int size, int W, int H) {
  srand(time(NULL));
  for(int i = 0; i < size; i++) {
    boids[i].pos.x = (float)(rand() % W);
    boids[i].pos.y = (float)(rand() % H);


    boids[i].vel.x = pow(-1.0, i) * MAX_SPEED*2 / ((float)(rand() % 30) + 1);
    boids[i].vel.y = pow(-1.0, i) * MAX_SPEED*2 / ((float)(rand() % 30) + 1);
  }
}


//local functions
void vect2_set(vect2 *v, float x);

tri_vect2 boid_behaviour(boid boids[], int size, int k) {
  tri_vect2 main;
  vect2_set(&main.separation, 0.0);
  vect2 diff;
  vect2_set(&main.alignment, 0.0);
  vect2_set(&main.cohesion, 0.0);

  int count = 0;

  for(int j = 0; j < size; j++) {
    float d = sqrt(pow(boids[j].pos.x - boids[k].pos.x,2) + pow(boids[j].pos.y - boids[k].pos.y,2));
    if(j!=k && d < PERCEPTION_RADIUS) {
      //separation
      diff.x = boids[k].pos.x - boids[j].pos.x;
      diff.y = boids[k].pos.y - boids[j].pos.y;

      diff.x /= d;
      diff.y /= d;

      main.separation.x += diff.x;
      main.separation.y += diff.y;
      //cohesion
      main.cohesion.x += boids[j].pos.x;
      main.cohesion.y += boids[j].pos.y;
      //alignment
      main.alignment.x += boids[j].vel.x;
      main.alignment.y += boids[j].vel.y;

      count++;
    }
  }
  if(count > 0) {
    //separation
    main.separation.x /= count;
    main.separation.y /= count;
    float separation_mag = 
      sqrt(main.separation.x * main.separation.x + main.separation.y * main.separation.y); 
    if(separation_mag > 0) {
      main.separation.x = (main.separation.x/separation_mag) * MAX_SPEED;
      main.separation.y = (main.separation.y/separation_mag) * MAX_SPEED;
    }
    main.separation.x -= boids[k].vel.x;
    main.separation.y -= boids[k].vel.y;
    float separation_steer_mag = 
      sqrt(main.separation.x * main.separation.x + main.separation.y * main.separation.y); 
    if(separation_steer_mag > MAX_SEPARATION_FORCE) {
      main.separation.x = (main.separation.x/separation_steer_mag) * MAX_SEPARATION_FORCE;
      main.separation.y = (main.separation.y/separation_steer_mag) * MAX_SEPARATION_FORCE;
    }
    //cohesion
    main.cohesion.x /= count;
    main.cohesion.y /= count;
    main.cohesion.x -= boids[k].pos.x;
    main.cohesion.y -= boids[k].pos.y;
    float cohesion_mag = 
      sqrt(main.cohesion.x * main.cohesion.x + main.cohesion.y * main.cohesion.y); 
    if(cohesion_mag > 0) {
      main.cohesion.x = (main.cohesion.x/cohesion_mag) * MAX_SPEED;
      main.cohesion.y = (main.cohesion.y/cohesion_mag) * MAX_SPEED;
    }
    float cohesion_steer_mag = 
      sqrt(main.cohesion.x * main.cohesion.x + main.cohesion.y * main.cohesion.y); 
    if(cohesion_steer_mag > MAX_FORCE) {
      main.cohesion.x = (main.cohesion.x/cohesion_steer_mag) * MAX_FORCE;
      main.cohesion.y = (main.cohesion.y/cohesion_steer_mag) * MAX_FORCE;
    }
    //alignment
    main.alignment.x /= count;
    main.alignment.y /= count;
    float alignment_mag = 
      sqrt(main.alignment.x * main.alignment.x + main.alignment.y * main.alignment.y); 
    if(alignment_mag > 0) {
      main.alignment.x = (main.alignment.x/alignment_mag) * MAX_SPEED;
      main.alignment.y = (main.alignment.y/alignment_mag) * MAX_SPEED;
    }
    main.alignment.x -= boids[k].vel.x;
    main.alignment.y -= boids[k].vel.y;
    float alignment_steer_mag = 
      sqrt(main.alignment.x * main.alignment.x + main.alignment.y * main.alignment.y); 
    if(alignment_steer_mag > MAX_FORCE) {
      main.alignment.x = (main.alignment.x/alignment_steer_mag) * MAX_FORCE;
      main.alignment.y = (main.alignment.y/alignment_steer_mag) * MAX_FORCE;
    }
  }
  return main;
} 

void edges(boid *bd, int W, int H) 
{
  if(bd->pos.x > W) {
    bd->pos.x = 0.0;
  }else if(bd->pos.x < 0.0) {
    bd->pos.x = W;
  }

  if(bd->pos.y > H) {
    bd->pos.y = 0.0;
  }else if(bd->pos.y < 0.0) {
    bd->pos.y = H;
  }
}

void vect2_set(vect2 *v, float x) {
  v->x = x;
  v->y = x;
}










