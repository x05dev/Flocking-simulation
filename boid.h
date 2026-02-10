typedef struct {
  float x, y;
}vect2;

typedef struct {
  vect2 pos;
  vect2 vel;
  vect2 acc; 
}boid;

typedef struct {
  vect2 separation;
  vect2 alignment;
  vect2 cohesion;
}tri_vect2;

void boid_init(boid boids[], int size, int W, int H);
void boid_update(boid boids[], int size, int W, int H);
void boid_draw(boid boids[], int size, SDL_Renderer *ren);
