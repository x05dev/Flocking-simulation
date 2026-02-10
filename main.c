/* Flocking simulation following The Coding Train's Coding challenge #124*/

#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <stdio.h>
#include "boid.h"

#define WPS SDL_WINDOWPOS_CENTERED
#define W 1200
#define H 900
#define FPS 60
#define MSFPS 1000/FPS
#define NUM_BOIDS 500

int main(void)
{
  char state = 1;
  boid boids[NUM_BOIDS];
  boid_init(boids, NUM_BOIDS, W, H);
  
  if(SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "SDL init error: %s\n", SDL_GetError());
    return 1;
  }
  SDL_Window *win = SDL_CreateWindow("flocking simulation", WPS, WPS, W, H, 0);
  if(!win) {
    fprintf(stderr, "SDL window creation error");
    return 1;
  }

  SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
  if(!ren) {
    fprintf(stderr, "SDL renderer creation error");
    return 1;
  }

  SDL_Event eve;

  while(state)
  {
    Uint32 start_frame = SDL_GetTicks();
    //Key input
    while(SDL_PollEvent(&eve))
    {
      switch(eve.type) {
        case SDL_KEYDOWN:
          switch(eve.key.keysym.sym) {
            case SDLK_ESCAPE:
              state = 0;
            default:
              break;
          }
        default:
          break;
      }
    }


    if(eve.type == SDL_QUIT) state = 0;
    SDL_SetRenderDrawColor(ren, 10, 10, 10, 255);
    SDL_RenderClear(ren);

    boid_update(boids, NUM_BOIDS, W, H);
    boid_draw(boids, NUM_BOIDS, ren);

    SDL_RenderPresent(ren);    

    Uint32 frame_time = SDL_GetTicks() - start_frame;
    if(frame_time < MSFPS) {
      SDL_Delay(MSFPS-frame_time);
    }
  }

  SDL_DestroyWindow(win);
  SDL_DestroyRenderer(ren);
  SDL_Quit();
  return 0;
}
