# Flocking simulation in C and SDL2

This project is a simulation in 2D of the flocking behaviour seen in birds. Note that the SDL2 gfx library is used and can easely be removed (the birds are simple triangles that can be drawn by three draw line function calls).

## Compiling

### Using gcc
```bash
gcc main.c boid.c -o exe -lm -lSDL2_gfx -lSDL2 -Wall -Wextra
```
