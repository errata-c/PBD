## Engine parameters:

Gravity, 3 float

Time step, 1 float

Substeps, 1 integer

Friction, 1 float (Can be made per-particle)



## Structural Constraints:

Distance, 2 integers, 1 float

Tetrahedral Volume, 4 integers, 1 float



## Collision Constraints:

Plane, origin 3 float, normal 3 float.

Particle, 2 integers, 1 float.

## Particle data:

Position 3 float, previous position 3 float, velocity 3 float, inverse mass 1 float.



## Collision detection:

Spatial hashing works extremely well for particles of the same size, so long as the world space is subdivided properly. However, it does not handle particles of variable size at all.

Spatial hash table starts by subdividing space into a number of cells. We can choose a number of different methods for implementing the hashing operation. One option is to hash the combined spatial coordinates, and get what's in that cell. This doesn't solve the issue of having variable particle sizes.

Another option is to skip hash tables, and just use a BVH. Intel Embree might be a good option.





# Todo:

- [ ] Design a simple library for R/W of constraints and particles
- [ ] Add collision groups
- [ ] Implement a faster collision detection
- [ ] 