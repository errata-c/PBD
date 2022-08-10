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

Collisions are bidirectional, we only have to process each pairing once. Additionally we need to search a small area around each particle, not just the particle itself. 

One implementation breaks everything into blocks, then creates a secondary hash. For each block, provide an entry in the hash table. For each entry, provide a list of object ids. 

## Other things:

Objects added to the scene will have a consecutive set of particles and constraints. Creation and deletion of objects will require changing the particle and constraint ordering. This will invalidate any indices the particles originally had. A mapping is needed to allow for rearrangement. Additionally, we need the constraints to be contained in a single location, instead of in several different ones. 

It may be possible to implement rigid body physics for larger objects (like boxes, capsules, meshes) by emulating multiple particles. Compute center of mass of object, create tetrahedron of virtual particles around the origin. Wherever a collision occurs,  treat that collision as collision with a particle with mass of object over distance from object origin. Then apply a distance constraint between origin  

# Todo:

- [ ] Design a simple library for R/W of physics objects.
- [ ] Add collision groups
- [ ] Add compliance to constraints
- [x] Make it easier to add constraints to the engine.
- [ ] Make it easier to add particles to the engine
- [ ] Add object mapping that updates when particles or constraints are removed
- [x] Implement spatial hashing for collision detection
- [ ] Find collisions using the spatial hashing, add them to a second dynamic constraint set 
- [x] Implement rotational extraction
- [ ] Implement rotational tracking for skeletal animation of deformations