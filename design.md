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

It may be possible to implement rigid body physics for larger objects (like boxes, capsules, meshes) by emulating multiple particles. Compute center of mass of object, create tetrahedron of virtual particles around the origin. Then in theory all we need to do is compute a limited set of constraints to get the emergent behavior.

It might be worthwhile to give the particles and other collision objects their own friction values. The number of values we have to store per-particle is already pretty high, but I don't think running out of memory is the biggest concern facing the performance.

Damping could also be added into the mix. We have to option of implementing multiple forms of the constraints such that something like damping can be optional. As of right now, it does not seem necessary.

Compliance could be added to the collision constraints as well, if desired. This would in effect make the collisions a bit softer. That could be a desirable effect. Just like the damping, this could also be made optional.

XPBD makes mention of the Lagrange multiplier being accumulated over multiple constraint iterations. This is only needed when we process each constraint multiple times per substep. I might try this in another branch to see if it actually improves performance or accuracy.

# Todo:

- [ ] Design a simple library for R/W of physics objects.
- [ ] Add collision groups
- [x] Add compliance to constraints
- [x] Make it easier to add constraints to the engine.
- [x] Make it easier to add particles to the engine
- [x] Provide a mechanism to add forces to particles
- [x] Implement constraint variant, store all constraints in single buffer
- [x] Provide method to reorder constraint ids
- [ ] Implement a reusable constraint list
  - [x] Constraint references
  - [ ] Constraint id array access
  - [ ] `iterator` class
  - [ ] `const_iterator` class
  - [ ] erase method for ranges of constraints with remapping
  - [ ] Combined erase method for efficiently erasing multiple ranges of constraints?
  - [ ] Append from another constraint list
- [ ] Implement a particle prefab class, containing a list of particles and constraints
  - [ ] Simple particle object, position, velocity, inverse mass, collision flags, radius.
  - [ ] Constraint list, local ids
- [ ] Implement a rotation tracker class, containing the list of ids to extract rotations from
- [x] Implement spatial hashing for collision detection
- [ ] Find collisions using the spatial hashing, add them to a second dynamic constraint set 
- [x] Implement rotational extraction methods
- [ ] Object map with ID generation
  - [ ] ID type, 64 bits
  - [ ] Mapping from ID to particle range and constraint range
  - [ ] Update ID mapping as needed
  - [ ] Creating new IDs as needed