## Collision detection:

Spatial hashing works extremely well for particles of the same size, so long as the world space is subdivided properly. However, it does not handle particles of variable size at all.

Spatial hash table starts by subdividing space into a number of cells. We can choose a number of different methods for implementing the hashing operation. One option is to hash the combined spatial coordinates, and get what's in that cell. This doesn't solve the issue of having variable particle sizes.

Collisions are bidirectional, we only have to process each pairing once. Additionally we need to search a small area around each particle, not just the particle itself. 

One implementation breaks everything into blocks, then creates a secondary hash. For each block, provide an entry in the hash table. For each entry, provide a list of object ids. 

## Other things:

It might be worthwhile to give the particles and other collision objects their own friction values. The number of values we have to store per-particle is already pretty high, but I don't think running out of memory is the biggest concern facing the performance.

XPBD makes mention of the Lagrange multiplier being accumulated over multiple constraint iterations. This is only needed when we process each constraint multiple times per substep. I might try this in another branch to see if it actually improves performance or accuracy.

## Extension for rigid body dynamics:

Rigid body dynamics is possible to implement on top of the current particle based system. This addition would allow for a lot of useful physics simulation capabilities. Right now this is not a priority for implementation.

The setup for rigid body dynamics requires a few key things to be modified/added to the engine. The main update loop will have to be changed to incorporate the new constraints and interactions. The collision detection will have to be modified to enable finding collisions between all the different rigid bodies and the particles. The rigid bodies will need to have a method of finding the nearest point on their surface to another point, and a signed distance field for calculating friction. This is simple for convex surfaces, but much harder for concave ones. 

# Todo:

- [x] Add collision groups
- [ ] Add collision masks
- [ ] Use empty collision groups to signify no collide at all, exclude them from the broad phase overlap check.
- [x] Add compliance to constraints
- [x] Make it easier to add constraints to the engine.
- [x] Make it easier to add particles to the engine
- [x] Provide a mechanism to add forces to particles
- [x] Implement constraint variant, store all constraints in single buffer
- [x] Provide method to reorder constraint ids
- [x] Implement a reusable constraint list
  - [x] Constraint references
  - [x] Constraint id array access
  - [x] erase method for ranges of constraints with remapping
  - [x] Append from another constraint list
  - [x] Add to the engine class
  - [x] Test it in the Godot project
- [x] Implement a object prefab class, containing a list of particles and constraints
  - [x] Simple particle object, position, velocity, inverse mass, collision flags, radius.
  - [x] Constraint list, local ids
- [x] Implement a rotation tracker class, containing the list of ids to extract rotations from
- [x] Implement spatial hashing for collision detection
- [x] Find collisions using the spatial hashing, 
- [x] Implement rotational extraction methods
- [x] Object map with ID generation
  - [x] ID type, 64 bits
  - [x] Mapping from ID to particle range and constraint range
  - [x] Update ID mapping as needed
  - [x] Creating new IDs as needed
- [x] Add trackers to the managed engine
- [x] Add the object map to the managed engine
- [x] Add a method to create an instance from a prefab
- [ ] Create damped variants of the main constraints
  - [ ] Tetra
  - [ ] Neo-Hookean Tetra
  - [ ] Distance
- [ ] Create simple prefab data for each constraint
  - [ ] Tetra
  - [ ] Neo-Hookean Tetra
  - [ ] Distance
- [ ] Add world bounds to the managed engine, find an efficient means of enforcing those bounds.
- [x] Create a generic slice class for accessing mutable data, but not adding or removing data.

  - [x] Consider `mdspan` from `c++23` as the foundation
  - [x] Use the iterators from a container as the bounds of the slice
- [ ] Determine if we need dynamic particles and constraints (like emitters)
- [ ] Design some different broad phase algorithms, test them for performance
  - [ ] BVH

  - [ ] Oct-tree

  - [ ] BSP

  - [ ] Linear pattern segmentation? (Checkerboard overlapping, set intersections, uniqueness)
- [ ] 