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

It may be possible to implement rigid body physics for larger objects (like boxes, capsules, meshes) by emulating multiple particles. Compute center of mass of object, create tetrahedron of virtual particles around the origin. Then in theory all we need to do is compute a limited set of constraints to get the emergent behavior.

It might be worthwhile to give the particles and other collision objects their own friction values. The number of values we have to store per-particle is already pretty high, but I don't think running out of memory is the biggest concern facing the performance.

Damping could also be added into the mix. We have to option of implementing multiple forms of the constraints such that something like damping can be optional.

Compliance could be added to the collision constraints as well, if desired. This would in effect make the collisions a bit softer. That could be a desirable effect. Just like the damping, this could also be made optional.

XPBD makes mention of the Lagrange multiplier being cumulative over the timesteps. It seems like they are talking about having the multiplier update just like the position update, only this time just for the specific constraints and not the particles. I don't fully know what the benefit of this is, as I haven't seen an implementation of it anywhere. Additionally, the implementations without it seem to work fairly well. I may create a test branch later to test this out and see how well it performs, and if its worth the additional memory overhead.

We need a way to allow the users to apply external forces on a per-particle basis.

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