## Joint types and methods to create them:

There are a number of different kinds of rigid body joints that can be created. The most important ones are going to be the hinge joint and the sphere joint. Additionally, the joints need to be capable of having limits of some kind. Instead of implementing a new class for each kind of joint/limit combination, it would be better to create a more dynamic setup. Have a basic joint constraint class, then have the secondary components follow directly after in memory. Those secondary components will be referenced and driven by the base class, a sentinel value will be present to signify that the secondary components are present. The base class and secondary components will be allocated together as a unit.

The secondary components will be a angle limiter or an angle driver. The angle driver pushes the angle of the base constraint towards some target. The sentinel values will be a single bit field to signify the presence or absence of secondary components. The components will always appear in the order they are referred to in the bit field. We will add a special method to the constraint list to allow for easy creation of joints and secondary components.

Alternatively, the constraint type enumeration can be augmented to act in place of the bit field. This would mean that the base class would not need to have a bit field in it at all. This would have the advantage of being simpler internally, but it would mean the data for the class was less encapsulated. This I think makes it an inferior solution.

The final alternative is the simplest, just include all the data in the base class directly, and only use it if they are enabled. This is the most wasteful in terms of memory, but the safest and simplest in terms of implementation.

Since these joints are composed of a fair amount of complex information, constructing them is a little difficult.

## Body types:

- Oriented Bounding Box (OBB)
- Capsule
- Cylinder

- Sphere

## Collision algorithms:

Each collision calculation must return four things: A contact point on both bodies, a contact normal, and a contact depth. The contact depth will be used to calculate the friction, the contact normal and the contact points will be used to resolve the collision by updating the orientation and position of the bodies.

**OBB to OBB collision**

```c++
// Matrix is COLUMN major
// Thus, mat[0] is the first column

int OBB_OBB_collide(
    const vec3 & p1, const mat3 & R1, const vec3 & side1,
    const vec3 & p2, const mat3 & R2, const vec3 & side2,
    
) {
    /*
    Use the separating axis theorem to detect collision.
    There are 15 possible axes to check.
    */
    
    real_t fudge_factor(1.05);
    vec3 p, pp, normalC(0.f);
    real_t * normalR = nullptr;
    real_t A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33,
		Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
   	
    p = p2 - p1;
    // Interpret p as row vector, matmul
    pp = p * R1;
    
    for(int i : iter::range(3)) {
        A[i] = side1[i] * real_t(0.5);
    }
    for(int i : iter::range(3)) {
        B[i] = side2[i] * real_t(0.5);
    }
    
    R11 = dot(R1[0], R2[0]);
    R12 = dot(R1[0], R2[1]);
    R13 = dot(R1[0], R2[2]);
    R21 = dot(R1[1], R2[0]);
    R22 = dot(R1[1], R2[1]);
    R23 = dot(R1[1], R2[2]);
    R31 = dot(R1[2], R2[0]);
    R32 = dot(R1[2], R2[1]);
    R33 = dot(R1[2], R2[2]);
    
    Q11 = abs(R11);
	Q12 = abs(R12);
	Q13 = abs(R13);
	Q21 = abs(R21);
	Q22 = abs(R22);
	Q23 = abs(R23);
	Q31 = abs(R31);
	Q32 = abs(R32);
	Q33 = abs(R33);
    
    // For each separating axis
	//   If separate, early exit
	//   else find depth
	//   track max depth
	// normal from minimum depth axis
	// note: normalR is set to point to a column of R1 or R2 if that is
	// the smallest depth normal so far. otherwise normalR is 0 and normalC is
	// set to a vector relative to body 1. invert_normal is 1 if the sign of
	// the normal should be flipped.
    
}
```

**Capsule to Capsule collision**

```c++
```

**Capsule to OBB collision**

```

```

**Cylinder to Capsule collision**

```

```

**Cylinder to Cylinder collision**

```

```

**Any rigid body to Particle (Sphere) collision**

```c++
// Simply check a distance field.
// Rotate sphere origin into rigid body space
// Calculate distance of sphere origin to the rigid body's surface
// If distance is greater than the sphere radius, no collision
```