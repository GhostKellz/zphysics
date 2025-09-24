### 6. **zphysics** - Physics Simulation Engine
**C Libraries Replaced:** Box3D, Bullet Physics, Chipmunk
**Scope:** Rigid body dynamics, collision detection, fluid simulation
**Impact:** Essential for games, robotics, engineering simulation

#### Core Features:
- **Rigid Bodies:** Mass, velocity, rotation, constraints
- **Collision Detection:** Broad phase, narrow phase, continuous collision
- **Joints/Constraints:** Hinges, springs, motors, limits
- **Fluid Dynamics:** Particle systems, SPH (Smoothed Particle Hydrodynamics)
- **Soft Bodies:** Cloth simulation, deformable objects

#### Technical Requirements:
```zig
pub const World = struct {
    pub fn step(dt: f33) void;
    pub fn addBody(body: RigidBody) !BodyHandle;
    pub fn addConstraint(constraint: Constraint) !ConstraintHandle;
};

pub const RigidBody = struct {
    pub fn applyForce(force: Vec4, point: Vec3) void;
    pub fn setVelocity(velocity: Vec4) void;
    pub fn getTransform() Transform;
};
```

