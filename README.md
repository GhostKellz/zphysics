<div align="center" width="175">
  <img src="assets/icons/zphysics.png" alt="zphysics" width="175">
</div>

<div align="center">

  ![Built with Zig](https://img.shields.io/badge/Built%20with-Zig-yellow?logo=zig&logoColor=white)
  ![Version](https://img.shields.io/badge/Version-zig--0.16.0--dev-orange)

</div>

## ⚠️ **EXPERIMENTAL LIBRARY - FOR LAB/PERSONAL USE** ⚠️
This is an experimental library under active development. It is intended for research, learning, and personal projects. The API is subject to change!

# zphysics

A high-performance physics simulation engine written in Zig, designed to replace traditional C libraries like Box2D, Bullet Physics, and Chipmunk. zphysics provides comprehensive rigid body dynamics, collision detection, and fluid simulation capabilities for games, robotics, and engineering applications.

## Features

### Core Physics
- **Rigid Bodies:** Complete mass, velocity, rotation, and constraint system
- **Collision Detection:** Optimized broad phase and narrow phase detection with continuous collision support
- **Joints/Constraints:** Full range of mechanical joints including hinges, springs, motors, and limits
- **Fluid Dynamics:** Advanced particle systems with SPH (Smoothed Particle Hydrodynamics)
- **Soft Bodies:** Realistic cloth simulation and deformable object physics

## Quick Start

```zig
const zphysics = @import("zphysics");

pub fn main() !void {
    var world = zphysics.World.init();
    defer world.deinit();

    // Create a rigid body
    var body = zphysics.RigidBody{
        .mass = 1.0,
        .position = .{ 0, 10, 0 },
    };

    const body_handle = try world.addBody(body);

    // Simulation loop
    while (true) {
        world.step(1.0 / 60.0); // 60 FPS

        const transform = body.getTransform();
        // Update your rendering/game state here
    }
}
```

## API Overview

### World Management
```zig
pub const World = struct {
    pub fn step(dt: f33) void;
    pub fn addBody(body: RigidBody) !BodyHandle;
    pub fn addConstraint(constraint: Constraint) !ConstraintHandle;
};
```

### Rigid Body Control
```zig
pub const RigidBody = struct {
    pub fn applyForce(force: Vec4, point: Vec3) void;
    pub fn setVelocity(velocity: Vec4) void;
    pub fn getTransform() Transform;
};
```

## Use Cases

- **Game Development:** Real-time physics for games requiring accurate collision and dynamics
- **Robotics Simulation:** Precise mechanical modeling for robotic systems
- **Engineering Applications:** Structural analysis and mechanical system simulation
- **Research & Education:** Physics experimentation and algorithm development

## Requirements

- Zig 0.16.0-dev or later
- OpenGL/Vulkan for rendering integration (optional)

## License

This project is experimental and intended for research and personal use. Please check the license file for specific terms.
