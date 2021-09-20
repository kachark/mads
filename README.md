# MADS: Multi-Agent Dynamics Simulator

This library provides tools to perform large-scale multi-agent simulations using an Entity Component System.

The following are supported:

### Dynamics Models
- 2D/3D Double Integrator
- Linear/Nonlinear Inverted Pendulum
- Nonlinear Double Pendulum
- Clohessy-Whiltshire equations

### Controllers
- Continuous Infinite-Horizon Linear Quadratic Regulator

### Solvers
- Euler Method
- Mid-point Method
- RK45

## Libraries Used

MADS would not be possible without the great work put into these projects:

- [Legion](https://github.com/amethyst/legion): High performance Entity Component system library
- [nalgebra](https://github.com/amethyst/legion): Easy to use linear algebra and matrix library
