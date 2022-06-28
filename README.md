# Physim

PhySim is a research-oriented physics simulation library in C++. 

This has been forever in development, working on a final pre-sharing clean-up now :)

Current key features:

- **Extensible and generic** definition of **mechanical models** based on kinematic and energy components

- Modeling complex kinematics and couplings using **kinematic trees**, with automatic derivative propagation

- Common interface for static and dynamic problems, with a **unified numerical solver** based on optimization

- A bunch of **ready-to-use mechanical models**: nonlinear FEM, DER, thin-shells, and articulated soft-bodies

- A geometry library with a unified interface for curves, surfaces, and volumetric **meshes with customizable traits**

- Easy-to-use system for the **definition of boundary conditions** with programmable incremental loading



Currently working on:

- Unified interface for **inverse design** of model parameters

- More robust collision detection and response system

- Proper documentation and user interface :)
