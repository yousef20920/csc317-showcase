# Title: Interactive Jello Soft Body Physics

## Personal Information
**Name:** Yousef Abdelhadi
**UtorID:** abdel192
**Student Number:** [Your Student Number]
**Assignment Augmented:** A8 (Mass-Spring Systems) and A6 (Shader Pipeline - for lighting/materials)

## Instructions
1.  **Build:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make masssprings_showcase
    ```
2.  **Run:**
    ```bash
    ./masssprings_showcase
    ```
3.  **Controls:**
    *   **SPACE**: Jump! (Launch jello into the air)
    *   **Click & Drag**: Grab and pull the jello vertices (Area Grab)
    *   **1**: Material Preset: Raspberry Jello (Balanced)
    *   **2**: Material Preset: Super Slime (High bounce, low friction)
    *   **3**: Material Preset: Heavy Dough (Stiff, high damping)
    *   **4**: Material Preset: Basketball (Sphere, Rigid, Bouncy)
    *   **T**: Toggle Slow Motion (Matrix style)
    *   **S**: Toggle Stress Visualization
    *   **R**: Reset Simulation

## Description
This showcase implements a **3D Volumetric Soft Body Simulation** using a mass-spring system. Unlike the 2D cloth in A8, this project simulates a solid 3D object with internal structure, volume preservation, and complex physical interactions.

### Features Implemented:
1.  **3D Volumetric Mesh Generation**
    *   **Code Location:** `src/JelloMesh.cpp` (`create_jello_cube`, `create_jello_sphere`)
    *   **Details:** Generates a 3D grid of vertices (6x6x6). Constructs a complex network of structural springs (x, y, z axes), shear springs (diagonals), and **Bending Springs** (2-hop connections) to maintain volume and shape.
    *   **Dynamic Mesh Switching:** Supports real-time switching between Cube and Sphere topologies (in `JelloState::set_material`).

2.  **Robust Physics Engine**
    *   **Code Location:** `src/fast_mass_springs_step_sparse.cpp`
    *   **Details:** Uses a Local-Global solver with **100-1000 iterations** (adaptive based on material) for rock-solid stability even with extreme stiffness.
    *   **Safety:** Implemented velocity clamping and air resistance in `main_showcase.cpp` (pre-draw loop) to prevent mesh inversion.

3.  **Interactive Materials**
    *   **Code Location:** `include/JelloState.h` (`set_material`)
    *   **Details:** Dynamic material system that updates physics parameters (`k`, `damping`, `restitution`, `friction`) in real-time.
    *   **Presets:** Jello (Balanced), Slime (Bouncy), Dough (Heavy), Basketball (Rigid Sphere).

4.  **Advanced Mouse Interaction (Area Grabbing)**
    *   **Code Location:** `main_showcase.cpp` (`callback_mouse_down`, `callback_mouse_move`)
    *   **Details:** Implemented **Area Grabbing** where the user grabs a cluster of vertices within a radius instead of a single point. This creates a rigid "chunk" hold. Includes velocity transfer for throwing.

5.  **Visual Polish**
    *   **Code Location:** `main_showcase.cpp` (Viewer setup)
    *   **Details:** Translucent rendering with alpha blending. **3D Ground Platform** generation (volumetric stage). Stress visualization mapping spring deformation to color.

## Acknowledgements
*   **libigl**: For geometry processing, viewer, and project/unproject functions.
*   **Eigen**: For linear algebra operations.
*   **CSC317 Course Materials**: Base mass-spring system theory (A8) and Shader concepts (A6).
