# Title: Interactive Jello Soft Body Physics

## Personal Information
**Name:** Yousef Abdelhadi
**UtorID:** abdel192
**Assignment Augmented:** A8 (Mass-Spring Systems) and A6 (Shader Pipeline - for lighting/materials)

## Requirements
- CMake 3.20 or higher
- C++17 compatible compiler (Clang, GCC, or MSVC)
- OpenGL support

### macOS
```bash
# Install Xcode Command Line Tools (if not already installed)
xcode-select --install

# Install CMake via Homebrew (if not already installed)
brew install cmake
```

### Ubuntu/Debian
```bash
sudo apt update
sudo apt install cmake build-essential libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

### Windows
- Install [Visual Studio](https://visualstudio.microsoft.com/) with C++ development tools
- Install [CMake](https://cmake.org/download/)

## Quick Setup

All dependencies (libigl, Eigen, GLFW, etc.) are **automatically downloaded** via CMake. No manual installation required!

```bash
# Clone the repository
git clone <repository-url>
cd csc317-showcase

# Create build directory and build
mkdir build
cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make masssprings_showcase

# Run the simulation
./masssprings_showcase
```

## Controls
| Key/Action | Description |
|------------|-------------|
| **SPACE** | Jump! (Launch jello into the air) |
| **Click & Drag** | Grab and pull the jello vertices (Area Grab) |
| **1** | Material Preset: Raspberry Jello (Balanced) |
| **2** | Material Preset: Super Slime (High bounce, low friction) |
| **3** | Material Preset: Heavy Dough (Stiff, high damping) |
| **4** | Material Preset: Basketball (Sphere, Rigid, Bouncy) |
| **T** | Toggle Slow Motion (Matrix style) |
| **S** | Toggle Stress Visualization |
| **R** | Reset Simulation |

## Description
This showcase implements a **3D Volumetric Soft Body Simulation** using a mass-spring system. Unlike the 2D cloth in A8, this project simulates a solid 3D object with internal structure, volume preservation, and complex physical interactions.

### Features Implemented:
1.  **3D Volumetric Mesh Generation**
    *   **Code Location:** `src/JelloMesh.cpp` (`create_jello_cube`, `create_jello_sphere`)
    *   **Details:** Generates a 3D grid of vertices (6x6x6). Constructs a complex network of structural springs (x, y, z axes), shear springs (diagonals), and **Bending Springs** (2-hop connections) to maintain volume and shape.
    *   **Dynamic Mesh Switching:** Supports real-time switching between Cube and Sphere topologies (in `JelloState::set_material`).

2.  **Robust Physics Engine**
    *   **Code Location:** `src/fast_mass_springs_step_sparse.cpp`
    *   **Details:** Uses a Local-Global solver with **100-500 iterations** (adaptive based on material) for rock-solid stability even with extreme stiffness.
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

## Troubleshooting

### CMake Error about `libigl`
Make sure the `cmake/libigl.cmake` file exists. It should automatically fetch libigl from GitHub.

### Basketball material is slow
This is expected - the basketball uses more solver iterations (500) to maintain rigidity. It's CPU-bound, not GPU-bound.

### Build fails on older CMake
Add the flag `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` to your cmake command.

## Acknowledgements
*   **libigl**: For geometry processing, viewer, and project/unproject functions.
*   **Eigen**: For linear algebra operations.
*   **CSC317 Course Materials**: Base mass-spring system theory (A8) and Shader concepts (A6).
