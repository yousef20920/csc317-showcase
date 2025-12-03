#ifndef JELLO_STATE_H
#define JELLO_STATE_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include "JelloMesh.h"
#include "fast_mass_springs_precomputation_sparse.h"
#include "fast_mass_springs_step_sparse.h"

/**
 * @brief Manages the physics state and parameters of the jello simulation.
 */
struct JelloState {
    // Mesh Data
    Eigen::MatrixXd V; ///< Initial vertices
    Eigen::MatrixXi F; ///< Faces
    Eigen::MatrixXi F_ground; ///< Ground faces
    Eigen::MatrixXi E; ///< Springs
    Eigen::VectorXi b; ///< Pinned vertices (indices)

    // Physics State
    Eigen::MatrixXd U;     ///< Current positions
    Eigen::MatrixXd Uprev; ///< Previous positions
    Eigen::MatrixXd Vel;   ///< Current velocities

    // Physics Parameters
    double k = 2e4;            ///< Spring stiffness
    double damping = 0.95;     ///< Velocity damping
    double restitution = 0.4;  ///< Bounciness
    double friction = 0.8;     ///< Ground friction
    double delta_t = 1.0/60.0; ///< Time step
    int solver_iterations = 100; ///< Number of solver iterations

    // Solver Data
    Eigen::VectorXd m;
    Eigen::VectorXd r;
    Eigen::SparseMatrix<double> M, A, C;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> pre;
    
    enum MeshType { CUBE, SPHERE };
    MeshType current_mesh_type = CUBE;

    /**
     * @brief Initialize the jello mesh and physics system.
     */
    void init(int res, double size) {
        if(current_mesh_type == CUBE) {
            JelloMesh::create_jello_cube(res, res, res, size, V, F, E, b);
        } else {
            JelloMesh::create_jello_sphere(res, size/2.0, V, F, E, b); // Radius = size/2
        }
        m = Eigen::VectorXd::Constant(V.rows(), 1.0);
        reset();
        recompute_system();
    }

    /**
     * @brief Reset simulation to initial state.
     */
    void reset() {
        U = V;
        Uprev = V;
        Vel = Eigen::MatrixXd::Zero(V.rows(), 3);
    }

    /**
     * @brief Recompute system matrices (call after changing k).
     */
    void recompute_system() {
        fast_mass_springs_precomputation_sparse(V, E, k, m, b, delta_t, r, M, A, C, pre);
    }

    /**
     * @brief Set material preset.
     * @param mode 1: Jello, 2: Slime, 3: Dough, 4: Basketball
     */
    void set_material(int mode) {
        bool mesh_changed = false;
        
        if(mode == 4) { // Basketball
            if(current_mesh_type != SPHERE) { current_mesh_type = SPHERE; mesh_changed = true; }
            k = 1e7; damping = 0.999; restitution = 0.98; friction = 0.1; // Super stiff & bouncy
            solver_iterations = 100; // Reduced from 1000 for better performance
        } else {
            solver_iterations = 100; // Standard iterations
            if(current_mesh_type != CUBE) { current_mesh_type = CUBE; mesh_changed = true; }
            
            if(mode == 1) { // Jello
                k = 2e4; damping = 0.95; restitution = 0.4; friction = 0.8;
            } else if(mode == 2) { // Slime
                k = 1e4; damping = 0.99; restitution = 0.85; friction = 0.9;
            } else if(mode == 3) { // Dough
                k = 6e4; damping = 0.98; restitution = 0.1; friction = 0.5;
            }
        }
        
        if(mesh_changed) {
            init(6, 1.0); // Re-initialize mesh
        } else {
            recompute_system();
            reset();
        }
    }
};

#endif // JELLO_STATE_H
