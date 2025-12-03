#ifndef JELLO_MESH_H
#define JELLO_MESH_H

#include <Eigen/Core>
#include <vector>

/**
 * @brief Handles the generation of the 3D jello mesh.
 */
class JelloMesh {
public:
    /**
     * @brief Creates a 3D grid mesh with structural and shear springs.
     * 
     * @param res_x Resolution in X (number of vertices)
     * @param res_y Resolution in Y
     * @param res_z Resolution in Z
     * @param size Physical size of the cube
     * @param V Output vertices
     * @param F Output faces (for rendering)
     * @param E Output edges (springs)
     * @param b Output pinned vertices (empty for free-falling)
     */
    static void create_jello_cube(
        int res_x, int res_y, int res_z,
        double size,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& b
    );

    /**
     * @brief Creates a spherical jello mesh using concentric cube mapping.
     */
    static void create_jello_sphere(
        int res,
        double radius,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& b
    );
};

#endif // JELLO_MESH_H
