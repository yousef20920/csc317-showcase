#ifndef SHOWCASE_UTILS_H
#define SHOWCASE_UTILS_H

#include <Eigen/Core>
#include <vector>

// Generate a simple geometric character (capsule body + sphere head)
void create_geometric_character(
  Eigen::MatrixXd & V,  // Output vertices
  Eigen::MatrixXi & F); // Output faces

// Generate a cape mesh attached to shoulders
void create_cape_mesh(
  const Eigen::MatrixXd & char_V,  // Character vertices
  int width,  // Cape width resolution
  int height, // Cape height resolution
  Eigen::MatrixXd & cape_V,  // Output cape vertices
  Eigen::MatrixXi & cape_F,  // Output cape faces
  Eigen::MatrixXi & cape_E,  // Output cape edges
  std::vector<int> & attachment_indices); // Vertices to pin

// Get body part collision spheres for character
struct CollisionSphere {
  Eigen::Vector3d center;
  double radius;
};

std::vector<CollisionSphere> get_character_collision_spheres(
  const Eigen::MatrixXd & char_V);

#endif
