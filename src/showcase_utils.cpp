#include "showcase_utils.h"
#include <igl/cylinder.h>
#include <igl/upsample.h>
#include <cmath>

void create_geometric_character(
  Eigen::MatrixXd & V,
  Eigen::MatrixXi & F)
{
  // Create a simple character using cylinders and spheres
  // Body: Cylinder (torso)
  // Head: Sphere on top
  
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> faces;
  
  // Generate torso cylinder (20 segments, height 1.5)
  const int segments = 20;
  const double torso_radius = 0.3;
  const double torso_height = 1.5;
  const double head_radius = 0.2;
  
  // Torso vertices
  for(int i = 0; i <= segments; i++) {
    double angle = 2.0 * M_PI * i / segments;
    double x = torso_radius * cos(angle);
    double z = torso_radius * sin(angle);
    
    // Bottom ring
    vertices.push_back(Eigen::Vector3d(x, 0.0, z));
    // Top ring (shoulders)
    vertices.push_back(Eigen::Vector3d(x, torso_height, z));
  }
  
  // Head vertices (sphere on top)
  const int head_segments = 10;
  for(int i = 0; i <= head_segments; i++) {
    double theta = M_PI * i / head_segments;  // 0 to PI
    for(int j = 0; j < segments; j++) {
      double phi = 2.0 * M_PI * j / segments;
      double x = head_radius * sin(theta) * cos(phi);
      double y = torso_height + 0.3 + head_radius * cos(theta);  // Offset above torso
      double z = head_radius * sin(theta) * sin(phi);
      vertices.push_back(Eigen::Vector3d(x, y, z));
    }
  }
  
  // Convert to Eigen matrix
  V.resize(vertices.size(), 3);
  for(size_t i = 0; i < vertices.size(); i++) {
    V.row(i) = vertices[i];
  }
  
  // Generate faces for torso
  int torso_verts = 2 * (segments + 1);
  for(int i = 0; i < segments; i++) {
    int i0 = 2 * i;
    int i1 = 2 * i + 1;
    int i2 = 2 * (i + 1);
    int i3 = 2 * (i + 1) + 1;
    
    faces.push_back(Eigen::Vector3i(i0, i2, i1));
    faces.push_back(Eigen::Vector3i(i1, i2, i3));
  }
  
  // Generate faces for head
  int head_start = torso_verts;
  for(int i = 0; i < head_segments; i++) {
    for(int j = 0; j < segments; j++) {
      int i0 = head_start + i * segments + j;
      int i1 = head_start + i * segments + (j + 1) % segments;
      int i2 = head_start + (i + 1) * segments + j;
      int i3 = head_start + (i + 1) * segments + (j + 1) % segments;
      
      faces.push_back(Eigen::Vector3i(i0, i2, i1));
      faces.push_back(Eigen::Vector3i(i1, i2, i3));
    }
  }
  
  // Convert faces to Eigen matrix
  F.resize(faces.size(), 3);
  for(size_t i = 0; i < faces.size(); i++) {
    F.row(i) = faces[i];
  }
}

void create_cape_mesh(
  const Eigen::MatrixXd & char_V,
  int width,
  int height,
  Eigen::MatrixXd & cape_V,
  Eigen::MatrixXi & cape_F,
  Eigen::MatrixXi & cape_E,
  std::vector<int> & attachment_indices)
{
  // Create a rectangular cloth mesh positioned behind character
  // width x height grid of vertices
  
  const double cape_width_m = 120;
  const double cape_height_m = 150;
  
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> faces;
  std::vector<Eigen::Vector2i> edges;
  
  for(int i = 0; i < height; i++) {
    for(int j = 0; j < width; j++) {
      double x = (j / double(width - 1) - 0.5) * cape_width_m;
      double y = 180 - (i / double(height - 1)) * cape_height_m;
      double z = -30;
      
      vertices.push_back(Eigen::Vector3d(x, y, z));
    }
  }
  
  // Convert to Eigen matrix
  cape_V.resize(vertices.size(), 3);
  for(size_t i = 0; i < vertices.size(); i++) {
    cape_V.row(i) = vertices[i];
  }
  
  // Generate triangular faces
  for(int i = 0; i < height - 1; i++) {
    for(int j = 0; j < width - 1; j++) {
      int i0 = i * width + j;
      int i1 = i * width + (j + 1);
      int i2 = (i + 1) * width + j;
      int i3 = (i + 1) * width + (j + 1);
      
      faces.push_back(Eigen::Vector3i(i0, i2, i1));
      faces.push_back(Eigen::Vector3i(i1, i2, i3));
    }
  }
  
  cape_F.resize(faces.size(), 3);
  for(size_t i = 0; i < faces.size(); i++) {
    cape_F.row(i) = faces[i];
  }
  
  // Generate edges (springs)
  // Horizontal edges
  for(int i = 0; i < height; i++) {
    for(int j = 0; j < width - 1; j++) {
      edges.push_back(Eigen::Vector2i(i * width + j, i * width + j + 1));
    }
  }
  // Vertical edges
  for(int i = 0; i < height - 1; i++) {
    for(int j = 0; j < width; j++) {
      edges.push_back(Eigen::Vector2i(i * width + j, (i + 1) * width + j));
    }
  }
  // Diagonal edges (shear springs)
  for(int i = 0; i < height - 1; i++) {
    for(int j = 0; j < width - 1; j++) {
      edges.push_back(Eigen::Vector2i(i * width + j, (i + 1) * width + j + 1));
      edges.push_back(Eigen::Vector2i(i * width + j + 1, (i + 1) * width + j));
    }
  }
  
  cape_E.resize(edges.size(), 2);
  for(size_t i = 0; i < edges.size(); i++) {
    cape_E.row(i) = edges[i];
  }
  
  // Top row vertices are attachment points (pinned to shoulders)
  attachment_indices.clear();
  for(int j = 0; j < width; j++) {
    attachment_indices.push_back(j);
  }
}

std::vector<CollisionSphere> get_character_collision_spheres(
  const Eigen::MatrixXd & char_V)
{
  std::vector<CollisionSphere> spheres;
  
  // Beast model collision spheres
  CollisionSphere head;
  head.center = Eigen::Vector3d(0, 220, 0);
  head.radius = 35;
  spheres.push_back(head);
  
  CollisionSphere upper_torso;
  upper_torso.center = Eigen::Vector3d(0, 180, 0);
  upper_torso.radius = 45;
  spheres.push_back(upper_torso);
  
  CollisionSphere mid_torso;
  mid_torso.center = Eigen::Vector3d(0, 140, 0);
  mid_torso.radius = 50;
  spheres.push_back(mid_torso);
  
  CollisionSphere lower_torso;
  lower_torso.center = Eigen::Vector3d(0, 100, 0);
  lower_torso.radius = 48;
  spheres.push_back(lower_torso);
  
  CollisionSphere back;
  back.center = Eigen::Vector3d(0, 160, -20);
  back.radius = 40;
  spheres.push_back(back);
  
  return spheres;
}
