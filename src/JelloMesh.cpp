#include "JelloMesh.h"
#include <set>
#include <iostream>

void JelloMesh::create_jello_cube(
  int res_x, int res_y, int res_z,
  double size,
  Eigen::MatrixXd& V,
  Eigen::MatrixXi& F,
  Eigen::MatrixXi& E,
  Eigen::VectorXi& b)
{
  // Create 3D grid of vertices
  std::vector<Eigen::Vector3d> vertices;
  auto idx = [&](int i, int j, int k) {
    return i * res_y * res_z + j * res_z + k;
  };
  
  for(int i = 0; i < res_x; i++) {
    for(int j = 0; j < res_y; j++) {
      for(int k = 0; k < res_z; k++) {
        vertices.push_back(Eigen::Vector3d(
          (i / double(res_x-1) - 0.5) * size,
          (j / double(res_y-1)) * size + 1.0,
          (k / double(res_z-1) - 0.5) * size
        ));
      }
    }
  }
  
  V.resize(vertices.size(), 3);
  for(size_t i = 0; i < vertices.size(); i++) {
    V.row(i) = vertices[i];
  }
  
  // Create edges (structural springs)
  std::set<std::pair<int,int>> edge_set;
  auto add_edge = [&](int a, int b) {
    if(a > b) std::swap(a, b);
    edge_set.insert({a, b});
  };
  
  // Connect neighboring vertices in 3D grid
  for(int i = 0; i < res_x; i++) {
    for(int j = 0; j < res_y; j++) {
      for(int k = 0; k < res_z; k++) {
        int v = idx(i, j, k);
        // X direction
        if(i < res_x-1) add_edge(v, idx(i+1, j, k));
        // Y direction
        if(j < res_y-1) add_edge(v, idx(i, j+1, k));
        // Z direction
        if(k < res_z-1) add_edge(v, idx(i, j, k+1));
        
        // Diagonal springs for stability
        if(i < res_x-1 && j < res_y-1) add_edge(v, idx(i+1, j+1, k));
        if(i < res_x-1 && k < res_z-1) add_edge(v, idx(i+1, j, k+1));
        if(j < res_y-1 && k < res_z-1) add_edge(v, idx(i, j+1, k+1));
        
        // Face diagonals
        if(i < res_x-1 && j < res_y-1 && k < res_z-1) {
          add_edge(v, idx(i+1, j+1, k+1));
          add_edge(idx(i+1, j, k), idx(i, j+1, k+1));
        }
        
        // Bending Springs (Connect i to i+2) - Helps maintain "Cube" shape
        if(i < res_x-2) add_edge(v, idx(i+2, j, k));
        if(j < res_y-2) add_edge(v, idx(i, j+2, k));
        if(k < res_z-2) add_edge(v, idx(i, j, k+2));
      }
    }
  }
  
  E.resize(edge_set.size(), 2);
  int e_idx = 0;
  for(auto& edge : edge_set) {
    E(e_idx, 0) = edge.first;
    E(e_idx, 1) = edge.second;
    e_idx++;
  }
  
  // Extract surface for rendering
  std::vector<Eigen::Vector3i> faces;
  
  // Bottom and top faces
  for(int i = 0; i < res_x-1; i++) {
    for(int k = 0; k < res_z-1; k++) {
      // Bottom (j=0)
      faces.push_back(Eigen::Vector3i(idx(i,0,k), idx(i+1,0,k), idx(i,0,k+1)));
      faces.push_back(Eigen::Vector3i(idx(i+1,0,k), idx(i+1,0,k+1), idx(i,0,k+1)));
      // Top (j=res_y-1)
      int j = res_y-1;
      faces.push_back(Eigen::Vector3i(idx(i,j,k), idx(i,j,k+1), idx(i+1,j,k)));
      faces.push_back(Eigen::Vector3i(idx(i+1,j,k), idx(i,j,k+1), idx(i+1,j,k+1)));
    }
  }
  
  // Front and back faces
  for(int i = 0; i < res_x-1; i++) {
    for(int j = 0; j < res_y-1; j++) {
      // Front (k=0)
      faces.push_back(Eigen::Vector3i(idx(i,j,0), idx(i,j+1,0), idx(i+1,j,0)));
      faces.push_back(Eigen::Vector3i(idx(i+1,j,0), idx(i,j+1,0), idx(i+1,j+1,0)));
      // Back (k=res_z-1)
      int k = res_z-1;
      faces.push_back(Eigen::Vector3i(idx(i,j,k), idx(i+1,j,k), idx(i,j+1,k)));
      faces.push_back(Eigen::Vector3i(idx(i+1,j,k), idx(i+1,j+1,k), idx(i,j+1,k)));
    }
  }
  
  // Left and right faces
  for(int j = 0; j < res_y-1; j++) {
    for(int k = 0; k < res_z-1; k++) {
      // Left (i=0)
      faces.push_back(Eigen::Vector3i(idx(0,j,k), idx(0,j+1,k), idx(0,j,k+1)));
      faces.push_back(Eigen::Vector3i(idx(0,j+1,k), idx(0,j+1,k+1), idx(0,j,k+1)));
      // Right (i=res_x-1)
      int i = res_x-1;
      faces.push_back(Eigen::Vector3i(idx(i,j,k), idx(i,j,k+1), idx(i,j+1,k)));
      faces.push_back(Eigen::Vector3i(idx(i,j+1,k), idx(i,j,k+1), idx(i,j+1,k+1)));
    }
  }
  
  F.resize(faces.size(), 3);
  for(size_t i = 0; i < faces.size(); i++) {
    F.row(i) = faces[i];
  }
  
  // No pinned vertices for free-falling jello
  b.resize(0);
}

void JelloMesh::create_jello_sphere(
  int res,
  double radius,
  Eigen::MatrixXd& V,
  Eigen::MatrixXi& F,
  Eigen::MatrixXi& E,
  Eigen::VectorXi& b)
{
  // 1. Generate Cube Topology first
  create_jello_cube(res, res, res, 2.0, V, F, E, b); // Size 2.0 -> Range [-1, 1]
  
  // 2. Warp positions to Sphere (Robust Normalization Method)
  Eigen::Vector3d center(0, 2.0, 0); // Center of the generated cube (y is [1, 3])
  
  for(int i = 0; i < V.rows(); i++) {
    Eigen::Vector3d p = V.row(i).transpose();
    Eigen::Vector3d dir = (p - center).normalized();
    
    // Project to sphere surface
    V.row(i) = dir.transpose() * radius + Eigen::RowVector3d(0, radius + 0.1, 0);
  }
  
  // 3. Add Internal Pressure Springs (Cross-connections)
  // Connect vertices to their far neighbors to prevent squashing
  std::set<std::pair<int,int>> edge_set;
  for(int i=0; i<E.rows(); i++) {
     if(E(i,0) < E(i,1)) edge_set.insert({E(i,0), E(i,1)});
     else edge_set.insert({E(i,1), E(i,0)});
  }
  
  // Add long-range springs
  for(int i = 0; i < V.rows(); i++) {
    for(int j = i + 1; j < V.rows(); j++) {
      // If distance is large (across the sphere), add a spring
      if((V.row(i) - V.row(j)).norm() > radius * 1.5) {
        // Only add a few random ones to avoid explosion, or structured ones?
        // Let's just add springs between i and (res*res*res - 1 - i) which is roughly opposite in the grid ordering
        // Actually, grid ordering: i,j,k. Opposite is res-1-i, res-1-j, res-1-k.
      }
    }
  }
  
  // Better approach: Since we know the grid structure, connect (i,j,k) to (res-1-i, res-1-j, res-1-k)
  auto idx = [&](int i, int j, int k) { return i * res * res + j * res + k; };
  for(int i = 0; i < res; i++) {
    for(int j = 0; j < res; j++) {
      for(int k = 0; k < res; k++) {
        int v1 = idx(i, j, k);
        int v2 = idx(res-1-i, res-1-j, res-1-k);
        if(v1 < v2) edge_set.insert({v1, v2}); // Diametric spring
        
        // Also connect to center-ish?
        // Let's add springs to (res/2, res/2, res/2) - center of volume
        // No, center might not exist as a vertex if res is even.
      }
    }
  }
  
  // Rebuild E
  E.resize(edge_set.size(), 2);
  int e_idx = 0;
  for(auto& edge : edge_set) {
    E(e_idx, 0) = edge.first;
    E(e_idx, 1) = edge.second;
    e_idx++;
  }
}
