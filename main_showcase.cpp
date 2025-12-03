#include "JelloState.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/project.h>
#include <igl/unproject.h>
#include <iostream>
#include <set>

// Global State
JelloState jello;
bool slow_motion = false;
bool show_stress = false;
double time_elapsed = 0;

// Mouse Interaction State
bool is_dragging = false;
struct GrabbedVertex {
  int id;
  Eigen::Vector3d offset;
};
std::vector<GrabbedVertex> dragged_vertices;
double drag_depth = 0.0;
Eigen::Vector3d mouse_world_pos;

int main(int argc, char * argv[])
{
  std::cout << "========================================" << std::endl;
  std::cout << "      🍮 JELLO SOFT BODY PHYSICS 🍮     " << std::endl;
  std::cout << "========================================" << std::endl;
  
  // Initialize Jello
  jello.init(6, 1.0); // 6x6x6 resolution
  std::cout << "Jello: " << jello.V.rows() << " vertices, " << jello.E.rows() << " springs" << std::endl;

  // Viewer Setup
  igl::opengl::glfw::Viewer viewer;
  
  // Mesh 0: Jello
  viewer.data().set_mesh(jello.V, jello.F);
  viewer.data().set_colors(Eigen::RowVector4d(0.9, 0.2, 0.3, 0.8)); // Raspberry
  viewer.data().show_lines = false;
  viewer.data().shininess = 100.0;
  
  // Mesh 1: Ground (3D Platform)
  viewer.append_mesh();
  const int grid_size = 8;
  const double tile_size = 2.0;
  const double platform_height = 1.0;
  const double ground_level = 0.05; // Align top with collision plane
  
  std::vector<Eigen::Vector3d> g_verts;
  std::vector<Eigen::Vector3i> g_faces;
  std::vector<Eigen::RowVector3d> g_colors;
  
  // 1. Top Surface (Checkerboard)
  for(int i = -grid_size/2; i < grid_size/2; i++) {
    for(int j = -grid_size/2; j < grid_size/2; j++) {
      int v_start = g_verts.size();
      g_verts.push_back(Eigen::Vector3d(i*tile_size, ground_level, j*tile_size));
      g_verts.push_back(Eigen::Vector3d((i+1)*tile_size, ground_level, j*tile_size));
      g_verts.push_back(Eigen::Vector3d((i+1)*tile_size, ground_level, (j+1)*tile_size));
      g_verts.push_back(Eigen::Vector3d(i*tile_size, ground_level, (j+1)*tile_size));
      
      g_faces.push_back(Eigen::Vector3i(v_start, v_start+1, v_start+2));
      g_faces.push_back(Eigen::Vector3i(v_start, v_start+2, v_start+3));
      
      Eigen::RowVector3d color = ((i+j)%2 == 0) ? 
        Eigen::RowVector3d(0.9, 0.9, 0.95) : Eigen::RowVector3d(0.8, 0.8, 0.85);
      g_colors.push_back(color);
      g_colors.push_back(color);
    }
  }
  
  // 2. Sides (Dark Grey)
  double min_x = -grid_size/2 * tile_size;
  double max_x = grid_size/2 * tile_size;
  double min_z = -grid_size/2 * tile_size;
  double max_z = grid_size/2 * tile_size;
  double bottom_y = ground_level - platform_height;
  
  auto add_quad = [&](Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4) {
    int v = g_verts.size();
    g_verts.push_back(p1); g_verts.push_back(p2); g_verts.push_back(p3); g_verts.push_back(p4);
    g_faces.push_back(Eigen::Vector3i(v, v+1, v+2));
    g_faces.push_back(Eigen::Vector3i(v, v+2, v+3));
    g_colors.push_back(Eigen::RowVector3d(0.2, 0.2, 0.25));
    g_colors.push_back(Eigen::RowVector3d(0.2, 0.2, 0.25));
  };
  
  // Front
  add_quad(Eigen::Vector3d(min_x, ground_level, max_z), Eigen::Vector3d(max_x, ground_level, max_z),
           Eigen::Vector3d(max_x, bottom_y, max_z), Eigen::Vector3d(min_x, bottom_y, max_z));
  // Back
  add_quad(Eigen::Vector3d(max_x, ground_level, min_z), Eigen::Vector3d(min_x, ground_level, min_z),
           Eigen::Vector3d(min_x, bottom_y, min_z), Eigen::Vector3d(max_x, bottom_y, min_z));
  // Left
  add_quad(Eigen::Vector3d(min_x, ground_level, min_z), Eigen::Vector3d(min_x, ground_level, max_z),
           Eigen::Vector3d(min_x, bottom_y, max_z), Eigen::Vector3d(min_x, bottom_y, min_z));
  // Right
  add_quad(Eigen::Vector3d(max_x, ground_level, max_z), Eigen::Vector3d(max_x, ground_level, min_z),
           Eigen::Vector3d(max_x, bottom_y, min_z), Eigen::Vector3d(max_x, bottom_y, max_z));
           
  // Convert to Matrix
  Eigen::MatrixXd ground_V(g_verts.size(), 3);
  Eigen::MatrixXi ground_F(g_faces.size(), 3);
  Eigen::MatrixXd ground_C(g_colors.size(), 3);
  
  for(size_t i=0; i<g_verts.size(); i++) ground_V.row(i) = g_verts[i];
  for(size_t i=0; i<g_faces.size(); i++) ground_F.row(i) = g_faces[i];
  for(size_t i=0; i<g_colors.size(); i++) ground_C.row(i) = g_colors[i];
  
  viewer.selected_data_index = 1;
  viewer.data().set_mesh(ground_V, ground_F);
  viewer.data().set_colors(ground_C);
  viewer.data().show_lines = false;
  viewer.selected_data_index = 0; // Back to Jello
  
  // Lighting
  viewer.core().background_color = Eigen::Vector4f(0.15f, 0.15f, 0.2f, 1.0f);
  viewer.core().lighting_factor = 0.8;
  viewer.core().is_animating = true;
  
  // --- Callbacks ---

  // Physics Loop
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer&) {
    if(!viewer.core().is_animating) return false;
    
    double dt = slow_motion ? jello.delta_t / 5.0 : jello.delta_t;
    time_elapsed += dt;
    
    // Gravity (Increased for snappier feel)
    Eigen::MatrixXd fext = Eigen::MatrixXd::Zero(jello.V.rows(), 3);
    fext.col(1).setConstant(-25.0); // Was -9.8
    
    // Step Physics
    Eigen::MatrixXd U_next = jello.U;
    fast_mass_springs_step_sparse(jello.V, jello.E, jello.k, jello.b, dt, fext, jello.r, jello.M, jello.A, jello.C, jello.pre, jello.Uprev, jello.U, U_next, jello.solver_iterations);
    
    // Mouse Interaction (Area Grab)
    if(is_dragging && !dragged_vertices.empty()) {
      viewer.data().set_points(mouse_world_pos.transpose(), Eigen::RowVector3d(1,1,1));
      
      for(const auto& gv : dragged_vertices) {
        Eigen::Vector3d target_pos = mouse_world_pos + gv.offset;
        
        // Correct Velocity Calculation: (New Position - Old Position) / dt
        Eigen::Vector3d mouse_vel = (target_pos - jello.U.row(gv.id).transpose()) / dt;
        
        // Safety: Clamp mouse velocity to prevent explosion
        double max_mouse_v = 30.0;
        if(mouse_vel.norm() > max_mouse_v) mouse_vel *= (max_mouse_v / mouse_vel.norm());
        
        U_next.row(gv.id) = target_pos.transpose(); // Hard constraint
        jello.Vel.row(gv.id) = mouse_vel.transpose(); // Velocity transfer
      }
    } else {
      viewer.data().set_points(Eigen::MatrixXd::Zero(0,3), Eigen::MatrixXd::Zero(0,3));
    }
    
    // Update Velocity (for non-grabbed vertices)
    std::set<int> grabbed_set;
    for(const auto& gv : dragged_vertices) grabbed_set.insert(gv.id);
    
    for(int i = 0; i < jello.V.rows(); i++) {
      if(grabbed_set.find(i) == grabbed_set.end()) {
        jello.Vel.row(i) = (U_next.row(i) - jello.U.row(i)) / dt;
      }
    }
    
    // Air Resistance & Velocity Clamping
    jello.Vel *= 0.999; // Reduced drag (was 0.99)
    double max_v = 20.0;
    for(int i = 0; i < jello.Vel.rows(); i++) {
      if(jello.Vel.row(i).norm() > max_v) jello.Vel.row(i) *= (max_v / jello.Vel.row(i).norm());
    }

    // Ground Collision
    const double ground_y = 0.05;
    for(int i = 0; i < U_next.rows(); i++) {
      if(U_next(i, 1) < ground_y) {
        double penetration = ground_y - U_next(i, 1);
        U_next(i, 1) = ground_y + penetration * 0.1; 
        
        if(jello.Vel(i, 1) < 0) {
          jello.Vel(i, 1) = -jello.Vel(i, 1) * jello.restitution;
          jello.Vel(i, 0) *= jello.friction; 
          jello.Vel(i, 2) *= jello.friction;
        }
      }
    }
    
    // Damping & Update
    jello.Vel *= jello.damping;
    jello.Uprev = jello.U;
    jello.U = jello.U + jello.Vel * dt;
    
    // Update Viewer
    viewer.data_list[0].set_vertices(jello.U);
    viewer.data_list[0].compute_normals();
    
    // Stress Visualization
    if(show_stress) {
      Eigen::MatrixXd colors = Eigen::MatrixXd::Zero(jello.V.rows(), 3);
      Eigen::VectorXd stress = Eigen::VectorXd::Zero(jello.V.rows());
      for(int e = 0; e < jello.E.rows(); e++) {
        double len = (jello.U.row(jello.E(e,0)) - jello.U.row(jello.E(e,1))).norm();
        double strain = abs(len - jello.r(e)) / jello.r(e);
        stress(jello.E(e,0)) += strain; stress(jello.E(e,1)) += strain;
      }
      double max_s = stress.maxCoeff();
      if(max_s > 1e-6) stress /= max_s;
      for(int i = 0; i < jello.V.rows(); i++) {
        double s = std::min(1.0, stress(i));
        colors(i, 0) = 0.9;
        colors(i, 1) = 0.3 * (1.0 - s) + 0.9 * s;
        colors(i, 2) = 0.2 * (1.0 - s) + 0.1 * s;
      }
      viewer.data_list[0].set_colors(colors);
    }
    
    return false;
  };
  
  // Mouse Inputs
  viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer&, int, int)->bool {
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    
    int closest_v = -1;
    double min_dist = 1e9;
    
    for(int i = 0; i < jello.V.rows(); i++) {
      Eigen::Vector3d p = jello.U.row(i);
      Eigen::Vector3d proj = igl::project(p, viewer.core().view.cast<double>().eval(), viewer.core().proj.cast<double>().eval(), viewer.core().viewport.cast<double>().eval());
      double dist = sqrt(pow(x - proj(0), 2) + pow(y - proj(1), 2));
      if(dist < min_dist) { min_dist = dist; closest_v = i; drag_depth = proj(2); }
    }
    
    if(closest_v != -1 && min_dist < 30.0) {
      is_dragging = true;
      mouse_world_pos = igl::unproject(Eigen::Vector3d(x, y, drag_depth), viewer.core().view.cast<double>().eval(), viewer.core().proj.cast<double>().eval(), viewer.core().viewport.cast<double>().eval());
      
      dragged_vertices.clear();
      double grab_radius = 0.4;
      for(int i = 0; i < jello.V.rows(); i++) {
        if((jello.U.row(i) - mouse_world_pos.transpose()).norm() < grab_radius) {
          dragged_vertices.push_back({i, jello.U.row(i).transpose() - mouse_world_pos});
        }
      }
      if(dragged_vertices.empty()) dragged_vertices.push_back({closest_v, jello.U.row(closest_v).transpose() - mouse_world_pos});
      return true;
    }
    return false;
  };
  
  viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer&, int, int)->bool {
    if(is_dragging) {
      double x = viewer.current_mouse_x;
      double y = viewer.core().viewport(3) - viewer.current_mouse_y;
      mouse_world_pos = igl::unproject(Eigen::Vector3d(x, y, drag_depth), viewer.core().view.cast<double>().eval(), viewer.core().proj.cast<double>().eval(), viewer.core().viewport.cast<double>().eval());
      return true;
    }
    return false;
  };
  
  viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int, int)->bool {
    is_dragging = false;
    dragged_vertices.clear();
    return false;
  };
  
  // Keyboard Inputs
  viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer&, unsigned char key, int) {
    switch(key) {
      case '1': 
        jello.set_material(1); 
        viewer.data_list[0].set_mesh(jello.V, jello.F); // Update Mesh
        viewer.data_list[0].compute_normals();
        viewer.data_list[0].set_colors(Eigen::RowVector4d(0.9, 0.2, 0.3, 0.8)); 
        std::cout << "Material: Jello" << std::endl; 
        return true;
      case '2': 
        jello.set_material(2); 
        viewer.data_list[0].set_mesh(jello.V, jello.F); // Update Mesh
        viewer.data_list[0].compute_normals();
        viewer.data_list[0].set_colors(Eigen::RowVector4d(0.2, 0.9, 0.9, 0.8)); 
        std::cout << "Material: Slime" << std::endl; 
        return true;
      case '3': 
        jello.set_material(3); 
        viewer.data_list[0].set_mesh(jello.V, jello.F); // Update Mesh
        viewer.data_list[0].compute_normals();
        viewer.data_list[0].set_colors(Eigen::RowVector4d(0.9, 0.8, 0.4, 1.0)); 
        std::cout << "Material: Dough (Heavy)" << std::endl;
        return true;
      case '4': 
        jello.set_material(4); 
        viewer.data_list[0].set_mesh(jello.V, jello.F); // Update Mesh
        viewer.data_list[0].compute_normals();
        viewer.data_list[0].set_colors(Eigen::RowVector4d(1.0, 0.45, 0.0, 1.0)); 
        std::cout << "Material: Basketball" << std::endl; 
        return true;
      case 'T': case 't': slow_motion = !slow_motion; std::cout << "Slow Motion: " << (slow_motion ? "ON" : "OFF") << std::endl; return true;
      case 'S': case 's': show_stress = !show_stress; std::cout << "Stress Viz: " << (show_stress ? "ON" : "OFF") << std::endl; return true;
      case 'R': case 'r': jello.reset(); std::cout << "Reset" << std::endl; return true;
      case ' ': 
        for(int i = 0; i < jello.U.rows(); i++) {
          jello.Uprev(i, 1) = jello.U(i, 1) - 12.0 * jello.delta_t;
          jello.Uprev(i, 0) -= ((rand()%100/100.0-0.5)*5.0) * jello.delta_t;
          jello.Uprev(i, 2) -= ((rand()%100/100.0-0.5)*5.0) * jello.delta_t;
        }
        std::cout << "BOING!" << std::endl;
        return true;
    }
    return false;
  };
  
  viewer.core().camera_eye = Eigen::Vector3f(2.5f, 2.0f, 2.5f);
  viewer.core().camera_center = Eigen::Vector3f(0, 0.5f, 0);
  viewer.launch();
  return 0;
}
