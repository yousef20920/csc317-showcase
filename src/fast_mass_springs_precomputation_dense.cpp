#include "fast_mass_springs_precomputation_dense.h"
#include "signed_incidence_matrix_dense.h"
#include <Eigen/Dense>

bool fast_mass_springs_precomputation_dense(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & E,
  const double k,
  const Eigen::VectorXd & m,
  const Eigen::VectorXi & b,
  const double delta_t,
  Eigen::VectorXd & r,
  Eigen::MatrixXd & M,
  Eigen::MatrixXd & A,
  Eigen::MatrixXd & C,
  Eigen::LLT<Eigen::MatrixXd> & prefactorization)
{
  // using tut8 and recomneded readings "Chapter 16.5 of Fundamentals of Computer Graphics (4th Edition)" and Fast Simulation of Mass-Spring Systems" by Liu et al. 2013

  const int n = V.rows();  // Number of vertices
  const int num_edges = E.rows();  // Number of edges
  const int num_pinned = b.rows();  // Number of pinned vertices
  
  // 1. Compute rest lengths r (initial edge lengths)
  r.resize(num_edges);
  for(int e = 0; e < num_edges; e++)
  {
    int i = E(e, 0);
    int j = E(e, 1);
    r(e) = (V.row(i) - V.row(j)).norm();
  }
  
  // 2. Build mass matrix M (diagonal matrix with mass values)
  M = Eigen::MatrixXd::Zero(n, n);
  for(int i = 0; i < n; i++)
  {
    M(i, i) = m(i);
  }
  
  // 3. Build signed incidence matrix A
  signed_incidence_matrix_dense(n, E, A);
  
  // 4. Build selection matrix C for pinned vertices
  C = Eigen::MatrixXd::Zero(num_pinned, n);
  for(int i = 0; i < num_pinned; i++)
  {
    C(i, b(i)) = 1.0;
  }
  
  // 5. Build system matrix Q
  // Q = k*A^T*A + (1/dt²)*M + w*C^T*C
  const double w = 1e10;  // Penalty weight for pinned vertices
  Eigen::MatrixXd Q = k * A.transpose() * A + 
                      (1.0 / (delta_t * delta_t)) * M + 
                      w * C.transpose() * C;
  
  // 6. Prefactorize Q using LLT (Cholesky) decomposition
  prefactorization.compute(Q);
  return prefactorization.info() != Eigen::NumericalIssue;
}
