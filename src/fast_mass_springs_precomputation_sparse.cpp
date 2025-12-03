#include "fast_mass_springs_precomputation_sparse.h"
#include "signed_incidence_matrix_sparse.h"
#include <vector>

bool fast_mass_springs_precomputation_sparse(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & E,
  const double k,
  const Eigen::VectorXd & m,
  const Eigen::VectorXi & b,
  const double delta_t,
  Eigen::VectorXd & r,
  Eigen::SparseMatrix<double>  & M,
  Eigen::SparseMatrix<double>  & A,
  Eigen::SparseMatrix<double>  & C,
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > & prefactorization)
{
    // using tut8 and recomneded readings "Chapter 16.5 of Fundamentals of Computer Graphics (4th Edition)" and Fast Simulation of Mass-Spring Systems" by Liu et al. 2013

  const int n = V.rows();
  const int num_edges = E.rows();
  const int num_pinned = b.rows();
  
  // 1. Compute rest lengths r (same as dense)
  r.resize(num_edges);
  for(int e = 0; e < num_edges; e++)
  {
    int i = E(e, 0);
    int j = E(e, 1);
    r(e) = (V.row(i) - V.row(j)).norm();
  }
  
  // 2. Build sparse mass matrix M (diagonal)
  std::vector<Eigen::Triplet<double>> M_triplets;
  M_triplets.reserve(n);
  for(int i = 0; i < n; i++)
  {
    M_triplets.emplace_back(i, i, m(i));
  }
  M.resize(n, n);
  M.setFromTriplets(M_triplets.begin(), M_triplets.end());
  
  // 3. Build sparse signed incidence matrix A
  signed_incidence_matrix_sparse(n, E, A);
  
  // 4. Build sparse selection matrix C for pinned vertices
  std::vector<Eigen::Triplet<double>> C_triplets;
  C_triplets.reserve(num_pinned);
  for(int i = 0; i < num_pinned; i++)
  {
    C_triplets.emplace_back(i, b(i), 1.0);
  }
  C.resize(num_pinned, n);
  C.setFromTriplets(C_triplets.begin(), C_triplets.end());
  
  // 5. Build system matrix Q = k*A^T*A + (1/dt²)*M + w*C^T*C
  const double w = 1e10;
  Eigen::SparseMatrix<double> Q = k * (A.transpose() * A) + 
                                   (1.0 / (delta_t * delta_t)) * M + 
                                   w * (C.transpose() * C);
  
  // 6. Prefactorize Q using SimplicialLLT
  prefactorization.compute(Q);
  return prefactorization.info() != Eigen::NumericalIssue;
}
