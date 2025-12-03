#include "signed_incidence_matrix_sparse.h"
#include <vector>

void signed_incidence_matrix_sparse(
  const int n,
  const Eigen::MatrixXi & E,
  Eigen::SparseMatrix<double>  & A)
{
    // using tut8 and recomneded readings "Chapter 16.5 of Fundamentals of Computer Graphics (4th Edition)" and Fast Simulation of Mass-Spring Systems" by Liu et al. 2013

  // Build sparse matrix using triplet list (only non-zeros)
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(2 * E.rows());  // Each edge has 2 non-zero entries
  
  // For each edge, add +1 and -1 entries
  for(int e = 0; e < E.rows(); e++)
  {
    int i = E(e, 0);  // First vertex
    int j = E(e, 1);  // Second vertex
    triplets.emplace_back(e, i, 1.0);   // +1 for first vertex
    triplets.emplace_back(e, j, -1.0);  // -1 for second vertex
  }
  
  // Construct sparse matrix from triplets
  A.resize(E.rows(), n);
  A.setFromTriplets(triplets.begin(), triplets.end());
}
