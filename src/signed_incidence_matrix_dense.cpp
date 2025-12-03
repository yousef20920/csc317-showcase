#include "signed_incidence_matrix_dense.h"

void signed_incidence_matrix_dense(
  const int n,
  const Eigen::MatrixXi & E,
  Eigen::MatrixXd & A)
{
    // using tut8 and recomneded readings "Chapter 16.5 of Fundamentals of Computer Graphics (4th Edition)" and Fast Simulation of Mass-Spring Systems" by Liu et al. 2013

  // Initialize A as zero matrix with dimensions: #edges × #vertices
  A = Eigen::MatrixXd::Zero(E.rows(), n);
  
  // For each edge, set +1 for first vertex and -1 for second vertex
  // This creates the signed incidence matrix where A*p gives spring vectors
  for(int e = 0; e < E.rows(); e++)
  {
    int i = E(e, 0);  // First vertex of edge e
    int j = E(e, 1);  // Second vertex of edge e
    A(e, i) = 1.0;    // +1 for first vertex
    A(e, j) = -1.0;   // -1 for second vertex
  }
}
