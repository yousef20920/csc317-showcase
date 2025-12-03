#include "fast_mass_springs_step_sparse.h"
#include <igl/matlab_format.h>

void fast_mass_springs_step_sparse(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & E,
  const double k,
  const Eigen::VectorXi & b,
  const double delta_t,
  const Eigen::MatrixXd & fext,
  const Eigen::VectorXd & r,
  const Eigen::SparseMatrix<double>  & M,
  const Eigen::SparseMatrix<double>  & A,
  const Eigen::SparseMatrix<double>  & C,
  const Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > & prefactorization,
  const Eigen::MatrixXd & Uprev,
  const Eigen::MatrixXd & Ucur,
  Eigen::MatrixXd & Unext,
  int iterations)
{
    // using tut8 and recomneded readings "Chapter 16.5 of Fundamentals of Computer Graphics (4th Edition)" and Fast Simulation of Mass-Spring Systems" by Liu et al. 2013

  const int num_edges = E.rows();
  const double w = 1e10;
  
  // Compute constant terms (outside iteration loop)
  // y1 = (1/dt²)*M*(2*Ucur - Uprev) + fext
  Eigen::MatrixXd y1 = (1.0 / (delta_t * delta_t)) * M * (2.0 * Ucur - Uprev) + fext;
  
  // y2 = w*C^T*C*V (penalty term for pinned vertices)
  Eigen::MatrixXd y2 = w * C.transpose() * (C * V);
  
  // Initialize Unext with current positions
  Unext = Ucur;
  
  // Local-Global iterations  // Iterate
  for(int iter = 0; iter < iterations; iter++)
  {
    // LOCAL STEP: Compute optimal spring directions d
    Eigen::MatrixXd d(num_edges, 3);
    for(int e = 0; e < num_edges; e++)
    {
      int i = E(e, 0);
      int j = E(e, 1);
      
      // Current spring vector
      Eigen::RowVector3d spring_vec = Unext.row(i) - Unext.row(j);
      
      // Normalize and scale to rest length
      double current_length = spring_vec.norm();
      if(current_length > 1e-10)
      {
        d.row(e) = spring_vec / current_length * r(e);
      }
      else
      {
        d.row(e) = spring_vec;
      }
    }
    
    // GLOBAL STEP: Solve Qp = rhs for new positions
    // rhs = k*A^T*d + y1 + y2
    Eigen::MatrixXd rhs = k * A.transpose() * d + y1 + y2;
    
    // Solve using prefactored Q
    Unext = prefactorization.solve(rhs);
  }
}
