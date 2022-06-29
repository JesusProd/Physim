//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Solvers/LinearSolver.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class LinearSolver_EigenLDLT : public LinearSolver {
  SimplicialLDLT<MatrixSd> m_solver;

 public:
  LinearSolver_EigenLDLT();
  LinearSolver_EigenLDLT(const MatrixSd& mA,
                         const LinearSolverOptions& options);
  virtual void Init(const MatrixSd& mA, const LinearSolverOptions& options);
  virtual ~LinearSolver_EigenLDLT();

 protected:
  virtual LSResult SolveInternal(MatrixSd& mA,
                                 const VectorXd& vb,
                                 VectorXd& vx);
  virtual LSResult SolveInternal(MatrixSd& mA,
                                 const MatrixXd& mB,
                                 MatrixXd& mX);
  virtual LSResult SolveInternal(MatrixSd& mA,
                                 const MatrixSd& mB,
                                 MatrixSd& mX);

  virtual void FreeInternal();
};
}  // namespace PhySim
