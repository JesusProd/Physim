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

#include <PhySim/Utils/CustomTimer.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

struct LinearSolverOptions {
  Real maxError;
  int maxIters;
  int regIters;
  Real regSign;
  LSSolverType type;
  bool profileTime;

  LinearSolverOptions() {
    type = LSSolverType::EigenLDLT;
    maxError = 1e-6;
    maxIters = 1000;
    regIters = 10;
    regSign = 1;
    profileTime = true;
  }
};

class ILinearSolver {
 public:
  ILinearSolver(){};
  virtual ~ILinearSolver(){};

  virtual void Init(const MatrixSd& mA, const LinearSolverOptions& options) = 0;
  virtual LSResult Solve(MatrixSd& mA, const VectorXd& vb, VectorXd& vx) = 0;
  virtual LSResult Solve(MatrixSd& mA, const MatrixXd& mB, MatrixXd& mX) = 0;
  virtual LSResult Solve(MatrixSd& mA, const MatrixSd& mB, MatrixSd& mX) = 0;

  virtual void GetMatrixData(const MatrixSd& mA,
                             Real& normA,
                             Real& sinT,
                             Real& regT) = 0;
};

class LinearSolver : public ILinearSolver {
 protected:
  LinearSolverOptions m_options;

  CustomTimer m_timerLinearSolve;

  bool m_isInitialized;

  MatrixSd m_mR;

 public:
  LinearSolver();
  LinearSolver(const MatrixSd& mA, const LinearSolverOptions& options);
  virtual void Init(const MatrixSd& mA, const LinearSolverOptions& options);
  virtual ~LinearSolver();

  virtual LinearSolverOptions& SetupOptions() { return this->m_options; }
  virtual void SetOptions(LinearSolverOptions& op) { m_options = op; }

  virtual void GetMatrixData(const MatrixSd& mA,
                             Real& normA,
                             Real& sinT,
                             Real& regT);

  virtual LSResult Solve(MatrixSd& mA, const VectorXd& vb, VectorXd& vx);
  virtual LSResult Solve(MatrixSd& mA, const MatrixXd& mB, MatrixXd& mX);
  virtual LSResult Solve(MatrixSd& mA, const MatrixSd& mB, MatrixSd& mX);

 protected:
  virtual LSResult SolveInternal(MatrixSd& mA,
                                 const VectorXd& vb,
                                 VectorXd& vx) = 0;
  virtual LSResult SolveInternal(MatrixSd& mA,
                                 const MatrixXd& mB,
                                 MatrixXd& mX) = 0;
  virtual LSResult SolveInternal(MatrixSd& mA,
                                 const MatrixSd& mB,
                                 MatrixSd& mX) = 0;

  virtual void FreeInternal();
};
}  // namespace PhySim
