//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/Solvers/OptimProblem.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class ISimulable;
struct OptimState;

class OptimProblem_BasicStatic : public OptimProblem {
 protected:
  ISimulable* m_pM;
  VectorXd m_pS;

 public:
  OptimProblem_BasicStatic(ISimulable* pM);
  virtual ~OptimProblem_BasicStatic();

  virtual void Init(ISimulable* pM);

  ISimulable* getSimulable() { return m_pM; }

  void GetVariables(VectorXd& vx) const override;
  void IncVariables(const VectorXd& vx) override;
  void SetVariables(const VectorXd& vx) override;

  bool GetEnergy(Real& e) override;
  bool GetGradient(AVectorXd& vg) override;
  bool GetHessian(AMatrixSd& mH) override;

  bool OnIterStart(const OptimState& state) override;
  bool OnIterFinish(const OptimState& state) override;
  bool OnSolveStart(const OptimState& state) override;
  bool OnSolveFinish(const OptimState& state) override;
  bool PrePerformStep(const OptimState& state) override;
  bool PosPerformStep(const OptimState& state) override;

  bool IsFullyConstrained() override;
};
}  // namespace PhySim