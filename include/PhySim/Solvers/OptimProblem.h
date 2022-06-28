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

#include <PhySim/BasicTypes.h>

#include <PhySim/Utils/CustomTimer.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

struct OptimState {
  int m_N;         // Variable number
  int m_M;         // Constraint number
  VectorXd m_vx;   // Solution vector
  VectorXd m_v0;   // Previous solution
  VectorXd m_vc;   // Constraints vector
  VectorXd m_vl;   // Lambda vector
  AVectorXd m_vg;  // Gradient vector
  AMatrixSd m_mH;  // Hessian matrix
  Real m_obj;      // Optimization objective
  Real m_opt;      // Gradient squared norm
  Real m_fea;      // Constraints squared norm

  int m_countIt;  // Iteration counter

  int m_minImprCount;  // Mininum improvemente count
  int m_minStepCount;  // Minimum step size count
  bool m_steepest;     // Falling back to steepest

  OSResult m_result;

  OptimState() {
    m_vx.resize(0);
    m_v0.resize(0);
    m_vc.resize(0);
    m_vl.resize(0);
    m_vg.resize(0);
    m_mH.Clear();
    m_obj = 0;
    m_opt = 0;
    m_fea = 0;
    m_countIt = 0;
    m_result = OSResult::OR_ONGOING;
    m_minImprCount = 0;
    m_minStepCount = 0;
    m_steepest = false;
  }
};

class IOptimProblem {
 public:
  IOptimProblem(){};
  virtual ~IOptimProblem(){};

  virtual const string& GetName() const = 0;

  virtual int GetNumVariables() const = 0;
  virtual int GetNumConstraints() const = 0;

  virtual void GetVariables(VectorXd& vx) const = 0;
  virtual void IncVariables(const VectorXd& vx) = 0;
  virtual void SetVariables(const VectorXd& vx) = 0;

  virtual void GetUpperBound(VectorXd& vub) const = 0;
  virtual void GetLowerBound(VectorXd& vlb) const = 0;

  virtual bool GetEnergy(Real& e) = 0;
  virtual bool GetGradient(AVectorXd& vg) = 0;
  virtual bool GetHessian(AMatrixSd& mH) = 0;

  virtual bool GetConstraint(VectorXd& vc) = 0;
  virtual bool GetJacobian(MatrixSd& mJ) = 0;

  virtual bool IsFullyConstrained() = 0;

  // Callbacks

  virtual bool OnSolveStart(const OptimState& state) = 0;
  virtual bool OnSolveFinish(const OptimState& state) = 0;

  virtual bool OnIterStart(const OptimState& state) = 0;
  virtual bool OnIterFinish(const OptimState& state) = 0;

  virtual bool PrePerformStep(const OptimState& state) = 0;
  virtual bool PosPerformStep(const OptimState& state) = 0;
};

class OptimProblem : public IOptimProblem {
 protected:
  int m_N;
  int m_M;
  string m_name;
  CustomTimer m_timerComputeObj;
  CustomTimer m_timerComputeGrad;
  CustomTimer m_timerComputeHess;

 public:
  OptimProblem();

  const string& GetName() const override { return this->m_name; }

  int GetNumVariables() const override { return this->m_N; }
  int GetNumConstraints() const override { return this->m_M; }

  void GetVariables(VectorXd& vx) const override = 0;
  void IncVariables(const VectorXd& vx) override = 0;
  void SetVariables(const VectorXd& vx) override = 0;

  void GetUpperBound(VectorXd& vub) const override;
  void GetLowerBound(VectorXd& vlb) const override;

  bool GetEnergy(Real& e) override;
  bool GetGradient(AVectorXd& vg) override;
  bool GetHessian(AMatrixSd& mH) override;

  bool GetConstraint(VectorXd& vc) override;
  bool GetJacobian(MatrixSd& mJ) override;

  virtual const CustomTimer& GetTimer_ComputeObjective() const {
    return this->m_timerComputeObj;
  }
  virtual const CustomTimer& GetTimer_ComputeGradient() const {
    return this->m_timerComputeGrad;
  }
  virtual const CustomTimer& GetTimer_ComputeHessian() const {
    return this->m_timerComputeHess;
  }

  bool IsFullyConstrained() override { return true; }

  bool OnSolveStart(const OptimState& state) override {
    // Default implementation, do nothing
    return true;
  };

  bool OnIterStart(const OptimState& state) override {
    // Default implementation, do nothing
    return true;
  };

  bool OnSolveFinish(const OptimState& state) override {
    // Default implementation, do nothing
    return true;
  };

  bool OnIterFinish(const OptimState& state) override {
    // Default implementation, do nothing
    return true;
  };

  bool PrePerformStep(const OptimState& state) override {
    // Default implementation, do nothing
    return true;
  };

  bool PosPerformStep(const OptimState& state) override {
    // Default implementation, do nothing
    return true;
  };

 protected:
  void Init() {
    // Real e;
    // VectorXd vg0;
    // AMatrixSd mH0;
    // this->GetEnergy(e);
    // this->GetGradient(vg0);
    // this->GetHessian(mH0);

    // logTrace(Verbosity::V1_Default, "\n--");
    // logTrace(Verbosity::V1_Default, "\nINITIALIZED: Problem %s",
    // this->GetName().c_str()); logTrace(Verbosity::V1_Default, "\nVariable
    // number: %i", this->m_N); logTrace(Verbosity::V1_Default, "\nConstraint
    // number: %i", this->m_M); logTrace(Verbosity::V1_Default, "\nInitial
    // energy: %f", e); logTrace(Verbosity::V1_Default, "\nInitial Gradient norm:
    // %f", vg0.norm()); logTrace(Verbosity::V1_Default, "\nInitial Hessian norm:
    // %f", mH0.norm()); if (this->m_M > 0)
    //{
    //	VectorXd vc0;
    //	MatrixSd mJ0;
    //	this->GetConstraint(vc0);
    //	this->GetJacobian(mJ0);
    //	logTrace(Verbosity::V1_Default, "\nInitial Constraints norm: %f",
    //vc0.norm()); 	logTrace(Verbosity::V1_Default, "\nInitial Jacobian norm:
    //%f", mJ0.norm());
    //}
    // logTrace(Verbosity::V1_Default, "\n--");
  }
};
}  // namespace PhySim