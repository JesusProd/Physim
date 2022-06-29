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

namespace PhySim {
using namespace std;
using namespace Eigen;

class IConstraintSet {
 public:
  enum Type { EQ, LEQ, LE };

  IConstraintSet(){};
  virtual ~IConstraintSet(){};

  virtual void Init() = 0;

  virtual const bool& IsSoft() const = 0;
  virtual const Type& GetType() const = 0;
  virtual const int& GetSize() const = 0;

  virtual int GetSupportSize() const = 0;

  virtual int& Offset() = 0;
  virtual bool IsActive() = 0;

  virtual void UpdateKinematics() = 0;
  virtual void UpdateMechanics() = 0;
  virtual void ProjectConstraint() = 0;
  virtual void ComputeAndStore_Constraint() = 0;
  virtual void ComputeAndStore_Jacobian() = 0;
  virtual void ComputeAndStore_Hessian() = 0;

  virtual void AssembleGlobal_Values(VectorXd& vtotalVector) = 0;
  virtual void AssembleGlobal_Jacobian(VectorTd& vtotalTriplets) = 0;
  virtual int GetJacobianSize() const = 0;

  virtual const VectorXd& GetConstraint() const = 0;
  virtual const MatrixXd& GetJacobian() const = 0;
  virtual const vector<MatrixXd>& GetLocalHessian() const = 0;
};

}  // namespace PhySim