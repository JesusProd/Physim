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

#include <PhySim/Physics/Elements/IConstraintSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Simulable;
class KinematicsEle;

class ConstraintSet : public IConstraintSet {
 protected:
  Simulable* m_pModel;

  Type m_type;
  int m_size;
  int m_index;
  int m_offset;
  bool m_soft;

  vector<KinematicsEle*> m_vDoFs;

  VectorXd m_vvalues;           // Local element constraint
  MatrixXd m_mJacobian;         // Local element Jacobian
  vector<MatrixXd> m_vHessian;  // Local element Hessians

 public:
  ConstraintSet(Simulable* pModel, bool soft);

  virtual ~ConstraintSet(void);

  virtual void Init() {}

  virtual const bool& IsSoft() const override { return this->m_soft; }
  virtual const Type& GetType() const override { return this->m_type; };
  virtual const int& GetSize() const override { return this->m_size; };

  virtual int& Offset() override { return this->m_offset; }
  virtual bool IsActive() override;

  inline virtual int GetSupportSize() const override;

  inline virtual const vector<KinematicsEle*> GetSupportDoF() const {
    return this->m_vDoFs;
  }

  virtual void UpdateKinematics() override {}
  virtual void UpdateMechanics() override {}
  virtual void ProjectConstraint() override {}
  virtual void ComputeAndStore_Constraint() override {}
  virtual void ComputeAndStore_Jacobian() override {}
  virtual void ComputeAndStore_Hessian() override {}

  virtual void AssembleGlobal_Values(VectorXd& vtotalVector) override;
  virtual void AssembleGlobal_Jacobian(VectorTd& vtotalTriplets) override;

  virtual int GetJacobianSize() const override {
    return this->GetSize() * this->GetSupportSize();
  };

  inline const VectorXd& GetConstraint() const override {
    return this->m_vvalues;
  }
  inline const MatrixXd& GetJacobian() const override {
    return this->m_mJacobian;
  }
  inline const vector<MatrixXd>& GetLocalHessian() const override {
    return this->m_vHessian;
  }
};
}  // namespace PhySim
