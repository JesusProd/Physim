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

#include <PhySim/Physics/Elements/IMassElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class ParameterSet;
class Simulable;

class MassElement : public IMassElement {
 protected:
  MatrixXd m_mMass;
  VectorXd m_vDMDtv;
  Vector3d m_vgravity;
  PtrS<ParameterSet> m_pParams;

 public:
  MassElement(Simulable* pModel, PtrS<ParameterSet> pParams);

  virtual ~MassElement(void);

  virtual void Init();

  virtual void ComputeAndStore_Mass() = 0;

  virtual const Vector3d& GetGravity() const override {
    return this->m_vgravity;
  }
  virtual void SetGravity(const Vector3d& vg) override {
    this->m_vgravity = vg;
  }

  virtual void AssembleGlobal_Mass(AMatrixSd& mMass) override;
  virtual void AssembleGlobal_DMDtv(AVectorXd& vDMDtv) override;
};
}  // namespace PhySim
