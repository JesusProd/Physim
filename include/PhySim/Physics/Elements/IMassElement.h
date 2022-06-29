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

#include <PhySim/Physics/Elements/EnergyElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Simulable;

class IMassElement : public EnergyElement {
 public:
  IMassElement(Simulable* pModel) : EnergyElement(pModel) {}
  virtual ~IMassElement(){};

  virtual void Init() = 0;

  virtual const Vector3d& GetGravity() const = 0;
  virtual void SetGravity(const Vector3d& vg) = 0;

  virtual void ComputeAndStore_Mass() = 0;

  virtual void AssembleGlobal_Mass(AMatrixSd& mMass) = 0;
  virtual void AssembleGlobal_DMDtv(AVectorXd& vDMDtv) = 0;
};
}  // namespace PhySim
