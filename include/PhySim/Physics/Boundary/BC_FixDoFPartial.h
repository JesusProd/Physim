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

#include <PhySim/Physics/Boundary/BCondition.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class BC_FixDoFPartial : public BCondition {
 protected:
  vector<IDoFSet*> m_vpDoF;
  vector<bVector> m_vFixed;

 public:
  BC_FixDoFPartial(Simulable* pModel,
                   const vector<IDoFSet*>& vpDoF,
                   const vector<VectorXd>& vt = vector<VectorXd>(),
                   const vector<bVector>& vFixed = vector<bVector>());
  virtual ~BC_FixDoFPartial(void);

  inline virtual string Name() const { return "PositionFix"; };

  virtual vector<IDoFSet*>& DoFs() { return this->m_vpDoF; }
  virtual vector<bVector>& Fixed() { return this->m_vFixed; }

  virtual void Init() override;
  virtual void Update() override;
};
}  // namespace PhySim
