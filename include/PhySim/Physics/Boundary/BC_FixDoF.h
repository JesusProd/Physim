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

class BC_FixDoF : public BCondition {
 protected:
  vector<IDoFSet*> m_vpDoF;
  bool m_isSoft;
  Real m_kAini;
  Real m_kAend;

 public:
  BC_FixDoF(Simulable* pModel,
            const vector<IDoFSet*>& vpDoF,
            const vector<VectorXd>& vt = vector<VectorXd>(),
            bool isSoft = false,
            Real kSini = 1e9,
            Real kSend = 1e9);
  virtual ~BC_FixDoF(void);

  inline virtual string Name() const { return "PositionFix"; };

  virtual const bool& IsSoft() { return this->m_isSoft; }
  virtual Real& KStart() { return this->m_kAini; }
  virtual Real& KFinal() { return this->m_kAend; }
  virtual vector<IDoFSet*>& DoFs() { return this->m_vpDoF; }

  virtual void Init() override;
  virtual void Update() override;
};
}  // namespace PhySim
