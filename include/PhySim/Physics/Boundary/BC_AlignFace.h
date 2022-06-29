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

class BC_AlignFace : public BCondition {
 protected:
  vector<Face*> m_vpFace;
  bool m_isSoft;
  Real m_kAini;
  Real m_kAend;

 public:
  BC_AlignFace(Simulable* pModel,
               const vector<Face*>& vpFaces,
               const vector<Vector3d>& vn = vector<Vector3d>(),
               bool isSoft = true,
               Real kSini = 1e9,
               Real kSend = 1e9);
  virtual ~BC_AlignFace(void);

  inline virtual string Name() const { return "AlignFace"; };

  virtual vector<Face*>& Faces() { return this->m_vpFace; }
  virtual Real& KStart() { return this->m_kAini; }
  virtual Real& KFinal() { return this->m_kAend; }

  virtual void Init() override;
  virtual void Update() override;
};
}  // namespace PhySim
