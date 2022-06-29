//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_AlignFace.h>

#include <PhySim/Physics/Boundary/BCondition.h>
#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Geometry/Polytopes/Face_Quad.h>
#include <PhySim/Geometry/Polytopes/Face_Tri.h>

#include <PhySim/Physics/Elements/ConstraintSet_AlignFace.h>
#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

BC_AlignFace::BC_AlignFace(Simulable* pModel,
                           const vector<Face*>& vpFaces,
                           const vector<Vector3d>& vn,
                           bool isSoft,
                           Real kAini,
                           Real kAend)
    : BCondition(pModel) {
  this->m_vpFace = vpFaces;
  this->m_kAini = kAini;
  this->m_kAend = kAend;
  this->m_isSoft = isSoft;

  // Initialize values to hold current position

  int numP = (int)vpFaces.size();
  this->m_vini.resize(numP);
  for (int i = 0; i < numP; ++i) {
    if (!vn.empty())
      this->m_vini[i] = vn[i];
    else
      this->m_vini[i] = vpFaces[i]->Normal(Tag::Position_X);
  }
}

BC_AlignFace::~BC_AlignFace(void) {
  // Nothing to do here...
}

void BC_AlignFace::Init() {
  int numP = (int)m_vpFace.size();

  if (this->m_isSoft) {
    this->m_vpEne.reserve(numP);

    for (size_t i = 0; i < numP; ++i) {
      PtrS<ConstraintSet> pCon(
          new ConstraintSet_AlignFace(m_pModel, m_vpFace[i], m_vini[i], true));
      PtrS<EnergyElement> pEne(
          new EnergyElement_SoftConstraint(m_pModel, pCon, this->m_kAini));
      this->m_vpCon.push_back(pCon);
      this->m_vpEne.push_back(pEne);
    }
  } else {
    this->m_vpCon.reserve(numP);

    for (size_t i = 0; i < numP; ++i) {
      this->m_vpCon.push_back(PtrS<ConstraintSet>(new ConstraintSet_AlignFace(
          m_pModel, m_vpFace[i], m_vini[i], false)));
    }
  }
}

void BC_AlignFace::Update() {
  vector<VectorXd> vval;
  this->GetCurValues(vval);

  Real a = ((Real)CurStage() / (Real)NumStage());
  Real kA = m_kAini + a * (m_kAend - m_kAini);

  bool changed = false;

  for (int i = 0; i < (int)this->m_vpCon.size(); ++i) {
    ConstraintSet_AlignFace* pCon =
        static_cast<ConstraintSet_AlignFace*>(this->m_vpCon[i].get());

    if (pCon->Normal() != vval[i]) {
      pCon->Normal() = vval[i];
      changed = true;
    }

    if (this->m_isSoft) {
      EnergyElement_SoftConstraint* pEne =
          static_cast<EnergyElement_SoftConstraint*>(this->m_vpEne[i].get());

      if (pEne->Stiffness() != kA) {
        pEne->Stiffness() = kA;
        changed = true;
      }
    }
  }

  if (changed)
    m_pModel->DirtyMechanics();
}

}  // namespace PhySim
