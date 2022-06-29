//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_PlaneColl.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Physics/Elements/ConstraintSet_NodePlaneColl.h>
#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

BC_PlaneColl::BC_PlaneColl(Simulable* pModel,
                           const Vector3d& vp,
                           const Vector3d& vn,
                           double tol,
                           bool isSoft)
    : BCondition(pModel) {
  this->m_vpoints = vp;
  this->m_vn = vn;
  m_vn.normalize();
  this->m_tol = tol;
  this->m_isSoft = isSoft;

  this->m_vini.resize(2);
  this->m_vini[0] = m_vpoints;
  this->m_vini[1] = m_vn;

  this->m_pFilter.reset(new GeometryFilter_PlaneDistNode(
      Tag_Position_X, vp, vn, Vector2d(-HUGE_VAL, tol)));
}

BC_PlaneColl::~BC_PlaneColl(void) {
  // Nothing to do here...
}

void BC_PlaneColl::Init() {
  // Nothing to do here...
}

void BC_PlaneColl::Update() {
  vector<Geometry*> vnodes = m_pFilter->Filter(this->m_pModel->Geometries()[0]);

  this->m_vpCon.clear();
  this->m_vpEne.clear();

  for (int i = 0; i < vnodes.size(); ++i) {
    KEleParticle3D* pDOF = vnodes[i]->Trait<KEleParticle3D*>(Tag::Tag_DOF_0);

    PtrS<ConstraintSet> pCon(new ConstraintSet_NodePlaneColl(
        m_pModel, this->m_isSoft, pDOF, this->m_vpoints, this->m_vn, m_tol));
    if (this->m_isSoft)
      m_vpEne.push_back(PtrS<IEnergyElement>(
          new EnergyElement_SoftConstraint(this->m_pModel, pCon, 1.0e9)));
    else
      this->m_vpCon.push_back(pCon);
  }
}
}  // namespace PhySim
