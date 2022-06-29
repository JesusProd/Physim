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

#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Elements/EnergyElement.h>
#include <PhySim/Utils/ParameterSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Node;

class EnergyElement_FaceNodeColl_Step : public EnergyElement {
 protected:
  ParameterSet* m_pMaterial;

 public:
  EnergyElement_FaceNodeColl_Step(Simulable* pModel,
                                  Face* pFace,
                                  Node* pNode,
                                  ParameterSet* pMaterial)
      : EnergyElement(pModel) {
    this->m_vDoF.resize(4);
    this->m_vDoF[0] = &pFace->Nodes()[0]->Trait<IDoFSet>(Tag::Tag_DOF_0);
    this->m_vDoF[1] = &pFace->Nodes()[1]->Trait<IDoFSet>(Tag::Tag_DOF_0);
    this->m_vDoF[2] = &pFace->Nodes()[2]->Trait<IDoFSet>(Tag::Tag_DOF_0);
    this->m_vDoF[3] = &pNode->Trait<IDoFSet>(Tag::Tag_DOF_0);
    m_pMaterial = pMaterial;
  }

  virtual ~EnergyElement_FaceNodeColl_Step(void) {
    // Nothing to do here...
  }

  virtual void ComputeAndStore_Energy_Internal() {
    const Vector3d& x0 =
        this->m_vDoF[0]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X);
    const Vector3d& x1 =
        this->m_vDoF[1]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X);
    const Vector3d& x2 =
        this->m_vDoF[2]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X);
    const Vector3d& x3 =
        this->m_vDoF[3]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X);
    Vector3d e0 = (x1 - x0).normalized();
    Vector3d e1 = (x2 - x0).normalized();
    Vector3d e3 = x3 - x0;
    const Real& T = (*m_pMaterial)[ParameterSet::Param_CollT];
    this->m_energy = (e0.cross(e1).dot(e3) > T) ? 0 : HUGE_VAL;
  }
};
}  // namespace PhySim
