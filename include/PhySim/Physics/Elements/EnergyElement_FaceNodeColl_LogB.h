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

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Elements/EnergyElement.h>
#include <PhySim/Utils/ParameterSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Node;

class EnergyElement_FaceNodeColl_LogB : public EnergyElement {
 protected:
  Face* m_pFace;
  Node* m_pEmbedding;

 public:
  EnergyElement_FaceNodeColl_LogB(Simulable* pModel, Face* pFace, Node* pNode)
      : EnergyElement(pModel) {
    this->m_pFace = pFace;
    this->m_pEmbedding = pNode;

    this->m_vDoF.resize(4);
    this->m_vDoF[0] = &pFace->Nodes()[0]->Trait<IDoFSet>(Tag::DOF_0);
    this->m_vDoF[1] = &pFace->Nodes()[1]->Trait<IDoFSet>(Tag::DOF_0);
    this->m_vDoF[2] = &pFace->Nodes()[2]->Trait<IDoFSet>(Tag::DOF_0);
    this->m_vDoF[3] = &pNode->Trait<IDoFSet>(Tag::DOF_0);
  }

  virtual ~EnergyElement_FaceNodeColl_LogB(void) {
    // Nothing to do here...
  }

  virtual void ComputeAndStore_Energy_Internal() {
    // TODO
  }

  virtual void ComputeAndStore_Gradient_Internal() {
    // TODO
  }

  virtual void ComputeAndStore_Hessian_Internal() {
    // TODO
  }
};
}  // namespace PhySim