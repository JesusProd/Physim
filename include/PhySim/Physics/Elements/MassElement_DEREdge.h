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

#include <PhySim/Physics/Elements/MassElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Edge;
class Simulable_DER;
class ParameterSet;

class MassElement_DEREdge : public MassElement {
 protected:
  Simulable_DER* m_pModelDER;

  Edge* m_pEdge;

 public:
  MassElement_DEREdge(Simulable_DER* pModel,
                      Edge* pEdge,
                      PtrS<ParameterSet> pParam);

  virtual ~MassElement_DEREdge(void);

  virtual void Init();

  virtual void ComputeAndStore_Mass() override;

  virtual int GetHessianSize() const override { return 0; };

  virtual Real GetElementMass() const { return this->m_mMass(0, 0); };
};
}  // namespace PhySim
