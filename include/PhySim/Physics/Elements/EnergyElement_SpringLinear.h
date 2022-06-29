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

class Edge;
class Simulable;
class ParameterSet;

class EnergyElement_SpringLinear : public EnergyElement {
 protected:
  Edge* m_pEdge;
  Real m_restLength;
  ParameterSet* m_pMaterial;

 public:
  EnergyElement_SpringLinear(Simulable* pModel,
                             Edge* pEdge,
                             ParameterSet* pMaterial);
  virtual ~EnergyElement_SpringLinear(void);

  virtual void Init();

  virtual Real ComputeRestLength();
  inline virtual Real GetRestLength() const { return this->m_restLength; }
  inline virtual void SetRestLength(Real rl) { this->m_restLength = rl; }

  // This element reimplements gradient and HessianFull computation
  // to take advantage of the symmetries of the gradient and the
  // HessianFull to optimize assembly.

  // virtual void AssembleGlobal_Gradient(VectorXd& vtotalGradient);
  // virtual void AssembleGlobal_Hessian(VectorTd & vtotalHessian);

  // virtual void AssembleGlobal_FastPreallocatedHessian();

  virtual void ComputeAndStore_Energy_Internal();
  virtual void ComputeAndStore_Gradient_Internal();
  virtual void ComputeAndStore_Hessian_Internal();

  inline virtual ParameterSet* GetMaterial() { return this->m_pMaterial; }
};
}  // namespace PhySim
