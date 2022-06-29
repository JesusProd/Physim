#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Physics/Elements/EnergyElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Node;
class Simulable;
class ParameterSet;

class EnergyElement_SpringBetwNodes : public EnergyElement {
 protected:
  Node* m_node1;
  Node* m_node2;
  Real m_restLength;
  PtrS<ParameterSet> m_pMaterial;

 public:
  EnergyElement_SpringBetwNodes(Simulable* pModel,
                                Node* node1,
                                Node* node2,
                                PtrS<ParameterSet> pMaterial);
  virtual ~EnergyElement_SpringBetwNodes(void);

  virtual void Init();

  virtual Real ComputeRestLength();
  inline virtual Real GetRestLength() const { return this->m_restLength; }
  inline virtual void SetRestLength(Real rl) { this->m_restLength = rl; }

  virtual void ComputeAndStore_Energy_Internal();
  virtual void ComputeAndStore_Gradient_Internal();
  virtual void ComputeAndStore_Hessian_Internal();

  inline virtual PtrS<ParameterSet> GetMaterial() { return this->m_pMaterial; }
};
}  // namespace PhySim