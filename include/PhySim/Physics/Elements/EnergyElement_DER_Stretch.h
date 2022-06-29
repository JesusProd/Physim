//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, IST Austria
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Physics/Elements/EnergyElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Edge;
class Simulable_DER;

class EnergyElement_DER_Stretch : public EnergyElement {
 protected:
  Edge* m_pEdge;
  MatrixXd m_mDeDx;
  Simulable_DER* m_pModelDER;

 public:
  EnergyElement_DER_Stretch(Simulable* pModel, Edge* pPoly);
  virtual ~EnergyElement_DER_Stretch(void);

  virtual string GetName() const override { return "[DERStretch]"; };

  virtual void Init();

  virtual void ComputeAndStore_Energy_Internal();
  virtual void ComputeAndStore_Gradient_Internal();
  virtual void ComputeAndStore_Hessian_Internal();

  virtual Real ComputeLength(Tag s = Tag::Position_X);

  inline virtual Real GetRestLength() const { return this->m_intVolume; }
  inline virtual void SetRestLength(Real l0) { this->m_intVolume = l0; }
};
}  // namespace PhySim
