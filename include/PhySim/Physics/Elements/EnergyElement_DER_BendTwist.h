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

class EnergyElement_DER_BendTwist : public EnergyElement {
 protected:
  vector<Edge*> m_vedges;

  Real m_twist0;
  Vector2d m_bend0;
  MatrixXd m_mDeDx;
  Simulable_DER* m_pModelDER;

 public:
  EnergyElement_DER_BendTwist(Simulable* pModel, const vector<Edge*>& vedges);
  virtual ~EnergyElement_DER_BendTwist(void);

  virtual string GetName() const override { return "[DERBendTwist]"; };

  virtual void Init();

  virtual void ComputeAndStore_Energy_Internal();
  virtual void ComputeAndStore_Gradient_Internal();
  virtual void ComputeAndStore_Hessian_Internal();

  virtual Vector2d ComputeBendStrain(Tag s = Tag_Position_X);
  virtual Real ComputeTwistStrain(Tag s = Tag_Position_X);

  inline virtual Vector2d GetRestBend() const { return this->m_bend0; }
  inline virtual void SetRestBend(Vector2d b0) { this->m_bend0 = b0; }

  inline virtual Real GetRestTwist() const { return this->m_twist0; }
  inline virtual void SetRestTwist(Real t0) { this->m_twist0 = t0; }
};
}  // namespace PhySim
