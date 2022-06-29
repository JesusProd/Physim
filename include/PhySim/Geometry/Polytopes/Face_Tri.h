//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC Madrid
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Face_Tri : public Face {
 public:
  Face_Tri(int ID, const vector<Node*>& vnodes);
  ~Face_Tri(void);

  inline Node* GetNode0(void) const { return m_vnodes[0]; }
  inline void SetNode0(Node* node) { m_vnodes[0] = node; }

  inline Node* GetNode1(void) const { return m_vnodes[1]; }
  inline void SetNode1(Node* node) { m_vnodes[1] = node; }

  inline Node* GetNode2(void) const { return m_vnodes[2]; }
  inline void SetNode2(Node* node) { m_vnodes[2] = node; }

  virtual Vector3d Normal(Tag s) const override;

  virtual Real VolumeSpace(Tag s) const override;

  virtual void TransformNat2Iso(VectorXd& b, MatrixXd& A, Tag s) const override;
  virtual void TransformIso2Nat(VectorXd& b, MatrixXd& A, Tag s) const override;

  virtual void TransformNat2Bas(VectorXd& b, MatrixXd& A, Tag s) const override;
  virtual void TransformBas2Nat(VectorXd& b, MatrixXd& A, Tag s) const override;
};

class Face_Tri3 : public Face_Tri {
 public:
  Face_Tri3(int ID, const vector<Node*>& vverts) : Face_Tri(ID, vverts) {
    this->m_pSFunction = ShapeFunction_Tri3::Instance();
  }

  virtual void InitSubelementPositions(Tag s) override {}
};

class Face_Tri6 : public Face_Tri {
 public:
  Face_Tri6(int ID, const vector<Node*>& vverts) : Face_Tri(ID, vverts) {
    this->m_pSFunction = ShapeFunction_Tri6::Instance();
  }

  virtual void InitSubelementPositions(Tag s) override {
    MatrixXd mN;
    this->GetNodesTrait(mN, s);
    this->m_vnodes[4]->Traits().Vector3d(s) = 0.5 * (mN.row(0) + mN.row(1));
    this->m_vnodes[5]->Traits().Vector3d(s) = 0.5 * (mN.row(1) + mN.row(2));
    this->m_vnodes[6]->Traits().Vector3d(s) = 0.5 * (mN.row(0) + mN.row(2));
  }
};

}  // namespace PhySim
