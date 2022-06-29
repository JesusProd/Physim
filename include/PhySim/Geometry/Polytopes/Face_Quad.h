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

#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Face_Quad : public Face {
 public:
  Face_Quad(int ID, const vector<Node*>& vnodes);
  virtual ~Face_Quad(void);

  virtual bool IsValidParametric(const VectorXd& vp) const override;
  virtual void TransformNat2Iso(VectorXd& vb,
                                MatrixXd& mA,
                                Tag s) const override;
  virtual void TransformIso2Nat(VectorXd& vb,
                                MatrixXd& mA,
                                Tag s) const override;
  virtual Real VolumeSpace(Tag s) const;
};

class Face_Quad4 : public Face_Quad {
 public:
  Face_Quad4(int ID, const vector<Node*>& vnodes) : Face_Quad(ID, vnodes) {
    this->m_pSFunction = ShapeFunction_Quad4::Instance();
  }

  virtual void InitSubelementPositions(Tag s) override {}
};

class Face_Quad8 : public Face_Quad {
 public:
  Face_Quad8(int ID, const vector<Node*>& vnodes) : Face_Quad(ID, vnodes) {
    this->m_pSFunction = ShapeFunction_Quad8::Instance();
  }

  virtual void InitSubelementPositions(Tag s) override {
    MatrixXd mN;
    this->GetNodesTrait(mN, s);
    this->m_vnodes[4]->Traits().Vector3d(s) = 0.5 * (mN.row(0) + mN.row(1));
    this->m_vnodes[5]->Traits().Vector3d(s) = 0.5 * (mN.row(1) + mN.row(2));
    this->m_vnodes[6]->Traits().Vector3d(s) = 0.5 * (mN.row(2) + mN.row(3));
    this->m_vnodes[7]->Traits().Vector3d(s) = 0.5 * (mN.row(3) + mN.row(4));
  }
};
}  // namespace PhySim