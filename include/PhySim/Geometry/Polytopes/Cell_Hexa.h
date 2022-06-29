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

#include <PhySim/Geometry/Polytopes/Cell.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Cell_Hexa : public Cell {
 public:
  Cell_Hexa(int ID, const vector<Node*>& mvnodes);

  virtual ~Cell_Hexa(void);

  virtual void TransformNat2Iso(VectorXd& vb,
                                MatrixXd& mA,
                                Tag s) const override;
  virtual void TransformIso2Nat(VectorXd& vb,
                                MatrixXd& mA,
                                Tag s) const override;
  virtual Real VolumeSpace(Tag s) const override;
};

class Cell_Hexa8 : public Cell_Hexa {
 public:
  Cell_Hexa8(int ID, const vector<Node*>& vnodes) : Cell_Hexa(ID, vnodes) {
    this->m_pSFunction = ShapeFunction_Hex8::Instance();
  }

  virtual void InitSubelementPositions(Tag s) override {}
};

}  // namespace PhySim