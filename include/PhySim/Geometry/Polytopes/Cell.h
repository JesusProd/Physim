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

#include <PhySim/Geometry/Polytopes/Poly.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Cell : public Poly {
 public:
  Cell(int ID, const vector<Node*>& vnodes);
  virtual ~Cell(void);

  inline int DimBasis() const override { return 3; }
};
}  // namespace PhySim
