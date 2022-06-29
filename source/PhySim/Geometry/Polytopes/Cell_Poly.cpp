//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Cell_Poly.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Cell_Poly::Cell_Poly(int id, const vector<Node*>& m_vnodes)
    : Cell(id, m_vnodes) {
  // Nothing to do here...
}

Cell_Poly::~Cell_Poly(void) {
  // Nothing to do here...
}

Real Cell_Poly::VolumeSpace(Tag s) const {
  throw exception("Not implemented");

  return -1;
}

}  // namespace PhySim