//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Face_Poly.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Face_Poly::Face_Poly(int id, const vector<Node*>& m_vnodes)
    : Face(id, m_vnodes) {
  // Nothing to do here...
}

Face_Poly::~Face_Poly(void) {
  // Nothing to do here...
}

Real Face_Poly::VolumeSpace(Tag s) const {
  throw exception("Not implemented");

  return -1;
}

}  // namespace PhySim