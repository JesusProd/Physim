//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Cell.h>

#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Cell::Cell(int ID, const vector<Node*>& m_vnodes) : Poly(ID) {
  this->m_vnodes = m_vnodes;
}

Cell::~Cell(void) {
  // Nothing to do here...
}

}  // namespace PhySim