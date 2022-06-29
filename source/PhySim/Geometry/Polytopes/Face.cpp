//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Face.h>

#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Face::Face(int ID, const vector<Node*>& m_vnodes) : Poly(ID) {
  this->m_vnodes = m_vnodes;
}

Face::~Face(void) {
  // Nothing to do here...
}

void Face::TransformNat2Bas(VectorXd& b, MatrixXd& A, Tag s) const {
  throw new exception("Not implemented");
}

void Face::TransformBas2Nat(VectorXd& b, MatrixXd& A, Tag s) const {
  throw new exception("Not implemented");
}

}  // namespace PhySim