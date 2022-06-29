//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Edge.h>

#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Edge::Edge(int ID, const vector<Node*>& m_vnodes) : Poly(ID) {
  this->m_vnodes = m_vnodes;
}

Edge::~Edge(void) {
  // Nothing to do here...
}

inline Real Edge::VolumeSpace(Tag s) const {
  return this->Vector(s).norm();
}

inline Vector3d Edge::Vector(Tag s) const {
  return (m_vnodes[1]->Traits().Vector3d(s) -
          m_vnodes[0]->Traits().Vector3d(s));
}

inline Vector3d Edge::Tangent(Tag s) const {
  return this->Vector(s).normalized();
}

void Edge::TransformNat2Iso(VectorXd& b, MatrixXd& A, Tag s) const {
  Vector3d vr = Vector3d::Random();
  Vector3d e0 = this->Vector(s);
  Vector3d e1 = e0.cross(vr);
  Vector3d e2 = e1.cross(e0);
  MatrixXd mNB;
  mNB.resize(3, 3);
  mNB.col(0) = e0;
  mNB.col(1) = e1;
  mNB.col(2) = e2;
  A = mNB.inverse().block(0, 0, 1, 3);
  b = -A * m_vnodes[0]->Traits().Vector3d(s);
}

void Edge::TransformIso2Nat(VectorXd& b, MatrixXd& A, Tag s) const {
  A.resize(3, 1);
  A.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) -
             this->m_vnodes[0]->Traits().Vector3d(s);
  b = this->m_vnodes[0]->Traits().Vector3d(s);
}

}  // namespace PhySim