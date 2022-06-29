//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Face_Quad.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Face_Quad::Face_Quad(int id, const vector<Node*>& m_vnodes)
    : Face(id, m_vnodes) {
  // Nothing to do here...
}

Face_Quad::~Face_Quad(void) {
  // Nothing to do here...
}

Real Face_Quad::VolumeSpace(Tag s) const {
  Vector3d e01 = this->m_vnodes[1]->Traits().Vector3d(s) -
                 this->m_vnodes[0]->Traits().Vector3d(s);
  Vector3d e03 = this->m_vnodes[3]->Traits().Vector3d(s) -
                 this->m_vnodes[0]->Traits().Vector3d(s);
  return e01.cross(e03).norm();
}

bool Face_Quad::IsValidParametric(const VectorXd& vp) const {
  if (vp.size() != 2)
    return false;

  if (vp[0] < -1 || vp[0] > 1)
    return false;

  if (vp[1] < -1 || vp[1] > 1)
    return false;

  return true;
}

void Face_Quad::TransformNat2Iso(VectorXd& vb, MatrixXd& mA, Tag s) const {
  MatrixXd mNB;
  mNB.resize(3, 3);
  Vector3d e0 = this->m_vnodes[1]->Traits().Vector3d(s) -
                this->m_vnodes[0]->Traits().Vector3d(s);
  Vector3d e1 = this->m_vnodes[3]->Traits().Vector3d(s) -
                this->m_vnodes[0]->Traits().Vector3d(s);
  mNB.col(0) = e0;
  mNB.col(1) = e1;
  mNB.col(2) = e0.cross(e1);
  mA = mNB.inverse().block(0, 0, 2, 3);
  vb = -mA * this->Centroid(s);
}

void Face_Quad::TransformIso2Nat(VectorXd& vb, MatrixXd& mA, Tag s) const {
  mA.resize(3, 2);
  mA.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) -
              this->m_vnodes[0]->Traits().Vector3d(s);
  mA.col(1) = this->m_vnodes[3]->Traits().Vector3d(s) -
              this->m_vnodes[0]->Traits().Vector3d(s);
  vb = this->Centroid(s);
}

}  // namespace PhySim