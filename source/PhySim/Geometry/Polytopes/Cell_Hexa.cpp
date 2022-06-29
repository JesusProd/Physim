//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Cell_Hexa::Cell_Hexa(int id, const vector<Node*>& m_vnodes)
    : Cell(id, m_vnodes) {
  // Nothing to do here...
}

Cell_Hexa::~Cell_Hexa(void) {
  // Nothing to do here...
}

Real Cell_Hexa::VolumeSpace(Tag s) const {
  // NOTE: This assumes the hexahedron is regular
  // TODO: General hexahedron volume calculation

  Vector3d e01 = this->m_vnodes[1]->Traits().Vector3d(s) -
                 this->m_vnodes[0]->Traits().Vector3d(s);
  Vector3d e03 = this->m_vnodes[3]->Traits().Vector3d(s) -
                 this->m_vnodes[0]->Traits().Vector3d(s);
  Vector3d e04 = this->m_vnodes[4]->Traits().Vector3d(s) -
                 this->m_vnodes[0]->Traits().Vector3d(s);
  return e01.cross(e03).dot(e04);
}

void Cell_Hexa::TransformNat2Iso(VectorXd& b, MatrixXd& A, Tag s) const {
  // NOTE: This assumes the hexahedron is regular
  // TODO: General hexahedron mapping calculation

  MatrixXd mNB;
  mNB.resize(3, 3);
  mNB.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) -
               this->m_vnodes[0]->Traits().Vector3d(s);
  mNB.col(1) = this->m_vnodes[3]->Traits().Vector3d(s) -
               this->m_vnodes[0]->Traits().Vector3d(s);
  mNB.col(2) = this->m_vnodes[4]->Traits().Vector3d(s) -
               this->m_vnodes[0]->Traits().Vector3d(s);
  A = 2 * mNB.inverse();
  b = -A * this->Centroid(s);
}

void Cell_Hexa::TransformIso2Nat(VectorXd& b, MatrixXd& A, Tag s) const {
  // NOTE: This assumes the hexahedron is regular
  // TODO: General hexahedron mapping calculation

  A.resize(3, 3);
  A.col(0) = 0.5 * this->m_vnodes[1]->Traits().Vector3d(s) -
             this->m_vnodes[0]->Traits().Vector3d(s);
  A.col(1) = 0.5 * this->m_vnodes[3]->Traits().Vector3d(s) -
             this->m_vnodes[0]->Traits().Vector3d(s);
  A.col(2) = 0.5 * this->m_vnodes[4]->Traits().Vector3d(s) -
             this->m_vnodes[0]->Traits().Vector3d(s);
  b = this->Centroid(s);
}

}  // namespace PhySim