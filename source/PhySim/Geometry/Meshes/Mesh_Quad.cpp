//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Quad.h>

#include <PhySim/Geometry/Polytopes/Face_Quad.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_Quad::Mesh_Quad() {
  this->UpdateMetadata();
}

Mesh_Quad::Mesh_Quad(const Mesh_Quad& toCopy) : Mesh_Face(toCopy) {
  this->UpdateMetadata();
}

Mesh_Quad::Mesh_Quad(const MatrixXd& mV,
                     const MatrixXi& mF,
                     Discretization D,
                     const vector<Tag>& vnTraits)
    : Mesh_Face(mV, mF, D, vnTraits) {
  this->UpdateMetadata();
}

void Mesh_Quad::Init(const MatrixXd& mV,
                     const MatrixXi& mF,
                     Discretization D,
                     const vector<Tag>& vnTraits) {
  Mesh::Init(mV, mF, D, vnTraits);

  this->UpdateMetadata();
}

Mesh_Quad::~Mesh_Quad(void) {
  this->FreeMetadata();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Quad");
#endif
}

void Mesh_Quad::FreeMetadata() {
  // Not implemented
}

void Mesh_Quad::UpdateMetadata() {
  this->FreeMetadata();

  // Not implemented
}

Real Mesh_Quad::VolumeSpace(Tag s) const {
  if (this->DimSpace() == 2) {
    return this->VolumeBasis(s);
  }

  if (this->DimSpace() == 3) {
    // For each quad, compute the volume of the two tetrahedra
    // associated with the two triangles composing the face

    Real volume = 0;

    for (int i = 0; i < (int)this->m_vfaces.size(); ++i) {
      Node* pNode0 = this->m_vfaces[i]->Nodes()[0];
      Node* pNode1 = this->m_vfaces[i]->Nodes()[1];
      Node* pNode2 = this->m_vfaces[i]->Nodes()[2];
      Node* pNode3 = this->m_vfaces[i]->Nodes()[3];

      Matrix3d mV0;
      mV0.col(0) = pNode0->Traits().Vector3d(s);
      mV0.col(1) = pNode1->Traits().Vector3d(s);
      mV0.col(2) = pNode2->Traits().Vector3d(s);

      Matrix3d mV1;
      mV1.col(0) = pNode2->Traits().Vector3d(s);
      mV1.col(1) = pNode1->Traits().Vector3d(s);
      mV1.col(2) = pNode3->Traits().Vector3d(s);

      volume += (1.0 / 6.0) * (mV0.determinant() + mV1.determinant());
    }

    return volume;
  }

  throw PhySim::exception("Unreachable section");
}

}  // namespace PhySim