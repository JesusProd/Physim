//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Tetra.h>

#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_Tetra::Mesh_Tetra() {
  this->UpdateMetadata();
}

Mesh_Tetra::Mesh_Tetra(const Mesh_Tetra& toCopy) : Mesh_Cell(toCopy) {
  this->UpdateMetadata();
}

Mesh_Tetra::Mesh_Tetra(const MatrixXd& mV,
                       const MatrixXi& mC,
                       Discretization D,
                       const vector<Tag>& vnTraits)
    : Mesh_Cell(mV, mC, D, vnTraits) {
  this->UpdateMetadata();
}

void Mesh_Tetra::Init(const MatrixXd& mV,
                      const MatrixXi& mE,
                      Discretization D,
                      const vector<Tag>& vnTraits) {
  Mesh_Cell::Init(mV, mE, D, vnTraits);

  this->UpdateMetadata();
}

Mesh_Tetra::~Mesh_Tetra(void) {
#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Tetra");
#endif
}

void Mesh_Tetra::FreeMetadata() {
  // Nothing to do here...
}

void Mesh_Tetra::UpdateMetadata() {
  this->FreeMetadata();
}

}  // namespace PhySim