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

#include <PhySim/Geometry/Meshes/Mesh.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Cell : public Mesh {
 public:
  Mesh_Cell();
  Mesh_Cell(const Mesh_Cell& toCopy);
  Mesh_Cell(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Tet4,
            const vector<Tag>& vnTraits = vector<Tag>());

  void Init(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Tet4,
            const vector<Tag>& vnTraits = vector<Tag>());

  virtual ~Mesh_Cell(void);

  virtual Real VolumeSpace(Tag s) const override {
    return this->VolumeBasis(s);
  }

 private:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim