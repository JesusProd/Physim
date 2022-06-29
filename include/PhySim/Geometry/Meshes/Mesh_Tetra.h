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

#include <PhySim/Geometry/Meshes/Mesh_Cell.h>
#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Tetra : public Mesh_Cell {
 protected:
  MatrixXi m_mSurface;

 public:
  Mesh_Tetra();
  Mesh_Tetra(const Mesh_Tetra& toCopy);
  Mesh_Tetra(const MatrixXd& mV,
             const MatrixXi& mT,
             Discretization D = Discretization::Discretization_Tet4,
             const vector<Tag>& vnTraits = vector<Tag>());

  void Init(const MatrixXd& mV,
            const MatrixXi& mT,
            Discretization D = Discretization::Discretization_Tet4,
            const vector<Tag>& vnTraits = vector<Tag>());

  virtual ~Mesh_Tetra(void);

  virtual Mesh* Clone() const override { return new Mesh_Tetra(*this); }

  virtual Cell_Tetra* GetTetra(int i) {
    return static_cast<Cell_Tetra*>(this->m_vcells[i]);
  }

  virtual MatrixXi& Surface() { return this->m_mSurface; }

 protected:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim
