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

#include <PhySim/Physics/Simulables/Simulable_FEM.h>

#include <PhySim/Geometry/Meshes/Mesh_Face.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

/**
 * Simulable_FEM_Surface
 *
 * TODO.
 */
class Simulable_FEM_Surface : public Simulable_FEM {
 public:
  virtual string GetName() const override { return "[FEM Surface]"; }

  virtual Mesh_Face* MeshFace() {
    return static_cast<Mesh_Face*>(this->m_pMesh.get());
  }
};
}  // namespace PhySim
