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

#include <PhySim/Geometry/Meshes/Mesh_Face.h>
#include <PhySim/Geometry/Polytopes/Face_Quad.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Quad : public Mesh_Face {
 protected:
 public:
  Mesh_Quad();
  Mesh_Quad(const Mesh_Quad& toCopy);
  Mesh_Quad(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Quad4,
            const vector<Tag>& vnTraits = vector<Tag>());
  void Init(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Quad4,
            const vector<Tag>& vnTraits = vector<Tag>());
  virtual ~Mesh_Quad(void);

  virtual Mesh* Clone() const override { return new Mesh_Quad(*this); }

  virtual Face_Quad* GetFace(int i) {
    return static_cast<Face_Quad*>(this->m_velems[i]);
  }

  virtual Real VolumeSpace(Tag s) const override;

 private:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim
