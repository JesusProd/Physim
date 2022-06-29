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
#include <PhySim/Geometry/Polytopes/Face_Tri.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Tri : public Mesh_Face {
 public:
  Mesh_Tri();
  Mesh_Tri(const Mesh_Tri& toCopy);
  Mesh_Tri(const MatrixXd& mV,
           const MatrixXi& mF,
           Discretization D = Discretization::Discretization_Tri3,
           const vector<Tag>& vnTraits = vector<Tag>());
  void Init(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Discretization_Tri3,
            const vector<Tag>& vnTraits = vector<Tag>());
  virtual ~Mesh_Tri(void);

  virtual Mesh* Clone() const override { return new Mesh_Tri(*this); }

  virtual Face_Tri* GetTriangle(int i) {
    return static_cast<Face_Tri*>(this->m_vfaces[i]);
  }

  virtual void GetBoundaryLoop(vector<Node*>& vbloop) override;
  virtual void GetBoundaryNodes(vector<Node*>& vbnode) override;

  virtual void GetBoundaryCurves(vector<vector<Node*>>& vC,
                                 Real angleTol,
                                 Tag s);

  virtual void SkinningBiharmonic(const VectorXi& vhandles,
                                  MatrixXd& mweights,
                                  Tag s);
  virtual void SkinningLaplacian(const VectorXi& vhandles,
                                 MatrixXd& mweights,
                                 Tag s);

  virtual Real VolumeSpace(Tag s) const override;

  virtual void MassProperties(Tag s,
                              Real rho,
                              Real& mass,
                              Vector3d& vcom,
                              Matrix3d& mI) const;

 private:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim
