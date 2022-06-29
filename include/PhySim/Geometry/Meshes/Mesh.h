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

#include <PhySim/Geometry/Geometry.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Poly.h>

#include <PhySim/Geometry/Polytopes/Embedding.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh : public Geometry {
 protected:
  int m_dimBasis;
  int m_dimSpace;

  PhySim::Discretization m_meshType;

  vector<Tag> m_vnodeTraits;
  vector<Tag> m_velemTraits;

 public:
  Mesh();
  Mesh(const Mesh& toCopy);
  Mesh(int numN,
       const MatrixXi& mE,
       PhySim::Discretization D = Discretization::Tri3,
       const vector<Tag>& vnTraits = vector<Tag>());
  Mesh(const MatrixXd& mN,
       const MatrixXi& mE,
       PhySim::Discretization D = Discretization::Tri3,
       const vector<Tag>& vnTraits = vector<Tag>());
  virtual ~Mesh(void);

  void Init(int numN,
            const MatrixXi& mE,
            PhySim::Discretization D = Discretization::Tri3,
            const vector<Tag>& vnTraits = vector<Tag>());
  void Init(const MatrixXd& mN,
            const MatrixXi& mE,
            PhySim::Discretization D = Discretization::Tri3,
            const vector<Tag>& vnTraits = vector<Tag>());

  virtual Mesh* Clone() const { return new Mesh(*this); }

  static void Merge(PtrS<Mesh>& pMesh,
                    const vector<PtrS<Mesh>>& vmeshes,
                    Tag trait,
                    Real tol);

  virtual void CloneTraits(const Mesh& toCopy);

  virtual Real VolumeSpace(Tag s) const override {
    return this->VolumeBasis(s);
  }

  virtual void MassProperties(Tag s,
                              Real rho,
                              Real& mass,
                              Vector3d& vcom,
                              Matrix3d& mI) const override;

  virtual int DimBasis() const override { return this->m_dimBasis; }
  virtual int DimSpace() const override { return this->m_dimSpace; }

  virtual int NumElemNodes() const {
    return (int)this->m_velems[0]->Nodes().size();
  }

  // Object

  virtual const vector<Tag>& NodeTraits() const { return this->m_vnodeTraits; }
  virtual const vector<Tag>& ElemTraits() const { return this->m_velemTraits; }
  virtual bool HasNodeTraits(Tag t) const {
    return find(this->m_vnodeTraits.begin(), this->m_vnodeTraits.end(), t) !=
           this->m_vnodeTraits.end();
  }
  virtual bool HasElemTraits(Tag t) const {
    return find(this->m_velemTraits.begin(), this->m_velemTraits.end(), t) !=
           this->m_velemTraits.end();
  }

  virtual PhySim::Discretization MeshType() const { return this->m_meshType; }

  virtual void GetElemMatrix(MatrixXi& mE) const;

  virtual Real VolumeBasis(Tag s = Tag::Position_X) const override;
  virtual Vector3d Centroid(Tag s = Tag::Position_X) const override;
  virtual Matrix3d Rotation(Tag f, Tag t) const override;

  // Topology

  virtual bool IsBoundary(const Node* pNode) {
    throw exception("Not implemented");
  }
  virtual bool IsBoundary(const Edge* pEdge) {
    throw exception("Not implemented");
  }
  virtual bool IsBoundary(const Face* pFace, bool node = true) {
    throw exception("Not implemented");
  }

  virtual void GetBoundaryNodes(vector<Node*>& vbnode) {
    throw exception("Not implemented");
  }
  virtual void GetInternalNodes(vector<Node*>& vbnode) {
    throw exception("Not implemented");
  }
  virtual void GetBoundaryEdges(vector<Edge*>& vbedge) {
    throw exception("Not implemented");
  }
  virtual void GetInternalEdges(vector<Edge*>& vbedge) {
    throw exception("Not implemented");
  }
  virtual void GetBoundaryFaces(vector<Face*>& vbface, bool node = true) {
    throw exception("Not implemented");
  }
  virtual void GetInternalFaces(vector<Face*>& vbface, bool node = true) {
    throw exception("Not implemented");
  }

 protected:
  virtual void FreeInternal();

 private:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim
