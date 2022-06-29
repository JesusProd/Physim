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

#include <PhySim/Geometry/Meshes/Mesh_Grid.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_GridCoarsen : public Mesh_Grid {
 public:
  struct FineNodeMeta {
   public:
    map<int, int> mLocalActiveIdx;  // Active index in the submesh (local)
    map<int, int> mLocalFullIdx;    // Full index in the submesh (local)
  };

  struct FineElemMeta {
   public:
    int localActiveIdx;  // Active index in the submesh
    int localFullIdx;    // Full index in the submesh

    Poly* coarElem;
  };

  struct CoarNodeMeta {
    // Nothing
  };

  struct CoarElemMeta {
   public:
    vector<Node*> vfineNode;
    vector<Poly*> vfineElem;
    vector<PtrS<DEdge>> vboundaryXp;
    vector<PtrS<DEdge>> vboundaryXm;
    vector<PtrS<DEdge>> vboundaryYp;
    vector<PtrS<DEdge>> vboundaryYm;
    vector<PtrS<DEdge>> vboundaryZp;
    vector<PtrS<DEdge>> vboundaryZm;
  };

  struct SharedNodes {
   public:
    int coarse0;  // Coarse element 0 (the one referred in the call)
    int coarse1;  // Coarse element 1 (the neighbor coarse element)
    vector<Node*> vFineNodes;
    iVector vnodeIdx0;  // Active node indices in 0
    iVector vnodeIdx1;  // Active node indices in 1
  };

  struct TopologyChanges : public Mesh_Grid::TopologyChanges {
   public:
    Mesh_Grid::TopologyChanges fineChanges;
  };

 protected:
  Vector3i m_vcoarsening;
  PtrS<Mesh_Grid> m_pFineMesh;

 public:
  Mesh_GridCoarsen();
  Mesh_GridCoarsen(const Mesh_GridCoarsen& toCopy);
  Mesh_GridCoarsen(const Vector3d& vorigin,
                   const Vector3d& vsize,
                   const Vector3i& vfineDims,
                   const Vector3i& vcoarDims,
                   const iVector& vfineOccu,
                   Discretization D = Discretization::Discretization_Hex8,
                   const vector<Tag>& vs = vector<Tag>());

  void Init(const Vector3d& vorigin,
            const Vector3d& vsize,
            const Vector3i& vfineDims,
            const Vector3i& vcoarDims,
            const iVector& vfineOccu,
            Discretization D = Discretization::Discretization_Hex8,
            const vector<Tag>& vs = vector<Tag>());

  virtual ~Mesh_GridCoarsen(void);

  virtual Mesh* Clone() const override { return new Mesh_GridCoarsen(*this); }

  static Mesh_GridCoarsen* CreateSubCuboid(
      double sizeX,
      double sizeY,
      double sizeZ,
      Vector2d ranX,
      Vector2d ranY,
      Vector2d ranZ,
      int coarseElements,
      int coarseningLevel,
      vector<Tag> vnTraits = vector<Tag>());

  static Mesh_GridCoarsen* CreateFullCuboid(
      double sizeX,
      double sizeY,
      double sizeZ,
      int coarseElements,
      int coarseningLevel,
      vector<Tag> vnTraits = vector<Tag>());

  static Mesh_GridCoarsen* CreateSphere(const Vector3d& center,
                                        double radius,
                                        int coarseElements,
                                        int coarseningLevel,
                                        vector<Tag> vnTraits = vector<Tag>());

  TopologyChanges CutMesh_Disc(Real radius,
                               const Matrix3d& mR,
                               const Vector3d& vt,
                               Tag vs);
  TopologyChanges CutMesh_Rect(Real length,
                               Real height,
                               const Matrix3d& mR,
                               const Vector3d& vt,
                               Tag vs);
  TopologyChanges CutMesh(const list<PtrS<EdgeCut>>& lcutList);

  const Vector3i& Coar() const { return this->m_vcoarsening; }

  PtrS<Mesh_Grid> FineMesh() { return this->m_pFineMesh; }

  FineNodeMeta& GetFineNodeMeta(int idx) {
    return this->m_pFineMesh->Nodes()[idx]->Trait<FineNodeMeta>(
        Tag::Tag_FineMeta);
  }
  FineElemMeta& GetFineElemMeta(int idx) {
    return this->m_pFineMesh->Elems()[idx]->Trait<FineElemMeta>(
        Tag::Tag_FineMeta);
  }
  CoarNodeMeta& GetCoarNodeMeta(int idx) {
    return this->Nodes()[idx]->Trait<CoarNodeMeta>(Tag::Tag_CoarMeta);
  }
  CoarElemMeta& GetCoarElemMeta(int idx) {
    return this->Elems()[idx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);
  }

  virtual void MapGlobalIndexToSubmeshIndex_Elem(int globalIdx,
                                                 int& coarIdx,
                                                 int& submeshIdx) {
    Vector3i globalCoord =
        this->m_pFineMesh->TransformRegularIndexToCoord_Elem(globalIdx);
    Vector3i submeshCoord(globalCoord.x() % this->m_vcoarsening.x(),
                          globalCoord.y() % this->m_vcoarsening.y(),
                          globalCoord.z() % this->m_vcoarsening.z());
    Vector3i coarCoord(globalCoord.x() / this->m_vcoarsening.x(),
                       globalCoord.y() / this->m_vcoarsening.y(),
                       globalCoord.z() / this->m_vcoarsening.z());
    coarIdx = this->TransformRegularCoordToIndex_Elem(coarCoord);
    submeshIdx = this->TransformRegularCoordToIndex(
        this->m_vcoarsening.x(), this->m_vcoarsening.y(),
        this->m_vcoarsening.z(), submeshCoord);
  }

  virtual void MapSubmeshIndexToGlobalIndex_Elem(int coarIdx,
                                                 int submeshIdx,
                                                 int& globalIdx) {
    Vector3i submeshCoord = this->TransformRegularIndexToCoord(
        this->m_vcoarsening.x(), this->m_vcoarsening.y(),
        this->m_vcoarsening.z(), submeshIdx);
    Vector3i coarCoord = this->TransformRegularIndexToCoord_Elem(coarIdx);
    Vector3i globalCoord =
        Vector3i(coarCoord.x() * this->m_vcoarsening.x() + submeshCoord.x(),
                 coarCoord.y() * this->m_vcoarsening.y() + submeshCoord.y(),
                 coarCoord.z() * this->m_vcoarsening.z() + submeshCoord.z());
    globalIdx =
        this->m_pFineMesh->TransformRegularCoordToIndex_Elem(globalCoord);
  }

  virtual void InitializeCoarseElements_RegularGrid();
  virtual void RecomputeCoarseNeighbors(int coarElemIdx);
  virtual void RecomputeCoarseBoundaries(int coarElemIdx);
  virtual void RecomputeCoarseFineSubmesh(int coarElemIdx);

  virtual void RecomputeCoarse(list<Poly*>& laffected,
                               TopologyChanges& changes);

  virtual void ListConnectedOrphanElements(Poly* pRoot, list<Poly*>& lfine);
  virtual Cell_Hexa* CreateCoarseElement(const Cell_Hexa* pOld,
                                         const list<Poly*>& lchildren);

  virtual PtrS<Mesh_Hexa> GetFineSubmesh(int coarElemIdx);
  virtual void GetFineSubmesh(int coarElemIdx,
                              MatrixXd& mN,
                              MatrixXi& mE,
                              Tag nodeTrait);
  virtual void GetOversamplingSubmesh(vector<int> vcoarElemIdx,
                                      MatrixXd& mN,
                                      MatrixXi& mE,
                                      Tag nodeTrait);
  virtual void GetSharedNodes(int coarElemIdx,
                              int elemSide,
                              vector<SharedNodes>& vshared);

 protected:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim