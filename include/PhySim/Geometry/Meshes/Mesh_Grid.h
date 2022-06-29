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

#include <PhySim/Geometry/Meshes/Mesh_Hexa.h>

#include <set>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Grid : public Mesh_Hexa {
 public:
  struct DEdge {
    int ID;
    int Dir;
    Poly* elem0;
    Poly* elem1;
    DEdge() {
      ID = -1;
      Dir = -1;
      elem0 = NULL;
      elem1 = NULL;
    }
  };

  struct EdgeCut {
    DEdge* pEdge;
    vector<Node*> vcutNodes;
    vector<Poly*> vcutElems;
    vector<Poly*> vallElems;
    int side0;
    int side1;
    EdgeCut() {
      pEdge = NULL;
      side0 = -1;
      side1 = -1;
    }
  };

  struct GridNodeMeta {
    int fullIdx;  // Index assuming a fully occupied regular grid
    int surfaces;
    vector<Poly*> vNeiElems;
    GridNodeMeta() {
      surfaces = 0;
      vNeiElems.resize(8, NULL);
    }
    bool IsSurface() { return surfaces > 0; }
  };

  struct GridElemMeta {
    int fullIdx;   // Index assuming a fully occupied regular grid
    bool surface;  // Is this element a surface element? (has at least one
                   // surface node)
    vector<PtrS<DEdge>> vDualEdgesXm;  // Dual edges at the X negative side
    vector<PtrS<DEdge>> vDualEdgesXp;  // Dual edges at the X positive side
    vector<PtrS<DEdge>> vDualEdgesYm;  // Dual edges at the Y negative side
    vector<PtrS<DEdge>> vDualEdgesYp;  // Dual edges at the Y positive side
    vector<PtrS<DEdge>> vDualEdgesZm;  // Dual edges at the Z negative side
    vector<PtrS<DEdge>> vDualEdgesZp;  // Dual edges at the z positive side
    GridElemMeta() {
      surface = false;
      fullIdx = -1;
    }
  };

  struct TopologyChanges {
    vector<int> vremElems;
    vector<int> vremNodes;
    vector<int> vnewElems;
    vector<int> vnewNodes;
  };

 protected:
  // Initial grid
  Vector3d m_vorigin;
  Vector3d m_vsize;
  Vector3i m_vdims;
  iVector m_voccu;
  vector<int> m_vmapFull2Active_Nodes;
  vector<int> m_vmapFull2Active_Elems;
  vector<int> m_vmapActive2Full_Nodes;
  vector<int> m_vmapActive2Full_Elems;
  map<pair<int, int>, PtrS<DEdge>> m_mdualEdges;

 public:
  Mesh_Grid();
  Mesh_Grid(const Mesh_Grid& toCopy);
  Mesh_Grid(const Vector3d& vorigin,
            const Vector3d& vsize,
            const Vector3i& vdims,
            const iVector& voccup,
            Discretization D = Discretization::Discretization_Hex8,
            const vector<Tag>& vs = vector<Tag>());

  void Init(const Vector3d& vorigin,
            const Vector3d& vsize,
            const Vector3i& vdims,
            const iVector& voccup,
            Discretization D = Discretization::Discretization_Hex8,
            const vector<Tag>& vs = vector<Tag>());

  virtual ~Mesh_Grid(void);

  virtual Mesh* Clone() const override { return new Mesh_Grid(*this); }

  void InitializeActiveMesh(const Vector3d& vorigin,
                            const Vector3d& vsize,
                            const Vector3i& vdims,
                            const iVector& voccup,
                            MatrixXd& mNode,
                            MatrixXi& mElem,
                            iVector& vfull2Active_Node,
                            iVector& vfull2Active_Elem,
                            iVector& vactive2Full_Node,
                            iVector& vactive2Full_Elem);

  void InitializeMetaData(const Vector3i& vdims,
                          const vector<Node*> vnodes,
                          const vector<Poly*> velems,
                          const iVector& vfull2active_elems,
                          const iVector& vactive2full_elems,
                          const iVector& vfull2active_nodes,
                          const iVector& vactive2full_nodes);

  const Vector3d& Size() const { return this->m_vsize; }
  const Vector3i& Dims() const { return this->m_vdims; }
  const iVector& Occupancy() const { return this->m_voccu; }
  const iVector& MapFull2Active_Nodes() const {
    return this->m_vmapFull2Active_Nodes;
  }
  const iVector& MapFull2Active_Elems() const {
    return this->m_vmapFull2Active_Elems;
  }
  const iVector& MapActive2Full_Nodes() const {
    return this->m_vmapActive2Full_Nodes;
  }
  const iVector& MapActive2Full_Elems() const {
    return this->m_vmapActive2Full_Elems;
  }

  const map<pair<int, int>, PtrS<DEdge>>& DualEdges() {
    return this->m_mdualEdges;
  }

  GridNodeMeta& GetNodeGridMeta(int idx) {
    return this->m_vnodes[idx]->Trait<GridNodeMeta>(Tag::Tag_GridMeta);
  }
  GridElemMeta& GetElemGridMeta(int idx) {
    return this->m_velems[idx]->Trait<GridElemMeta>(Tag::Tag_GridMeta);
  }

  TopologyChanges CutMesh_Disc(Real radius,
                               const Matrix3d& mR,
                               const Vector3d& vt,
                               Tag vs);
  void CollectCuts_Disc(Real radius,
                        const Matrix3d& mR,
                        const Vector3d& vt,
                        Tag vs,
                        list<PtrS<EdgeCut>>& cutList);
  PtrS<EdgeCut> CheckDualEdge_Disc(DEdge* pDualEdge,
                                   Real radius,
                                   const Matrix3d& mR,
                                   const Vector3d& vt,
                                   Tag vs);

  TopologyChanges CutMesh_Rect(Real length,
                               Real height,
                               const Matrix3d& mR,
                               const Vector3d& vt,
                               Tag vs);
  void CollectCuts_Rect(Real length,
                        Real height,
                        const Matrix3d& mR,
                        const Vector3d& vt,
                        Tag vs,
                        list<PtrS<EdgeCut>>& cutList);
  PtrS<EdgeCut> CheckDualEdge_Rect(DEdge* pDualEdge,
                                   Real length,
                                   Real height,
                                   const Matrix3d& mR,
                                   const Vector3d& vt,
                                   Tag vs);

  TopologyChanges CutMesh(const list<PtrS<EdgeCut>>& lcutList);

  void CollectCutAffectedNodesAndElems(const Matrix3d& mR,
                                       const Vector3d& vt,
                                       Tag s,
                                       PtrS<EdgeCut> pCut);

  void RemoveIsolatedNodes();
  void RemoveNodes(const list<Node*>& lnodes);
  void RemoveElems(const list<Poly*>& lelems);
  void RemoveEdges(const list<DEdge*>& ledges);
  vector<Node*> ClusterNodes(map<Poly*, vector<Node*>>& mcandidates);

  void RecomputeNodeNeighborElements(const set<Node*>& snodes);

  int TransformRegularCoordToIndex_Node(const Vector3i& coord) {
    return TransformRegularCoordToIndex(this->m_vdims.x() + 1,
                                        this->m_vdims.y() + 1,
                                        this->m_vdims.z() + 1, coord);
  }

  Vector3i TransformRegularIndexToCoord_Node(const int& idx) {
    return TransformRegularIndexToCoord(this->m_vdims.x() + 1,
                                        this->m_vdims.y() + 1,
                                        this->m_vdims.z() + 1, idx);
  }

  int TransformRegularCoordToIndex_Elem(const Vector3i& coord) {
    return TransformRegularCoordToIndex(this->m_vdims.x(), this->m_vdims.y(),
                                        this->m_vdims.z(), coord);
  }

  Vector3i TransformRegularIndexToCoord_Elem(const int& idx) {
    return TransformRegularIndexToCoord(this->m_vdims.x(), this->m_vdims.y(),
                                        this->m_vdims.z(), idx);
  }

  int TransformRegularCoordToIndex(int dimX,
                                   int dimY,
                                   int dimZ,
                                   const Vector3i& cor) {
    return (dimY * dimZ) * cor.x() + (dimZ)*cor.y() + cor.z();
  }

  Vector3i TransformRegularIndexToCoord(int dimX,
                                        int dimY,
                                        int dimZ,
                                        const int& idx) {
    int i = idx / (dimY * dimZ);
    int ri = idx % (dimY * dimZ);
    int j = ri / dimZ;
    int k = ri % dimZ;
    return Vector3i(i, j, k);
  }

 protected:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim