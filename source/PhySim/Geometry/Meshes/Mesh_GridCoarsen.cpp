//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_GridCoarsen.h>

#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_GridCoarsen::Mesh_GridCoarsen() : Mesh_Grid() {
  this->UpdateMetadata();
}

Mesh_GridCoarsen::Mesh_GridCoarsen(const Mesh_GridCoarsen& toCopy)
    : Mesh_Grid(toCopy) {
  throw PhySim::exception("Not implemented");

  this->UpdateMetadata();
}

Mesh_GridCoarsen::Mesh_GridCoarsen(const Vector3d& vorigin,
                                   const Vector3d& vsize,
                                   const Vector3i& vfineDims,
                                   const Vector3i& vcoarDims,
                                   const iVector& vfineOccu,
                                   Discretization D,
                                   const vector<Tag>& vnTraits)
    : Mesh_Grid() {
  this->Init(vorigin, vsize, vfineDims, vcoarDims, vfineOccu, D, vnTraits);
}

void Mesh_GridCoarsen::Init(const Vector3d& vorigin,
                            const Vector3d& vsize,
                            const Vector3i& vfineDims,
                            const Vector3i& vcoarDims,
                            const iVector& vfineOccu,
                            Discretization D,
                            const vector<Tag>& vnTraits) {
  // Initialize fine mesh

  this->m_pFineMesh.reset(
      new Mesh_Grid(vorigin, vsize, vfineDims, vfineOccu, D, vnTraits));

  // Initialize coarse mesh: create occupancy

  iVector vcoarOccu;
  this->m_vcoarsening = vfineDims.cwiseQuotient(vcoarDims);
  vcoarOccu.reserve(vcoarDims.x() * vcoarDims.y() * vcoarDims.z());

  for (int i = 0; i < vcoarDims.x(); ++i)
    for (int j = 0; j < vcoarDims.y(); ++j)
      for (int k = 0; k < vcoarDims.z(); ++k) {
        int minI = m_vcoarsening.x() * i;
        int minJ = m_vcoarsening.y() * j;
        int minK = m_vcoarsening.z() * k;
        int maxI = minI + m_vcoarsening.x();
        int maxJ = minJ + m_vcoarsening.y();
        int maxK = minK + m_vcoarsening.z();

        bool found = false;
        for (int ii = minI; ii <= maxI && !found; ++ii)
          for (int jj = minJ; jj <= maxJ && !found; ++jj)
            for (int kk = minK; kk <= maxK && !found; ++kk) {
              if (this->m_pFineMesh->MapFull2Active_Elems()
                      [this->m_pFineMesh->TransformRegularCoordToIndex_Elem(
                          Vector3i(ii, jj, kk))] != -1)
                found = true;
            }

        if (found)
          vcoarOccu.push_back(TransformRegularCoordToIndex(
              vcoarDims.x(), vcoarDims.y(), vcoarDims.z(), Vector3i(i, j, k)));
      }

  // Initialize coarse mesh: create coarse mesh

  Mesh_Grid::Init(vorigin, vsize, vcoarDims, vcoarOccu, D, vnTraits);

  this->UpdateMetadata();
}

Mesh_GridCoarsen::~Mesh_GridCoarsen(void) {
  this->FreeMetadata();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[DEBUG] Deleting Mesh_GridCoarsen");
#endif
}

Mesh_GridCoarsen* Mesh_GridCoarsen::CreateSubCuboid(double sizeX,
                                                    double sizeY,
                                                    double sizeZ,
                                                    Vector2d ranX,
                                                    Vector2d ranY,
                                                    Vector2d ranZ,
                                                    int coarseElements,
                                                    int coarseningLevel,
                                                    vector<Tag> vnTraits) {
  int coarDimX = coarseElements;
  int coarDimY = coarseElements;
  int coarDimZ = coarseElements;
  int fineDimX = coarseElements * coarseningLevel;
  int fineDimY = coarseElements * coarseningLevel;
  int fineDimZ = coarseElements * coarseningLevel;

  double fineElemSizeX = sizeX / fineDimX;
  double fineElemSizeY = sizeY / fineDimY;
  double fineElemSizeZ = sizeZ / fineDimZ;

  // Set occupancy for a sphere of radius

  int countFine = 0;
  iVector voccupancy;
  for (int i = 0; i < fineDimX; ++i)
    for (int j = 0; j < fineDimY; ++j)
      for (int k = 0; k < fineDimZ; ++k) {
        double centerX = -sizeX / 2 + fineElemSizeX / 2 + i * fineElemSizeX;
        double centerY = -sizeY / 2 + fineElemSizeY / 2 + j * fineElemSizeY;
        double centerZ = -sizeZ / 2 + fineElemSizeZ / 2 + k * fineElemSizeZ;
        if (centerX >= ranX.x() && centerX <= ranX.y() && centerY >= ranY.x() &&
            centerY <= ranY.y() && centerZ >= ranZ.x() && centerZ <= ranZ.y()) {
          voccupancy.push_back(countFine);
        }
        countFine++;
      }

  // Create and initialize simulator

  Vector3d vsize = Vector3d(sizeX, sizeY, sizeZ);

  return new Mesh_GridCoarsen(
      -vsize * 0.5, vsize, Vector3i(fineDimX, fineDimY, fineDimZ),
      Vector3i(coarDimX, coarDimY, coarDimZ), voccupancy,
      Discretization::Discretization_Hex8, vnTraits);
}

Mesh_GridCoarsen* Mesh_GridCoarsen::CreateFullCuboid(double sizeX,
                                                     double sizeY,
                                                     double sizeZ,
                                                     int coarseElements,
                                                     int coarseningLevel,
                                                     vector<Tag> vnTraits) {
  int coarDimX = coarseElements;
  int coarDimY = coarseElements;
  int coarDimZ = coarseElements;
  int fineDimX = coarseElements * coarseningLevel;
  int fineDimY = coarseElements * coarseningLevel;
  int fineDimZ = coarseElements * coarseningLevel;

  double fineElemSizeX = sizeX / fineDimX;
  double fineElemSizeY = sizeY / fineDimY;
  double fineElemSizeZ = sizeZ / fineDimZ;

  int countFine = 0;
  iVector voccupancy;
  for (int i = 0; i < fineDimX; ++i)
    for (int j = 0; j < fineDimY; ++j)
      for (int k = 0; k < fineDimZ; ++k) {
        voccupancy.push_back(countFine++);
      }

  Vector3d vsize = Vector3d(sizeX, sizeY, sizeZ);

  return new Mesh_GridCoarsen(
      -vsize * 0.5, vsize, Vector3i(fineDimX, fineDimY, fineDimZ),
      Vector3i(coarDimX, coarDimY, coarDimZ), voccupancy,
      Discretization::Discretization_Hex8, vnTraits);
}

Mesh_GridCoarsen* Mesh_GridCoarsen::CreateSphere(const Vector3d& center,
                                                 double radius,
                                                 int coarseElements,
                                                 int coarseningLevel,
                                                 vector<Tag> vnTraits) {
  int coarDimX = coarseElements;
  int coarDimY = coarseElements;
  int coarDimZ = coarseElements;
  int fineDimX = coarseElements * coarseningLevel;
  int fineDimY = coarseElements * coarseningLevel;
  int fineDimZ = coarseElements * coarseningLevel;

  double sizeX = 2.001 * radius;
  double sizeY = 2.001 * radius;
  double sizeZ = 2.001 * radius;
  double fineElemSizeX = sizeX / fineDimX;
  double fineElemSizeY = sizeY / fineDimY;
  double fineElemSizeZ = sizeZ / fineDimZ;

  int countFine = 0;
  iVector voccupancy;
  for (int i = 0; i < fineDimX; ++i)
    for (int j = 0; j < fineDimY; ++j)
      for (int k = 0; k < fineDimZ; ++k) {
        double centerX = -sizeX / 2 + fineElemSizeX / 2 + i * fineElemSizeX;
        double centerY = -sizeY / 2 + fineElemSizeY / 2 + j * fineElemSizeY;
        double centerZ = -sizeZ / 2 + fineElemSizeZ / 2 + k * fineElemSizeZ;
        if (centerX * centerX + centerY * centerY + centerZ * centerZ <=
            radius) {
          voccupancy.push_back(countFine);
        }
        countFine++;
      }

  Vector3d vsize = Vector3d(sizeX, sizeY, sizeZ);

  return new Mesh_GridCoarsen(
      -vsize * 0.5, vsize, Vector3i(fineDimX, fineDimY, fineDimZ),
      Vector3i(coarDimX, coarDimY, coarDimZ), voccupancy,
      Discretization::Discretization_Hex8, vnTraits);
}

void Mesh_GridCoarsen::FreeMetadata() {
  // Not implemented
}

void Mesh_GridCoarsen::UpdateMetadata() {
  this->FreeMetadata();

  // Create metadata elements

  for (int i = 0; i < this->m_pFineMesh->NumNodes(); ++i)
    this->m_pFineMesh->Nodes()[i]->Traits().AddTrait<FineNodeMeta>(
        Tag::Tag_FineMeta, FineNodeMeta());
  for (int i = 0; i < this->m_pFineMesh->NumElems(); ++i)
    this->m_pFineMesh->Elems()[i]->Traits().AddTrait<FineElemMeta>(
        Tag::Tag_FineMeta, FineElemMeta());

  for (int i = 0; i < this->NumNodes(); ++i)
    this->Nodes()[i]->Traits().AddTrait<CoarNodeMeta>(Tag::Tag_CoarMeta,
                                                      CoarNodeMeta());
  for (int i = 0; i < this->NumElems(); ++i)
    this->Elems()[i]->Traits().AddTrait<CoarElemMeta>(Tag::Tag_CoarMeta,
                                                      CoarElemMeta());

  // Initialize regular grid

  this->InitializeCoarseElements_RegularGrid();

  // Update neighbors/indices

  for (int i = 0; i < this->NumElems(); ++i) {
    this->RecomputeCoarseFineSubmesh(i);
    this->RecomputeCoarseBoundaries(i);
    this->RecomputeCoarseNeighbors(i);
  }

  // Setup coarse-fine link

  this->Traits().AddTrait<PtrS<Mesh>>(Tag::Tag_FineMesh_0, this->m_pFineMesh);

  MatrixXd mVfine;
  this->m_pFineMesh->GetNodesTrait(mVfine, Tag_Position_0);
  this->EmbedMesh(*this->m_pFineMesh, Tag_Position_0);
}

void Mesh_GridCoarsen::InitializeCoarseElements_RegularGrid() {
  // Setup elements

  for (int i = 0; i < this->m_pFineMesh->NumElems(); ++i) {
    Vector3i fineElemFullCoord =
        this->m_pFineMesh->TransformRegularIndexToCoord_Elem(
            this->m_pFineMesh->MapActive2Full_Elems()[i]);
    Vector3i coarElemFullCoord(fineElemFullCoord.x() / this->m_vcoarsening.x(),
                               fineElemFullCoord.y() / this->m_vcoarsening.y(),
                               fineElemFullCoord.z() / this->m_vcoarsening.z());
    int coarElemFullIndex =
        this->TransformRegularCoordToIndex_Elem(coarElemFullCoord);
    int coarElemActiveIndex = this->MapFull2Active_Elems()[coarElemFullIndex];

    Poly* pFine = this->m_pFineMesh->Elems()[i];

    this->GetCoarElemMeta(coarElemActiveIndex).vfineElem.push_back(pFine);
    this->GetFineElemMeta(i).coarElem = this->Elems()[coarElemActiveIndex];
  }
}

void Mesh_GridCoarsen::RecomputeCoarseBoundaries(int coarElemIdx) {
  CoarElemMeta& coarElemMeta =
      this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);
  coarElemMeta.vboundaryXm.clear();
  coarElemMeta.vboundaryXp.clear();
  coarElemMeta.vboundaryYm.clear();
  coarElemMeta.vboundaryYp.clear();
  coarElemMeta.vboundaryZm.clear();
  coarElemMeta.vboundaryZp.clear();

  for (int j = 0; j < (int)coarElemMeta.vfineElem.size(); ++j) {
    Poly* pFineElem = coarElemMeta.vfineElem[j];
    GridElemMeta& gridElemMeta =
        pFineElem->Trait<GridElemMeta>(Tag::Tag_GridMeta);
    FineElemMeta& fineElemMeta =
        pFineElem->Trait<FineElemMeta>(Tag::Tag_FineMeta);

    if (!gridElemMeta.vDualEdgesXm.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesXm[0]->elem0;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        coarElemMeta.vboundaryXm.push_back(gridElemMeta.vDualEdgesXm[0]);
    }

    if (!gridElemMeta.vDualEdgesXp.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesXp[0]->elem1;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        coarElemMeta.vboundaryXp.push_back(gridElemMeta.vDualEdgesXp[0]);
    }

    if (!gridElemMeta.vDualEdgesYm.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesYm[0]->elem0;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        coarElemMeta.vboundaryYm.push_back(gridElemMeta.vDualEdgesYm[0]);
    }

    if (!gridElemMeta.vDualEdgesYp.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesYp[0]->elem1;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        coarElemMeta.vboundaryYp.push_back(gridElemMeta.vDualEdgesYp[0]);
    }

    if (!gridElemMeta.vDualEdgesZm.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesZm[0]->elem0;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        coarElemMeta.vboundaryZm.push_back(gridElemMeta.vDualEdgesZm[0]);
    }

    if (!gridElemMeta.vDualEdgesZp.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesZp[0]->elem1;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        coarElemMeta.vboundaryZp.push_back(gridElemMeta.vDualEdgesZp[0]);
    }
  }
}

void Mesh_GridCoarsen::RecomputeCoarseNeighbors(int coarElemIdx) {
  GridElemMeta& gridElemMeta =
      this->Elems()[coarElemIdx]->Trait<GridElemMeta>(Tag::Tag_GridMeta);
  CoarElemMeta& coarElemMeta =
      this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);
  gridElemMeta.vDualEdgesXm.clear();
  gridElemMeta.vDualEdgesXp.clear();
  gridElemMeta.vDualEdgesYm.clear();
  gridElemMeta.vDualEdgesYp.clear();
  gridElemMeta.vDualEdgesZm.clear();
  gridElemMeta.vDualEdgesZp.clear();

  {  // X-
    set<int> sneighbors;
    for (int i = 0; i < (int)coarElemMeta.vboundaryXm.size(); ++i) {
      int neigElemIdx = coarElemMeta.vboundaryXm[i]
                            ->elem0->Trait<FineElemMeta>(Tag::Tag_FineMeta)
                            .coarElem->ID();
      if (sneighbors.find(neigElemIdx) == sneighbors.end()) {
        pair<int, int> key(neigElemIdx, coarElemIdx);
        if (this->m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = this->Elems()[neigElemIdx];
          dualEdge->elem1 = this->Elems()[coarElemIdx];
          dualEdge->Dir = 0;
          m_mdualEdges[key] = dualEdge;
        }
        gridElemMeta.vDualEdgesXm.push_back(m_mdualEdges[key]);
        sneighbors.insert(neigElemIdx);  // Already connected
      }
    }
  }

  {  // X+
    set<int> sneighbors;
    for (int i = 0; i < (int)coarElemMeta.vboundaryXp.size(); ++i) {
      int neigElemIdx = coarElemMeta.vboundaryXp[i]
                            ->elem1->Trait<FineElemMeta>(Tag::Tag_FineMeta)
                            .coarElem->ID();
      if (sneighbors.find(neigElemIdx) == sneighbors.end()) {
        pair<int, int> key(coarElemIdx, neigElemIdx);
        if (this->m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = this->Elems()[coarElemIdx];
          dualEdge->elem1 = this->Elems()[neigElemIdx];
          dualEdge->Dir = 0;
          m_mdualEdges[key] = dualEdge;
        }
        gridElemMeta.vDualEdgesXp.push_back(m_mdualEdges[key]);
        sneighbors.insert(neigElemIdx);  // Already connected
      }
    }
  }

  {  // Y-
    set<int> sneighbors;
    for (int i = 0; i < (int)coarElemMeta.vboundaryYm.size(); ++i) {
      int neigElemIdx = coarElemMeta.vboundaryYm[i]
                            ->elem0->Trait<FineElemMeta>(Tag::Tag_FineMeta)
                            .coarElem->ID();
      if (sneighbors.find(neigElemIdx) == sneighbors.end()) {
        pair<int, int> key(neigElemIdx, coarElemIdx);
        if (this->m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = this->Elems()[neigElemIdx];
          dualEdge->elem1 = this->Elems()[coarElemIdx];
          dualEdge->Dir = 1;
          m_mdualEdges[key] = dualEdge;
        }
        gridElemMeta.vDualEdgesYm.push_back(m_mdualEdges[key]);
        sneighbors.insert(neigElemIdx);  // Already connected
      }
    }
  }

  {  // Y+
    set<int> sneighbors;
    for (int i = 0; i < (int)coarElemMeta.vboundaryYp.size(); ++i) {
      int neigElemIdx = coarElemMeta.vboundaryYp[i]
                            ->elem1->Trait<FineElemMeta>(Tag::Tag_FineMeta)
                            .coarElem->ID();
      if (sneighbors.find(neigElemIdx) == sneighbors.end()) {
        pair<int, int> key(coarElemIdx, neigElemIdx);
        if (this->m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = this->Elems()[coarElemIdx];
          dualEdge->elem1 = this->Elems()[neigElemIdx];
          dualEdge->Dir = 1;
          m_mdualEdges[key] = dualEdge;
        }
        gridElemMeta.vDualEdgesYp.push_back(m_mdualEdges[key]);
        sneighbors.insert(neigElemIdx);  // Already connected
      }
    }
  }

  {  // Z-
    set<int> sneighbors;
    for (int i = 0; i < (int)coarElemMeta.vboundaryZm.size(); ++i) {
      int neigElemIdx = coarElemMeta.vboundaryZm[i]
                            ->elem0->Trait<FineElemMeta>(Tag::Tag_FineMeta)
                            .coarElem->ID();
      if (sneighbors.find(neigElemIdx) == sneighbors.end()) {
        pair<int, int> key(neigElemIdx, coarElemIdx);
        if (this->m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = this->Elems()[neigElemIdx];
          dualEdge->elem1 = this->Elems()[coarElemIdx];
          dualEdge->Dir = 1;
          m_mdualEdges[key] = dualEdge;
        }
        gridElemMeta.vDualEdgesZm.push_back(m_mdualEdges[key]);
        sneighbors.insert(neigElemIdx);  // Already connected
      }
    }
  }

  {  // Z+
    set<int> sneighbors;
    for (int i = 0; i < (int)coarElemMeta.vboundaryZp.size(); ++i) {
      int neigElemIdx = coarElemMeta.vboundaryZp[i]
                            ->elem1->Trait<FineElemMeta>(Tag::Tag_FineMeta)
                            .coarElem->ID();
      if (sneighbors.find(neigElemIdx) == sneighbors.end()) {
        pair<int, int> key(coarElemIdx, neigElemIdx);
        if (this->m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = this->Elems()[coarElemIdx];
          dualEdge->elem1 = this->Elems()[neigElemIdx];
          dualEdge->Dir = 1;
          m_mdualEdges[key] = dualEdge;
        }
        gridElemMeta.vDualEdgesZp.push_back(m_mdualEdges[key]);
        sneighbors.insert(neigElemIdx);  // Already connected
      }
    }
  }
}

void Mesh_GridCoarsen::RecomputeCoarseFineSubmesh(int coarElemIdx) {
  // Assign local indices (full and active) to the fine
  // elements and nodes of the coarse element. Consider
  // fine elements can only belong to one coarse element
  // while fine nodes can belong to several of them. As
  // a consequence, local node indices are stored in a
  // map indexed by the coarse element index.

  int localElemCount = 0;
  int localNodeCount = 0;
  CoarElemMeta& coarElemMeta =
      this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);
  coarElemMeta.vfineNode.clear();

  // The bounding box of the coarse element is used to assign local
  // full indices to the fine nodes (depending on their parametric
  // position)

  MatrixXd mN;
  this->Elems()[coarElemIdx]->GetNodesTrait(mN, Tag_Position_0);
  Vector3d vminBox = mN.colwise().minCoeff();
  Vector3d vmaxBox = mN.colwise().maxCoeff();
  Vector3d vranBox = vmaxBox - vminBox;

  set<Node*> sfineNode;

  // Set fine elements local index

  for (int j = 0; j < (int)coarElemMeta.vfineElem.size(); ++j) {
    Poly* pFineElem = coarElemMeta.vfineElem[j];
    GridElemMeta& gridElemMeta =
        pFineElem->Trait<GridElemMeta>(Tag::Tag_GridMeta);
    FineElemMeta& fineElemMeta =
        pFineElem->Trait<FineElemMeta>(Tag::Tag_FineMeta);

    // Clear fine nodes local index

    for (int k = 0; k < pFineElem->NumNodes(); ++k) {
      Node* pFineNode = pFineElem->Nodes()[k];
      map<int, int>& mlocalFullIdx =
          pFineNode->Trait<FineNodeMeta>(Tag_FineMeta).mLocalFullIdx;
      map<int, int>& mlocalActiveIdx =
          pFineNode->Trait<FineNodeMeta>(Tag_FineMeta).mLocalActiveIdx;
      mlocalFullIdx.erase(coarElemIdx);
      mlocalActiveIdx.erase(coarElemIdx);
    }

    // Set fine element local index

    int globalFullElemIdx = gridElemMeta.fullIdx;
    Vector3i globalFullElemCoord =
        this->m_pFineMesh->TransformRegularIndexToCoord_Elem(globalFullElemIdx);
    Vector3i localFullElemCoord(
        globalFullElemCoord.x() % this->m_vcoarsening.x(),
        globalFullElemCoord.y() % this->m_vcoarsening.y(),
        globalFullElemCoord.z() % this->m_vcoarsening.z());
    int localFullElemIdx = this->TransformRegularCoordToIndex(
        this->m_vcoarsening.x(), this->m_vcoarsening.y(),
        this->m_vcoarsening.z(), localFullElemCoord);
    fineElemMeta.localActiveIdx = localElemCount++;
    fineElemMeta.localFullIdx = localFullElemIdx;
  }

  // Set fine nodes local index

  for (int j = 0; j < (int)coarElemMeta.vfineElem.size(); ++j) {
    Poly* pFineElem = coarElemMeta.vfineElem[j];
    GridElemMeta& gridElemMeta =
        pFineElem->Trait<GridElemMeta>(Tag::Tag_GridMeta);
    FineElemMeta& fineElemMeta =
        pFineElem->Trait<FineElemMeta>(Tag::Tag_FineMeta);

    for (int k = 0; k < pFineElem->NumNodes(); ++k) {
      Node* pFineNode = pFineElem->Nodes()[k];
      map<int, int>& mlocalFullIdx =
          pFineNode->Trait<FineNodeMeta>(Tag_FineMeta).mLocalFullIdx;
      map<int, int>& mlocalActiveIdx =
          pFineNode->Trait<FineNodeMeta>(Tag_FineMeta).mLocalActiveIdx;
      if (mlocalActiveIdx.find(coarElemIdx) == mlocalActiveIdx.end()) {
        Vector3d pos = pFineNode->Traits().Vector3d(Tag_Position_0);
        Real alphaX = 1e-6 + (pos.x() - vminBox.x()) / vranBox.x();
        Real alphaY = 1e-6 + (pos.y() - vminBox.y()) / vranBox.y();
        Real alphaZ = 1e-6 + (pos.z() - vminBox.z()) / vranBox.z();
        Vector3i localFullCoord((int)floor(alphaX * m_vcoarsening.x()),
                                (int)floor(alphaY * m_vcoarsening.y()),
                                (int)floor(alphaZ * m_vcoarsening.z()));
        int localFullIndex = this->TransformRegularCoordToIndex(
            m_vcoarsening.x() + 1, m_vcoarsening.y() + 1, m_vcoarsening.z() + 1,
            localFullCoord);
        mlocalActiveIdx[coarElemIdx] = localNodeCount++;
        mlocalFullIdx[coarElemIdx] = localFullIndex;
      }

      // Add the node to the list of nodes, the same  node might have
      // been added before but the set handles the repetition of nodes

      sfineNode.insert(pFineElem->Nodes()[k]);
    }
  }

  coarElemMeta.vfineNode.insert(coarElemMeta.vfineNode.end(), sfineNode.begin(),
                                sfineNode.end());

  // Sort nodes according to their index

  struct NodeLesser {
    int m_coarElemIdx;

    NodeLesser(int coarElemIdx) { this->m_coarElemIdx = coarElemIdx; }
    bool operator()(const Node* pA, const Node* pB) {
      auto& nodeMetaA = pA->Trait<FineNodeMeta>(Tag::Tag_FineMeta);
      auto& nodeMetaB = pB->Trait<FineNodeMeta>(Tag::Tag_FineMeta);
      return nodeMetaA.mLocalActiveIdx.at(m_coarElemIdx) <
             nodeMetaB.mLocalActiveIdx.at(m_coarElemIdx);
    }
  };

  std::sort(coarElemMeta.vfineNode.begin(), coarElemMeta.vfineNode.end(),
            NodeLesser(coarElemIdx));

  // Ensure the sorting was correct

  for (int i = 0; i < coarElemMeta.vfineNode.size(); ++i) {
    int found = coarElemMeta.vfineNode[i]
                    ->Trait<FineNodeMeta>(Tag::Tag_FineMeta)
                    .mLocalActiveIdx[coarElemIdx];
    if (found != i) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "[ERROR] Expected local index %d, found %d", i, found);
      assert(false);
    }
  }

  // Update the embedding of the new nodes

  for (auto it = coarElemMeta.vfineNode.begin();
       it != coarElemMeta.vfineNode.end(); ++it) {
    const Vector3d vpos = (*it)->Traits().Vector3d(Tag::Tag_Position_0);
    Embedding embed =
        *Elems()[coarElemIdx]->ComputeEmbedding(vpos, Tag::Tag_Position_0);
    if (embed.Valid())
      (*it)->Traits().AddTrait<Embedding>(Tag::Tag_Embedding_0, embed);
  }
}

void Mesh_GridCoarsen::GetSharedNodes(int coarElemIdx,
                                      int elemSide,
                                      vector<SharedNodes>& vshared) {
  CoarElemMeta& coarElemMeta =
      this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);

  // Choose side

  vector<PtrS<DEdge>>* vboundaryEdges = NULL;
  switch (elemSide) {
    case 0:
      vboundaryEdges = &coarElemMeta.vboundaryXm;
      break;
    case 1:
      vboundaryEdges = &coarElemMeta.vboundaryXp;
      break;
    case 2:
      vboundaryEdges = &coarElemMeta.vboundaryYm;
      break;
    case 3:
      vboundaryEdges = &coarElemMeta.vboundaryYp;
      break;
    case 4:
      vboundaryEdges = &coarElemMeta.vboundaryZm;
      break;
    case 5:
      vboundaryEdges = &coarElemMeta.vboundaryZp;
      break;
  }

  // Collect nodes

  map<int, set<pair<Node*, Node*>>> mcollect;

  for (int j = 0; j < (int)vboundaryEdges->size(); ++j) {
    PtrS<DEdge> pBoundaryEdge = vboundaryEdges->at(j);

    Poly* pThisElem;
    Poly* pOtherElem;
    switch (elemSide) {
      case 0:  // Negative X
      case 2:  // Negative Y
      case 4:  // Negative Z
        pThisElem = pBoundaryEdge->elem1;
        pOtherElem = pBoundaryEdge->elem0;
        break;
      case 1:  // Positive X
      case 3:  // Positive Y
      case 5:  // Positive Z
        pThisElem = pBoundaryEdge->elem0;
        pOtherElem = pBoundaryEdge->elem1;
        break;
    }

    int otherCoarIdx =
        pOtherElem->Trait<FineElemMeta>(Tag::Tag_FineMeta).coarElem->ID();

    if (mcollect.find(otherCoarIdx) == mcollect.end())
      mcollect[otherCoarIdx] = set<pair<Node*, Node*>>();
    set<pair<Node*, Node*>>& pairSet = mcollect[otherCoarIdx];

    switch (elemSide) {
      case 0:  // Negative X
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[0], pOtherElem->Nodes()[1]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[3], pOtherElem->Nodes()[2]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[7], pOtherElem->Nodes()[6]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[4], pOtherElem->Nodes()[5]));
        break;
      case 1:  // Positive X
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[1], pOtherElem->Nodes()[0]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[2], pOtherElem->Nodes()[3]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[6], pOtherElem->Nodes()[7]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[5], pOtherElem->Nodes()[4]));
        break;
      case 2:  // Negative Y
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[0], pOtherElem->Nodes()[3]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[4], pOtherElem->Nodes()[7]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[5], pOtherElem->Nodes()[6]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[1], pOtherElem->Nodes()[2]));
        break;
      case 3:  // Positive Y
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[3], pOtherElem->Nodes()[0]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[7], pOtherElem->Nodes()[4]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[6], pOtherElem->Nodes()[5]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[2], pOtherElem->Nodes()[1]));
        break;
      case 4:  // Negative Z
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[0], pOtherElem->Nodes()[4]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[1], pOtherElem->Nodes()[5]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[2], pOtherElem->Nodes()[6]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[3], pOtherElem->Nodes()[7]));
        break;
      case 5:  // Positive Z
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[4], pOtherElem->Nodes()[0]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[5], pOtherElem->Nodes()[1]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[6], pOtherElem->Nodes()[2]));
        pairSet.insert(
            pair<Node*, Node*>(pThisElem->Nodes()[7], pOtherElem->Nodes()[3]));
        break;
    }
  }

  // Extract list

  for (auto it = mcollect.begin(); it != mcollect.end(); ++it) {
    vshared.push_back(SharedNodes());
    vshared.back().coarse0 = coarElemIdx;
    vshared.back().coarse1 = it->first;

    set<pair<Node*, Node*>>::iterator itCurPairs = it->second.begin();
    set<pair<Node*, Node*>>::iterator itEndPairs = it->second.end();

    for (; itCurPairs != itEndPairs; ++itCurPairs) {
      assert(itCurPairs->first == itCurPairs->second);
      vshared.back().vnodeIdx0.push_back(
          itCurPairs->first->Trait<FineNodeMeta>(Tag::Tag_FineMeta)
              .mLocalActiveIdx[coarElemIdx]);
      vshared.back().vnodeIdx1.push_back(
          itCurPairs->second->Trait<FineNodeMeta>(Tag::Tag_FineMeta)
              .mLocalActiveIdx[it->first]);
      vshared.back().vFineNodes.push_back(itCurPairs->first);
    }
  }
}

Mesh_GridCoarsen::TopologyChanges Mesh_GridCoarsen::CutMesh_Rect(
    Real length,
    Real height,
    const Matrix3d& mR,
    const Vector3d& vt,
    Tag vs) {
  // Compute potential cuts

  list<PtrS<EdgeCut>> lcuts;
  this->m_pFineMesh->CollectCuts_Rect(length, height, mR, vt, vs, lcuts);

  return this->CutMesh(lcuts);
}

Mesh_GridCoarsen::TopologyChanges Mesh_GridCoarsen::CutMesh_Disc(
    Real radius,
    const Matrix3d& mR,
    const Vector3d& vt,
    Tag vs) {
  // Compute potential cuts

  list<PtrS<EdgeCut>> lcuts;
  this->m_pFineMesh->CollectCuts_Disc(radius, mR, vt, vs, lcuts);

  return this->CutMesh(lcuts);
}

Mesh_GridCoarsen::TopologyChanges Mesh_GridCoarsen::CutMesh(
    const list<PtrS<EdgeCut>>& lcutList) {
  Mesh_GridCoarsen::TopologyChanges changes;

  changes.fineChanges = this->m_pFineMesh->CutMesh(lcutList);

  // Collect all elements affected

  set<Poly*> sfineAffected;
  for (auto it = lcutList.begin(); it != lcutList.end(); ++it)
    sfineAffected.insert((*it)->vallElems.begin(), (*it)->vallElems.end());

  list<Poly*> lfineAffected;
  lfineAffected.insert(lfineAffected.begin(), sfineAffected.begin(),
                       sfineAffected.end());

  this->RecomputeCoarse(lfineAffected, changes);

  return changes;
}

void Mesh_GridCoarsen::RecomputeCoarse(
    list<Poly*>& lfineAffected,
    Mesh_GridCoarsen::TopologyChanges& changes) {
  set<Poly*> lcoarAffected;
  for (auto itCur = lfineAffected.begin(); itCur != lfineAffected.end();
       ++itCur)
    lcoarAffected.insert(
        (*itCur)->Trait<FineElemMeta>(Tag::Tag_FineMeta).coarElem);

  // Subdivide coarse elements

  list<Poly*> lremCoarElem;

  map<Poly*, vector<Node*>> mcandidates;

  for (auto itCur = lcoarAffected.begin(); itCur != lcoarAffected.end();
       ++itCur) {
    Cell_Hexa* pOldCoarElem = (Cell_Hexa*)(*itCur);
    GridElemMeta& oldGridElemMeta =
        pOldCoarElem->Trait<GridElemMeta>(Tag::Tag_GridMeta);
    CoarElemMeta& oldCoarElemMeta =
        pOldCoarElem->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);

    // Add the coarse element to revove

    lremCoarElem.push_back(pOldCoarElem);

    // Reset the coar link of all fine elements in the affected coarse element

    int numCoar = 0;
    for (int i = 0; i < (int)oldCoarElemMeta.vfineElem.size(); ++i)
      oldCoarElemMeta.vfineElem[i]
          ->Trait<FineElemMeta>(Tag::Tag_FineMeta)
          .coarElem = NULL;

    // Group fine elements in connected componets and create new coarse elements

    int numFine = (int)oldCoarElemMeta.vfineElem.size();

    int numTot = 0;

    for (int i = 0; i < numFine && numTot != numFine; ++i) {
      if (oldCoarElemMeta.vfineElem[i]
              ->Trait<FineElemMeta>(Tag::Tag_FineMeta)
              .coarElem == NULL) {
        // List all elements in the same connected component

        list<Poly*> lCC;
        this->ListConnectedOrphanElements(oldCoarElemMeta.vfineElem[i], lCC);

        numTot += (int)lCC.size();

        // Create coarse element copy of its parent. Create new
        // coarse nodes, initially, the new coarse nodes has no
        // identifiers. Some of this nodes will fuse

        Poly* pNewCoarElem = this->CreateCoarseElement(pOldCoarElem, lCC);
        mcandidates[pNewCoarElem] = pNewCoarElem->Nodes();
      }
    }
  }

  // Collect rem elements IDs

  for (auto it = lremCoarElem.begin(); it != lremCoarElem.end(); ++it)
    changes.vremElems.push_back((*it)->ID());

  this->RemoveElems(lremCoarElem);

  // Collect new elements IDs

  for (auto it = mcandidates.begin(); it != mcandidates.end(); ++it)
    changes.vnewElems.push_back(it->first->ID());

  // Rebuild coarse dual edges

  for (int i = 0; i < (int)this->m_velems.size(); ++i) {
    this->RecomputeCoarseFineSubmesh(i);
    this->RecomputeCoarseBoundaries(i);
    this->RecomputeCoarseNeighbors(i);
  }

  // Merge elements

  vector<Node*> vnewNodes = this->ClusterNodes(mcandidates);
  this->RemoveIsolatedNodes();

  // Collect new nodes IDs

  for (auto it = vnewNodes.begin(); it != vnewNodes.end(); ++it)
    changes.vnewNodes.push_back((*it)->ID());

  // Recompute all node neighbors

  set<Node*> sallNodes;
  sallNodes.insert(m_vnodes.begin(), m_vnodes.end());
  this->RecomputeNodeNeighborElements(sallNodes);
}

Cell_Hexa* Mesh_GridCoarsen::CreateCoarseElement(const Cell_Hexa* pOldCoarElem,
                                                 const list<Poly*>& lchildren) {
  Cell_Hexa* pNewCoarElem = new Cell_Hexa(*(Cell_Hexa*)pOldCoarElem);
  pNewCoarElem->ID() = (int)this->m_velems.size();
  this->m_velems.push_back(pNewCoarElem);
  this->m_vcells.push_back(pNewCoarElem);
  for (int j = 0; j < 8; ++j) {
    pNewCoarElem->Nodes()[j] = new Node(*pOldCoarElem->Nodes()[j]);
    pNewCoarElem->Nodes()[j]->ID() = -1;
  }

  pNewCoarElem->Traits().AddTrait<GridElemMeta>(Tag::Tag_GridMeta,
                                                GridElemMeta());
  pNewCoarElem->Traits().AddTrait<CoarElemMeta>(Tag::Tag_CoarMeta,
                                                CoarElemMeta());
  GridElemMeta& newGridElemMeta =
      pNewCoarElem->Trait<GridElemMeta>(Tag::Tag_GridMeta);
  CoarElemMeta& newCoarElemMeta =
      pNewCoarElem->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);

  set<Node*> sfineNodes;

  for (auto it = lchildren.begin(); it != lchildren.end(); ++it) {
    Poly* pFineElem = (*it);
    FineElemMeta& fineElemMeta =
        pFineElem->Trait<FineElemMeta>(Tag::Tag_FineMeta);
    GridElemMeta& gridElemMeta =
        pFineElem->Trait<GridElemMeta>(Tag::Tag_GridMeta);

    // Set basic fine-coar element link

    newCoarElemMeta.vfineElem.push_back(*it);
    fineElemMeta.coarElem = pNewCoarElem;

    // Collect coarse element fine nodes

    sfineNodes.insert(pFineElem->Nodes().begin(), pFineElem->Nodes().end());

    // Select boundary fine elements

    if (!gridElemMeta.vDualEdgesXm.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesXm[0]->elem0;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        newCoarElemMeta.vboundaryXm.push_back(gridElemMeta.vDualEdgesXm[0]);
    }

    if (!gridElemMeta.vDualEdgesXp.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesXp[0]->elem1;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        newCoarElemMeta.vboundaryXp.push_back(gridElemMeta.vDualEdgesXp[0]);
    }

    if (!gridElemMeta.vDualEdgesYm.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesYm[0]->elem0;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        newCoarElemMeta.vboundaryYm.push_back(gridElemMeta.vDualEdgesYm[0]);
    }

    if (!gridElemMeta.vDualEdgesYp.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesYp[0]->elem1;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        newCoarElemMeta.vboundaryYp.push_back(gridElemMeta.vDualEdgesYp[0]);
    }

    if (!gridElemMeta.vDualEdgesZm.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesZm[0]->elem0;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        newCoarElemMeta.vboundaryZm.push_back(gridElemMeta.vDualEdgesZm[0]);
    }

    if (!gridElemMeta.vDualEdgesZp.empty()) {
      Poly* pNeigElem = gridElemMeta.vDualEdgesZp[0]->elem1;
      if (GetFineElemMeta(pNeigElem->ID()).coarElem != fineElemMeta.coarElem)
        newCoarElemMeta.vboundaryZp.push_back(gridElemMeta.vDualEdgesZp[0]);
    }
  }

  // Collect all the fine nodes of this coarse node

  newCoarElemMeta.vfineNode.insert(newCoarElemMeta.vfineNode.end(),
                                   sfineNodes.begin(), sfineNodes.end());

  return pNewCoarElem;
}

void Mesh_GridCoarsen::ListConnectedOrphanElements(Poly* pRoot,
                                                   list<Poly*>& lfine) {
  // To store selected

  set<Poly*> sselected;

  // Create tree queue

  list<Poly*> lqueue;
  lqueue.push_back(pRoot);

  // While there are elems

  while (!lqueue.empty()) {
    // Breadth first exploration
    Poly* pNext = lqueue.front();
    lqueue.erase(lqueue.begin());

    if (pNext == NULL)
      continue;

    if (sselected.find(pNext) != sselected.end())
      continue;  // Already visited before!

    GridElemMeta& gridElemMeta = pNext->Trait<GridElemMeta>(Tag::Tag_GridMeta);
    FineElemMeta& fineElemMeta = pNext->Trait<FineElemMeta>(Tag::Tag_FineMeta);
    if (fineElemMeta.coarElem == NULL)  // Not asigned yet
    {
      if (!gridElemMeta.vDualEdgesXm.empty())
        lqueue.push_back(gridElemMeta.vDualEdgesXm[0]->elem0);
      if (!gridElemMeta.vDualEdgesXp.empty())
        lqueue.push_back(gridElemMeta.vDualEdgesXp[0]->elem1);
      if (!gridElemMeta.vDualEdgesYm.empty())
        lqueue.push_back(gridElemMeta.vDualEdgesYm[0]->elem0);
      if (!gridElemMeta.vDualEdgesYp.empty())
        lqueue.push_back(gridElemMeta.vDualEdgesYp[0]->elem1);
      if (!gridElemMeta.vDualEdgesZm.empty())
        lqueue.push_back(gridElemMeta.vDualEdgesZm[0]->elem0);
      if (!gridElemMeta.vDualEdgesZp.empty())
        lqueue.push_back(gridElemMeta.vDualEdgesZp[0]->elem1);

      sselected.insert(pNext);
    }
  }

  // Build selection list

  for (auto itCur = sselected.begin(); itCur != sselected.end(); ++itCur)
    lfine.push_back(*itCur);
}

PtrS<Mesh_Hexa> Mesh_GridCoarsen::GetFineSubmesh(int coarElemIdx) {
  CoarElemMeta& coarElemMeta =
      this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);

  MatrixXi mE(coarElemMeta.vfineElem.size(), 8);
  vector<MatrixXd> vmN(this->m_vnodeTraits.size());
  for (int i = 0; i < (int)vmN.size(); ++i)
    vmN[i].resize(coarElemMeta.vfineNode.size(), 3);

  // Get mesh matrices

  for (int i = 0; i < (int)mE.rows(); ++i)
    for (int j = 0; j < coarElemMeta.vfineElem[i]->NumNodes(); ++j) {
      Node* pFineNode =
          m_pFineMesh->Nodes()[coarElemMeta.vfineElem[i]->Nodes()[j]->ID()];
      FineNodeMeta& fineNodeMeta =
          pFineNode->Trait<FineNodeMeta>(Tag::Tag_FineMeta);
      mE(i, j) = fineNodeMeta.mLocalActiveIdx[coarElemIdx];

      for (int k = 0; k < (int)vmN.size(); ++k)
        vmN[k].row(fineNodeMeta.mLocalActiveIdx[coarElemIdx]) =
            pFineNode->Trait<Vector3d>(this->m_vnodeTraits[k]);
    }

  // Build fine submesh

  PtrS<Mesh_Hexa> pFineSubmesh(
      new Mesh_Hexa(vmN[0], mE, this->m_meshType, this->m_vnodeTraits));
  for (int i = 1; i < (int)vmN.size(); ++i)
    pFineSubmesh->SetNodesTrait(vmN[i], this->m_vnodeTraits[i]);

  return pFineSubmesh;
}

void Mesh_GridCoarsen::GetFineSubmesh(int coarElemIdx,
                                      MatrixXd& mN,
                                      MatrixXi& mE,
                                      Tag s) {
  CoarElemMeta& coarElemMeta =
      this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);

  mN.resize(coarElemMeta.vfineNode.size(), 3);
  mE.resize(coarElemMeta.vfineElem.size(), 8);

  for (int i = 0; i < (int)mE.rows(); ++i)
    for (int j = 0; j < coarElemMeta.vfineElem[i]->NumNodes(); ++j) {
      Node* pFineNode =
          m_pFineMesh->Nodes()[coarElemMeta.vfineElem[i]->Nodes()[j]->ID()];
      FineNodeMeta& fineNodeMeta =
          pFineNode->Trait<FineNodeMeta>(Tag::Tag_FineMeta);
      mN.row(fineNodeMeta.mLocalActiveIdx[coarElemIdx]) =
          pFineNode->Trait<Vector3d>(s);
      mE(i, j) = fineNodeMeta.mLocalActiveIdx[coarElemIdx];
    }
}
void Mesh_GridCoarsen::GetOversamplingSubmesh(vector<int> vcoarElemIdx,
                                              MatrixXd& mN,
                                              MatrixXi& mE,
                                              Tag s) {
  // CoarElemMeta& coarElemMeta =
  // this->Elems()[coarElemIdx]->Trait<CoarElemMeta>(Tag::Tag_CoarMeta);

  // mN.resize(coarElemMeta.vfineNode.size(), 3);
  // mE.resize(coarElemMeta.vfineElem.size(), 8);

  // for (int i = 0; i < (int)mE.rows(); ++i)
  // 	for (int j = 0; j < coarElemMeta.vfineElem[i]->NumNodes(); ++j)
  // 	{
  // 		Node* pFineNode =
  // m_pFineMesh->Nodes()[coarElemMeta.vfineElem[i]->Nodes()[j]->ID()];
  // 		FineNodeMeta& fineNodeMeta =
  // pFineNode->Trait<FineNodeMeta>(Tag::Tag_FineMeta);
  // 		mN.row(fineNodeMeta.mLocalActiveIdx[coarElemIdx]) =
  // pFineNode->Trait<Vector3d>(s); 		mE(i, j) =
  // fineNodeMeta.mLocalActiveIdx[coarElemIdx];
  // 	}
}
}  // namespace PhySim