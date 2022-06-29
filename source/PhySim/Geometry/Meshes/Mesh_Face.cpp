//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Face.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face_Tri.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_Face::Mesh_Face() : Mesh() {
  this->UpdateMetadata();
}

Mesh_Face::Mesh_Face(const Mesh_Face& toCopy) : Mesh(toCopy) {
  this->UpdateMetadata();
}

Mesh_Face::Mesh_Face(const MatrixXd& mV,
                     const MatrixXi& mF,
                     Discretization D,
                     const vector<Tag>& vnTraits)
    : Mesh(mV, mF, D, vnTraits) {
  this->UpdateMetadata();
}

void Mesh_Face::Init(const MatrixXd& mV,
                     const MatrixXi& mF,
                     Discretization D,
                     const vector<Tag>& vnTraits) {
  Mesh::Init(mV, mF, D, vnTraits);

  this->UpdateMetadata();
}

Mesh_Face::~Mesh_Face(void) {
  this->FreeInternal();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Face");
#endif
}

void Mesh_Face::FreeMetadata() {
  // Delete edges and faces

  for (int i = 0; i < this->m_vedges.size(); ++i)
    delete m_vedges[i];

  this->m_vedges.clear();
  this->m_vfaces.clear();

  // Delete HE structure

  for (int i = 0; i < this->m_vheNodes.size(); ++i)
    delete m_vheNodes[i];

  for (int i = 0; i < this->m_vheEdges.size(); ++i)
    delete m_vheEdges[i];

  for (int i = 0; i < this->m_vheFaces.size(); ++i)
    delete m_vheFaces[i];

  for (int i = 0; i < this->m_vheHalfs.size(); ++i)
    delete m_vheHalfs[i];

  this->m_vheNodes.clear();
  this->m_vheEdges.clear();
  this->m_vheFaces.clear();
  this->m_vheHalfs.clear();
}

Real Mesh_Face::ComputeArea(Tag s) {
  Real area = 0;
  for (int i = 0; i < this->m_vfaces.size(); ++i)
    area += m_vfaces[i]->Area(s);  // Add area
  return area;
}

Real Mesh_Face::Perimeter(Tag s) {
  Real perimeter = 0;

  vector<Edge*> vbedges;
  this->GetBoundaryEdges(vbedges);
  for (int i = 0; i < this->m_vedges.size(); ++i)
    perimeter += m_vedges[i]->Length(s);
  return perimeter;
}

void Mesh_Face::GetEdgeHinge_Nodes(const Edge* pEdge, vector<Node*>& vhinge) {
  HE_Edge* pheEdge = GetHEEdge(pEdge->ID());

  if (this->IsBoundary(pEdge)) {
    HE_Half* pheHalf = pheEdge->m_pheHalf0;
    vhinge.push_back(pheHalf->m_pheNode->m_pHandle);
    pheHalf = pheHalf->m_pheNext;
    vhinge.push_back(pheHalf->m_pheNode->m_pHandle);
    pheHalf = pheHalf->m_pheNext;
    vhinge.push_back(pheHalf->m_pheNode->m_pHandle);
  } else {
    HE_Half* pheHalf = pheEdge->m_pheHalf0;
    vhinge.push_back(pheHalf->m_pheNode->m_pHandle);
    vhinge.push_back(pheHalf->m_pheNext->m_pheNode->m_pHandle);
    vhinge.push_back(pheHalf->m_phePair->m_pheNode->m_pHandle);
    vhinge.push_back(pheHalf->m_phePair->m_pheNext->m_pheNode->m_pHandle);
  }
}

void Mesh_Face::GetNodeRing_InputHE(const Node* pNode,
                                    vector<HE_Half*>& vring) {
  HE_Node* pheNode0 = GetHENode(pNode->ID());

  // Go through input half-edges CW

  HE_Half* pheHalfCW = pheNode0->m_pheHalf->m_phePair;

  while (vring.empty() || vring.front() != pheHalfCW) {
    // Add input half-edge to the ring

    vring.insert(vring.end(), pheHalfCW);

    // If boundary: go out

    if (pheHalfCW == NULL)
      break;  // Finished

    // Iterate to next input half-edge CW

    pheHalfCW = pheHalfCW->m_pheNext->m_phePair;
  }

  // If boundary found

  if (pheHalfCW == NULL) {
    // Go through input half-edges CCW

    HE_Half* pheHalfCCW = pheNode0->m_pheHalf;

    while (pheHalfCCW != NULL) {
      // Next input half-edge CCW

      HE_Half* pheHalf0 = pheHalfCCW;
      do
        pheHalfCCW = pheHalfCCW->m_pheNext;
      while (pheHalf0 != pheHalfCCW->m_pheNext);

      // Add ring input half-edge

      vring.insert(vring.begin(), pheHalfCCW);

      // Next input half-edge clockwise

      pheHalfCCW = pheHalfCCW->m_phePair;

      // It's a boundary half-edge

      if (pheHalfCCW == NULL)
        break;
    }
  }
}

void Mesh_Face::GetNodeRing_OutputHE(const Node* pNode,
                                     vector<HE_Half*>& vring) {
  vector<HE_Half*> vheInput;
  this->GetNodeRing_InputHE(pNode, vheInput);
  for (int i = 0; i < (int)vheInput.size(); ++i)
    if (vheInput[i] != NULL)  // It is not boundary
      vring.push_back(vheInput[i]->m_phePair);
}

void Mesh_Face::GetNodeRing_Faces(const Node* pNode,
                                  vector<Face*>& vring,
                                  int D) {
  if (D != 1) {
    throw exception("Not implemented");
  }

  vector<HE_Half*> vheInput;
  this->GetNodeRing_InputHE(pNode, vheInput);
  for (int i = 0; i < (int)vheInput.size(); ++i)
    if (vheInput[i] != NULL)  // It is not boundary
      vring.push_back(vheInput[i]->m_pheFace->m_pHandle);
}

void Mesh_Face::GetNodeRing_Edges(const Node* pNode,
                                  vector<Edge*>& vring,
                                  int D) {
  throw exception("Not implemented");
}

void Mesh_Face::GetNodeRing_Nodes(const Node* pNode,
                                  vector<Node*>& vring,
                                  int D) {
  throw exception("Not implemented");
}

void Mesh_Face::ComputeFaceToNodeMap_Angle(MatrixSd& mM, Tag s) {
  throw exception("Not implemented");
}

void Mesh_Face::ComputeFaceToNodeMap_Area(MatrixSd& mM, Tag s) {
  VectorTd vweiT;

  for (int i = 0; i < (int)m_vnodes.size(); ++i) {
    vector<Face*> vfaces;

    this->GetNodeRing_Faces(m_vnodes[i], vfaces, 1);

    Real weiSum = 0;
    VectorXd vwei(vfaces.size());
    for (int j = 0; j < (int)vfaces.size(); ++j)
      weiSum += vwei[j] = vfaces[j]->Area(s);

    vwei /= weiSum;

    for (int j = 0; j < (int)vfaces.size(); ++j)
      vweiT.push_back(
          Triplet<Real>(m_vnodes[i]->ID(), vfaces[j]->ID(), vwei[j]));
  }

  mM = MatrixSd(m_vnodes.size(), m_vfaces.size());
  mM.setFromTriplets(vweiT.begin(), vweiT.end());
  mM.makeCompressed();
}

Real Mesh_Face::ComputeDihedralAngle(const Edge* pEdge, Tag s) {
  if (this->IsBoundary(pEdge))
    return 0;  // No angle

  HE_Edge* pheEdge = this->GetHEEdge(pEdge->ID());

  Vector3d vt = pEdge->Tangent(s);

  if (vt.squaredNorm() == 0)
    return 0;  // Sanity check

  Vector3d vn0 = pheEdge->m_pheHalf0->m_pheFace->m_pHandle->Normal(s);
  Vector3d vn1 = pheEdge->m_pheHalf1->m_pheFace->m_pHandle->Normal(s);

  if (vn0.squaredNorm() == 0)
    return 0;  // Sanity check

  if (vn1.squaredNorm() == 0)
    return 0;  // Sanity check

  Real cosine = vn0.dot(vn1);
  Real sine = vt.dot(vn0.cross(vn1));
  Real angle = atan2(sine, cosine);

  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;

  return angle;
}

Matrix2d Mesh_Face::ComputeShapeOperator(const Node* pNode, Tag s) {
  // TODO: Average curvature at incident edges?

  return Matrix2d();
}

Matrix2d Mesh_Face::ComputeShapeOperator(const Edge* pEdge, Tag s) {
  HE_Edge* pheEdge = this->GetHEEdge(pEdge->ID());

  Face* pFace0 = pheEdge->m_pheHalf0->m_pheFace->m_pHandle;
  Face* pFace1 = pheEdge->m_pheHalf1->m_pheFace->m_pHandle;
  Vector3d n0 = pFace0->Normal(s);
  Vector3d n1 = pFace1->Normal(s);
  Real angle = acos(n0.dot(n1));

  // Compute transform to tangential space

  VectorXd vb;
  MatrixXd mA;
  pEdge->TransformNat2Bas(vb, mA, s);

  // Compute projection vector

  Vector2d t = mA * ((n0 + n1).cross(pEdge->Vector(s)).normalized());
  Real F = (6 * pEdge->Length(s)) / (pFace0->Area(s) + pFace1->Area(s));

  return F * angle * (t * t.transpose());
}

Matrix2d Mesh_Face::ComputeShapeOperator(const Face* pFace, Tag s) {
  Matrix2d S = Matrix2d::Zero();

  HE_Face* phFace = this->GetHEFace(pFace->ID());

  HE_Half* phHalfIni = phFace->m_pheHalf;
  HE_Half* phHalfCur0 = phFace->m_pheHalf;
  HE_Half* phHalfCur1 = phHalfCur0->m_phePair;

  Vector3d n = phFace->m_pHandle->Normal(s);

  // Compute transform to tangential space

  VectorXd vb;
  MatrixXd mA;
  pFace->TransformNat2Bas(vb, mA, s);

  int N = 0;

  do {
    N++;

    if (phHalfCur1 == NULL) {
      phHalfCur0 = phHalfCur0->m_pheNext;
      phHalfCur1 = phHalfCur0->m_phePair;
      continue;  // This is boundary edge
    }

    Edge* pEdge_i = phHalfCur0->m_pheEdge->m_pHandle;
    Face* pFace_i = phHalfCur1->m_pheFace->m_pHandle;

    Vector3d n_i = pFace_i->Normal(s);
    Real angle_i = acos(n.dot(n_i));

    // Compute projection vector

    Vector3d v_i = phHalfCur0->m_pheNode->m_pHandle->Traits().Vector3d(s) -
                   phHalfCur1->m_pheNode->m_pHandle->Traits().Vector3d(s);
    v_i.normalize();
    Vector2d t_i = mA * (v_i.cross(n_i).normalized());

    S += angle_i * pEdge_i->Length(s) * (t_i * t_i.transpose());

    phHalfCur0 = phHalfCur0->m_pheNext;
    phHalfCur1 = phHalfCur0->m_phePair;
  } while (phHalfCur0 != phHalfIni);

  return S / (2 * pFace->Area(s));
}

Matrix2d Mesh_Face::ComputePlanarStrain(const Face* pFace,
                                        Tag s0,
                                        Tag sx,
                                        const VectorXd& vp) {
  MatrixXd mF = pFace->DeformationGradient(vp, s0, sx);
  return 0.5 * (mF.transpose() * mF - Matrix2d::Identity());
}

Matrix2d Mesh_Face::ComputeBendingStrain(const Node* pNode, Tag s0, Tag sx) {
  // TODO: Average curvature at incident edges?

  return Matrix2d();
}

Matrix2d Mesh_Face::ComputeBendingStrain(const Edge* pEdge, Tag s0, Tag sx) {
  HE_Edge* pheEdge = this->GetHEEdge(pEdge->ID());

  Face* pFace0 = pheEdge->m_pheHalf0->m_pheFace->m_pHandle;
  Face* pFace1 = pheEdge->m_pheHalf1->m_pheFace->m_pHandle;
  Vector3d n0_x = pFace0->Normal(sx);
  Vector3d n1_x = pFace1->Normal(sx);
  Vector3d n0_0 = pFace0->Normal(s0);
  Vector3d n1_0 = pFace1->Normal(s0);
  Real angle_x = acos(n0_x.dot(n1_x));
  Real angle_0 = acos(n0_0.dot(n1_0));

  // Compute transform to tangential space

  VectorXd vb;
  MatrixXd mA;
  pEdge->TransformNat2Bas(vb, mA, s0);

  // Compute projection vector

  Vector2d t = mA * ((n0_0 + n1_0).cross(pEdge->Vector(s0)).normalized());
  Real F = (6 * pEdge->Length(s0)) / (pFace0->Area(s0) + pFace1->Area(s0));

  return F * (angle_x - angle_0) * (t * t.transpose());
}

Matrix2d Mesh_Face::ComputeBendingStrain(const Face* pFace, Tag s0, Tag sx) {
  Matrix2d S = Matrix2d::Zero();

  HE_Face* phFace = this->GetHEFace(pFace->ID());

  HE_Half* phHalfIni = phFace->m_pheHalf;
  HE_Half* phHalfCur0 = phFace->m_pheHalf;
  HE_Half* phHalfCur1 = phHalfCur0->m_phePair;

  Vector3d n_x = phFace->m_pHandle->Normal(sx);
  Vector3d n_0 = phFace->m_pHandle->Normal(s0);

  // Compute transform to tangential space

  VectorXd vb;
  MatrixXd mA;
  pFace->TransformNat2Bas(vb, mA, s0);

  int N = 0;

  do {
    N++;

    if (phHalfCur1 == NULL) {
      phHalfCur0 = phHalfCur0->m_pheNext;
      phHalfCur1 = phHalfCur0->m_phePair;
      continue;  // This is boundary edge
    }

    Edge* pEdge_i = phHalfCur0->m_pheEdge->m_pHandle;
    Face* pFace_i = phHalfCur1->m_pheFace->m_pHandle;

    Vector3d n_i_x = pFace_i->Normal(sx);
    Vector3d n_i_0 = pFace_i->Normal(s0);
    Real angle_i_x = acos(n_x.dot(n_i_x));
    Real angle_i_0 = acos(n_0.dot(n_i_0));

    // Compute projection vector

    Vector3d v_i = phHalfCur0->m_pheNode->m_pHandle->Traits().Vector3d(s0) -
                   phHalfCur1->m_pheNode->m_pHandle->Traits().Vector3d(s0);
    v_i.normalize();
    Vector2d t_i = mA * (v_i.cross(n_i_0).normalized());

    S +=
        (angle_i_x - angle_i_0) * pEdge_i->Length(s0) * (t_i * t_i.transpose());

    phHalfCur0 = phHalfCur0->m_pheNext;
    phHalfCur1 = phHalfCur0->m_phePair;
  } while (phHalfCur0 != phHalfIni);

  return S / (2 * pFace->Area(s0));
}

bool Mesh_Face::IsBoundary(const Node* pNode) {
  throw exception("Not implemented");

  return false;
}

bool Mesh_Face::IsBoundary(const Edge* pEdge) {
  HE_Edge* pheEdge = this->GetHEEdge(pEdge->ID());
  return pheEdge->m_pheHalf0 == NULL || pheEdge->m_pheHalf1 == NULL;
}

bool Mesh_Face::IsBoundary(const Face* pFace, bool node) {
  HE_Half* pheHalf = this->GetHEFace(pFace->ID())->m_pheHalf;

  bool hasBoundaryEdge = false;
  hasBoundaryEdge |= IsBoundary(pheHalf->m_pheEdge->m_pHandle);
  pheHalf = pheHalf->m_pheNext;
  hasBoundaryEdge |= IsBoundary(pheHalf->m_pheEdge->m_pHandle);
  pheHalf = pheHalf->m_pheNext;
  hasBoundaryEdge |= IsBoundary(pheHalf->m_pheEdge->m_pHandle);

  if (!hasBoundaryEdge && node) {
    bool hasBoundaryNode = false;
    hasBoundaryNode |=
        IsBoundary(pheHalf->m_phePair->m_pheFace->m_pHandle, false);
    pheHalf = pheHalf->m_pheNext;
    hasBoundaryNode |=
        IsBoundary(pheHalf->m_phePair->m_pheFace->m_pHandle, false);
    pheHalf = pheHalf->m_pheNext;
    hasBoundaryNode |=
        IsBoundary(pheHalf->m_phePair->m_pheFace->m_pHandle, false);
    return hasBoundaryNode;
  }

  return hasBoundaryEdge;
}

void Mesh_Face::GetBoundaryNodes(vector<Node*>& vbnode) {
  for (int i = 0; i < this->m_vnodes.size(); ++i)
    if (this->IsBoundary(m_vnodes[i]))
      vbnode.push_back(m_vnodes[i]);
}

void Mesh_Face::GetBoundaryEdges(vector<Edge*>& vbedge) {
  for (int i = 0; i < this->m_vedges.size(); ++i)
    if (this->IsBoundary(m_vedges[i]))
      vbedge.push_back(m_vedges[i]);
}

void Mesh_Face::GetBoundaryFaces(vector<Face*>& vbface, bool node) {
  for (int i = 0; i < this->m_vfaces.size(); ++i)
    if (this->IsBoundary(m_vfaces[i], node))
      vbface.push_back(m_vfaces[i]);
}

void Mesh_Face::GetInternalNodes(vector<Node*>& vinodes) {
  vector<Node*> vbnodes;
  this->GetBoundaryNodes(vbnodes);
  for (int i = 0; i < this->m_vnodes.size(); ++i)
    if (find(vbnodes.begin(), vbnodes.end(), m_vnodes[i]) == vbnodes.end())
      vinodes.push_back(m_vnodes[i]);
}

void Mesh_Face::GetInternalEdges(vector<Edge*>& viedges) {
  throw exception("Not implemented");
}

void Mesh_Face::GetInternalFaces(vector<Face*>& vifaces, bool node) {
  throw exception("Not implemented");
}

void Mesh_Face::GetBoundaryLoop(vector<Node*>& vbnode) {
  throw exception("Not implemented");
}

void Mesh_Face::UpdateMetadata() {
  this->FreeMetadata();

  // Initialize mesh faces vector

  this->m_vfaces.reserve(this->NumElems());
  for (int i = 0; i < this->m_velems.size(); ++i)
    this->m_vfaces.push_back(static_cast<Face*>(m_velems[i]));

  // Initialize HE structure

  map<IntPair, HE_Edge*> mheEdge;
  map<IntPair, HE_Half*> mheHalf;

  this->m_vheFaces.resize(this->NumFaces());
  this->m_vheNodes.resize(this->NumNodes());
  this->m_vheHalfs.reserve(3 * this->NumFaces());
  this->m_vheEdges.reserve(3 * this->NumFaces());
  this->m_vedges.reserve(3 * this->NumFaces());

  // Traverse faces

  for (int i = 0; i < this->m_vfaces.size(); ++i) {
    Face* pFace = this->m_vfaces[i];

    // Create HE face

    m_vheFaces[pFace->ID()] = new HE_Face();
    m_vheFaces[pFace->ID()]->m_pHandle = pFace;

    // Traverse face edges

    int NF = pFace->NumNodes();
    for (int i = 0; i < NF; ++i) {
      Node* pNode0 = pFace->Nodes()[(i + 0) % NF];
      Node* pNode1 = pFace->Nodes()[(i + 1) % NF];
      IntPair dir01(pNode0->ID(), pNode1->ID());
      IntPair dir10(pNode1->ID(), pNode0->ID());

      // Try to recover the previously created edge. If
      // the edge is already created, it must has been
      // done while visiting the neighbor face, thus,
      // in the opposite direction.

      HE_Edge* pheEdge = NULL;

      if (mheEdge.find(dir10) != mheEdge.end())
        pheEdge = mheEdge[dir10];  // Found

      // If the edge has not been created yet, create it

      if (pheEdge == NULL) {
        // Create edge

        vector<Node*> vedgeNodes(2);
        vedgeNodes[0] = pNode0;
        vedgeNodes[1] = pNode1;

        this->m_vedges.push_back(
            new Edge((int)this->m_vedges.size(), vedgeNodes));

        // Create HE edge

        this->m_vheEdges.push_back(new HE_Edge());
        mheEdge[dir01] = this->m_vheEdges.back();
        m_vheEdges.back()->m_pHandle = m_vedges.back();
        m_vheEdges.back()->m_pheHalf0 = NULL;
        m_vheEdges.back()->m_pheHalf1 = NULL;
        pheEdge = this->m_vheEdges.back();
      }

      // Create HE node

      if (this->m_vheNodes[pNode1->ID()] == NULL) {
        this->m_vheNodes[pNode1->ID()] = new HE_Node();
        this->m_vheNodes[pNode1->ID()]->m_pHandle = pNode1;
        this->m_vheNodes[pNode1->ID()]->m_pheHalf = NULL;
      }

      // Create HE half

      this->m_vheHalfs.push_back(new HE_Half());
      mheHalf[dir01] = this->m_vheHalfs.back();
      this->m_vheHalfs.back()->m_pheEdge = pheEdge;
      this->m_vheHalfs.back()->m_pheFace = m_vheFaces[pFace->ID()];
      this->m_vheHalfs.back()->m_pheNode = m_vheNodes[pNode1->ID()];
      m_vheFaces[pFace->ID()]->m_pheHalf = this->m_vheHalfs.back();
      this->m_vheHalfs.back()->m_phePair = NULL;
      this->m_vheHalfs.back()->m_pheNext = NULL;
      if (pheEdge->m_pheHalf0 == NULL)
        pheEdge->m_pheHalf0 = this->m_vheHalfs.back();
      else
        pheEdge->m_pheHalf1 = this->m_vheHalfs.back();
    }
  }

  // Initialize circulation links

  // Traverse faces

  for (int i = 0; i < this->m_vfaces.size(); ++i) {
    Face* pFace = this->m_vfaces[i];

    // Traverse face edges

    int NF = pFace->NumNodes();
    for (int i = 0; i < NF; ++i) {
      Node* pNode0 = pFace->Nodes()[(i + 0) % NF];
      Node* pNode1 = pFace->Nodes()[(i + 1) % NF];
      Node* pNode2 = pFace->Nodes()[(i + 2) % NF];
      IntPair dir01(pNode0->ID(), pNode1->ID());
      IntPair dir10(pNode1->ID(), pNode0->ID());
      IntPair dir12(pNode1->ID(), pNode2->ID());

      // Set next half edges

      mheHalf[dir01]->m_pheNext = mheHalf[dir12];

      // Set pair half edges

      if (mheHalf.find(dir01) != mheHalf.end() &&
          mheHalf.find(dir10) != mheHalf.end()) {
        mheHalf[dir01]->m_phePair = mheHalf[dir10];
        mheHalf[dir10]->m_phePair = mheHalf[dir01];
      }

      // Set node out half

      if (mheHalf[dir01]->m_pheNode->m_pheHalf == NULL)
        mheHalf[dir01]->m_pheNode->m_pheHalf = mheHalf[dir12];
    }
  }
}
}  // namespace PhySim