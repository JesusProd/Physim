//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Frame.h>

#include <PhySim/Geometry/Meshes/Rod.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/GeometryUtils.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_Frame::Mesh_Frame() : Mesh_Edge() {
  this->UpdateMetadata();
}

Mesh_Frame::Mesh_Frame(const Mesh_Frame& toCopy) : Mesh_Edge(toCopy) {
  this->UpdateMetadata();
}

Mesh_Frame::Mesh_Frame(const MatrixXd& mV,
                       const MatrixXi& mE,
                       const vector<Frame3d>& vF,
                       const vector<Tag>& vnTraits,
                       const vector<Tag>& vfTraits) {
  this->Init(mV, mE, vF, vnTraits, vfTraits);
}

void Mesh_Frame::Init(const MatrixXd& mV,
                      const MatrixXi& mE,
                      const vector<Frame3d>& vF,
                      const vector<Tag>& vnTraits,
                      const vector<Tag>& vfTraits) {
  Mesh_Edge::Init(mV, mE, Discretization_Edge2, vnTraits);

  // Add frames

  this->m_velemTraits = vfTraits;
  if (this->m_velemTraits.empty())
    this->m_velemTraits.push_back(Tag_Frame_X);

  for (int i = 0; i < (int)m_velems.size(); ++i) {
    for (int j = 0; j < (int)this->m_velemTraits.size(); ++j) {
      if (!vF.empty()) {
        m_velems[i]->Traits().AddTrait<Frame3d>(this->m_velemTraits[j],
                                                Frame3d(vF[i]));
      } else {
        Vector3d vt = this->GetEdge(i)->Tangent(this->m_vnodeTraits[j]);
        m_velems[i]->Traits().AddTrait<Frame3d>(this->m_velemTraits[j],
                                                Frame3d(vt));
      }
    }
  }

  if (vF.empty()) {
    for (int i = 0; i < (int)this->m_velemTraits.size(); ++i)
      this->SetRefFrames_TwistFreeRods(this->m_velemTraits[i]);
  }
}

Mesh_Frame::~Mesh_Frame(void) {
  this->FreeInternal();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Frame");
#endif
}

void Mesh_Frame::Merge(PtrS<Mesh_Frame>& pMesh,
                       const vector<PtrS<Mesh_Frame>>& vmeshes,
                       Tag trait,
                       Real tol) {
  PtrS<Mesh_Edge> pMeshMerged = dynamic_pointer_cast<Mesh_Edge>(pMesh);

  vector<PtrS<Mesh_Edge>> vbaseMeshes(vmeshes.size());
  for (int i = 0; i < (int)vmeshes.size(); ++i)
    vbaseMeshes[i] = dynamic_pointer_cast<Mesh_Edge>(vmeshes[i]);

  Mesh_Edge::Merge(pMeshMerged, vbaseMeshes, trait, tol);

  pMesh->UpdateMetadata();
}

void Mesh_Frame::GetFrameVector(vector<Frame3d>& vF, Tag s) const {
  vF.resize(this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    vF[i] = this->Elems()[i]->Traits().Frame3d(s);
}

void Mesh_Frame::SetFrameVector(const vector<Frame3d>& vF, Tag s) {
  assert((int)vF.size() == this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    this->Elems()[i]->Traits().Frame3d(s) = vF[i];
}

void Mesh_Frame::FreeMetadata() {
  // Not implemented
}

void Mesh_Frame::UpdateMetadata() {
  this->FreeMetadata();

  // Not implemented
}

void Mesh_Frame::SetRefFrames_TwistFreeRods(Tag s) {
  // Initialized connected rods

  bVector vvisited(this->m_velems.size(), false);
  for (int i = 0; i < (int)m_vconns.size(); ++i) {
    for (int j = 0; j < (int)m_vconns[i]->m_vedges.size(); ++j) {
      Edge* pEdge = this->m_vconns[i]->m_vedges[j];

      if (vvisited[pEdge->ID()])
        continue;  // Initialized

      // Get complete rod

      vector<Edge*> vrod;
      GetRod(pEdge, vrod);

      // Mark all elements as visited

      for (int k = 0; k < (int)vrod.size(); ++k)
        vvisited[vrod[k]->ID()] = true;

      if (m_vconns[i]->m_vedges[j]->Traits().Int(Tag::Tag_Sign_0) > 0)
        ParallalelTransportForward(s, pEdge->Traits().Frame3d(s), vrod);
      else
        ParallalelTransportBackward(s, pEdge->Traits().Frame3d(s), vrod);
    }
  }

  // Initialized not connected rods

  for (int i = 0; i < (int)vvisited.size(); ++i) {
    if (vvisited[i])
      continue;

    Edge* pEdge = this->m_vedges[i];

    vector<Edge*> vrod;
    GetRod(pEdge, vrod);

    ParallalelTransportForward(s, pEdge->Traits().Frame3d(s), vrod);
  }
}

void Mesh_Frame::GetRod(Edge* pEdge, vector<Edge*>& vrod) const {
  Edge* pCurr = NULL;

  vrod.push_back(pEdge);

  // Look forward

  pCurr = pEdge;

  vector<Edge*> vnext = this->NextEdges(pCurr);

  while (!m_vnode2connMap[pCurr->GetHead()->ID()] && vnext.size() == 1) {
    vrod.insert(vrod.end(), vnext[0]);
    pCurr = vnext[0];
    vnext = NextEdges(pCurr);
  }

  // Look backward

  pCurr = pEdge;

  vector<Edge*> vprev = this->PrevEdges(pCurr);

  while (!m_vnode2connMap[pCurr->GetTail()->ID()] && vprev.size() == 1) {
    vrod.insert(vrod.begin(), vprev[0]);
    pCurr = vprev[0];
    vprev = PrevEdges(pCurr);
  }
}

void Mesh_Frame::ParallalelTransportForward(Tag s,
                                            const Frame3d& F,
                                            vector<Edge*> vrod) {
  vrod.front()->Traits().Frame3d(s) = GeometryUtils::parallelTransport(
      F, vrod.front()->Traits().Frame3d(s).tan);

  for (int i = 1; i <= (int)vrod.size() - 1; ++i)
    vrod[i]->Traits().Frame3d(s) = GeometryUtils::parallelTransport(
        vrod[i - 1]->Traits().Frame3d(s), vrod[i]->Traits().Frame3d(s).tan);
}

void Mesh_Frame::ParallalelTransportBackward(Tag s,
                                             const Frame3d& F,
                                             vector<Edge*> vrod) {
  vrod.back()->Traits().Frame3d(s) = GeometryUtils::parallelTransport(
      F, vrod.front()->Traits().Frame3d(s).tan);

  for (int i = (int)vrod.size() - 2; i >= 0; --i)
    vrod[i]->Traits().Frame3d(s) = GeometryUtils::parallelTransport(
        vrod[i + 1]->Traits().Frame3d(s), vrod[i]->Traits().Frame3d(s).tan);
}

void Mesh_Frame::SetRefFrames_FromConnectionNormals(Tag s,
                                                    const vector<Vector3d>& vn,
                                                    bool twistFree) {
  // TODO
}

void Mesh_Frame::SetRefFrames_FromRotatedRefFrames(Tag sourceTrait,
                                                   Tag targetTrait,
                                                   bool twistFree) {
  // TODO
}

}  // namespace PhySim