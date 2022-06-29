//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Edge.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_Edge::Mesh_Edge() {
  // Nothing to do here...
}

Mesh_Edge::Mesh_Edge(const Mesh_Edge& toCopy) : Mesh(toCopy) {
  this->UpdateMetadata();
}

Mesh_Edge::Mesh_Edge(const MatrixXd& mV,
                     const MatrixXi& mE,
                     Discretization D,
                     const vector<Tag>& vnTraits)
    : Mesh(mV, mE, D, vnTraits) {
  this->UpdateMetadata();
}

void Mesh_Edge::Init(const MatrixXd& mV,
                     const MatrixXi& mE,
                     Discretization D,
                     const vector<Tag>& vnTraits) {
  Mesh::Init(mV, mE, D, vnTraits);

  this->UpdateMetadata();
}

Mesh_Edge::~Mesh_Edge(void) {
  this->FreeInternal();

  this->FreeMetadata();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Edge");
#endif
}

void Mesh_Edge::Merge(PtrS<Mesh_Edge>& pMesh,
                      const vector<PtrS<Mesh_Edge>>& vmeshes,
                      Tag trait,
                      Real tol) {
  PtrS<Mesh> pMeshMerged = dynamic_pointer_cast<Mesh>(pMesh);

  vector<PtrS<Mesh>> vbaseMeshes(vmeshes.size());
  for (int i = 0; i < (int)vmeshes.size(); ++i)
    vbaseMeshes[i] = dynamic_pointer_cast<Mesh>(vmeshes[i]);

  Mesh::Merge(pMeshMerged, vbaseMeshes, trait, tol);

  pMesh->UpdateMetadata();
}

vector<Node*> Mesh_Edge::NextNodes(Node* pn) const {
  vector<Node*> vnodes;
  for (int i = 0; i < m_vnodeAdjaEdges[pn->ID()].size(); ++i)
    if (m_vnodeAdjaEdges[pn->ID()][i]->GetTail() == pn)
      vnodes.push_back(m_vnodeAdjaEdges[pn->ID()][i]->GetHead());
  return vnodes;
}

vector<Node*> Mesh_Edge::PrevNodes(Node* pn) const {
  vector<Node*> vnodes;
  for (int i = 0; i < m_vnodeAdjaEdges[pn->ID()].size(); ++i)
    if (m_vnodeAdjaEdges[pn->ID()][i]->GetHead() == pn)
      vnodes.push_back(m_vnodeAdjaEdges[pn->ID()][i]->GetTail());
  return vnodes;
}

vector<Edge*> Mesh_Edge::NextEdges(Node* pn) const {
  vector<Edge*> vedges;
  for (int i = 0; i < m_vnodeAdjaEdges[pn->ID()].size(); ++i)
    if (m_vnodeAdjaEdges[pn->ID()][i]->GetTail() == pn)
      vedges.push_back(m_vnodeAdjaEdges[pn->ID()][i]);
  return vedges;
}

vector<Edge*> Mesh_Edge::PrevEdges(Node* pn) const {
  vector<Edge*> vedges;
  for (int i = 0; i < m_vnodeAdjaEdges[pn->ID()].size(); ++i)
    if (m_vnodeAdjaEdges[pn->ID()][i]->GetHead() == pn)
      vedges.push_back(m_vnodeAdjaEdges[pn->ID()][i]);
  return vedges;
}

vector<Edge*> Mesh_Edge::PrevEdges(Edge* pe) const {
  return this->m_vedgePrevEdges[pe->ID()];
}

vector<Edge*> Mesh_Edge::NextEdges(Edge* pe) const {
  return this->m_vedgeNextEdges[pe->ID()];
}

vector<Node*> Mesh_Edge::NextNodes(Edge* pe) const {
  vector<Node*> vnodes;
  for (int i = 0; i < m_vedgeNextEdges[pe->ID()].size(); ++i)
    vnodes.push_back(m_vedgeNextEdges[pe->ID()][i]->GetHead());

  return vnodes;
}

vector<Node*> Mesh_Edge::PrevNodes(Edge* pe) const {
  vector<Node*> vnodes;
  for (int i = 0; i < m_vedgePrevEdges[pe->ID()].size(); ++i)
    vnodes.push_back(m_vedgePrevEdges[pe->ID()][i]->GetTail());
  return vnodes;
}

void Mesh_Edge::FreeMetadata() {
  this->m_vnodeAdjaEdges.clear();
  this->m_vedgeNextEdges.clear();
  this->m_vedgePrevEdges.clear();
  this->m_vnode2connMap.clear();

  for (int i = 0; i < this->m_vconns.size(); ++i)
    delete m_vconns[i];
  this->m_vconns.clear();
}

void Mesh_Edge::UpdateMetadata() {
  this->FreeMetadata();

  // Initialize edges

  this->m_vedges.resize(this->m_velems.size());
  for (int i = 0; i < this->m_velems.size(); ++i)
    this->m_vedges[i] = (Edge*)this->m_velems[i];

  // Node adjacent edges

  this->m_vnodeAdjaEdges.resize(this->NumNodes());

  for (int i = 0; i < this->m_vedges.size(); ++i) {
    Edge* pe = this->m_vedges[i];

    m_vnodeAdjaEdges[pe->GetHead()->ID()].push_back(pe);
    m_vnodeAdjaEdges[pe->GetTail()->ID()].push_back(pe);
  }

  // Edge adjacent edges

  vector<vector<Edge*>> vedgeAdjaEdges(this->NumElems());

  for (int i = 0; i < this->m_vedges.size(); ++i) {
    Edge* pe = this->m_vedges[i];

    for (int j = 0; j < m_vnodeAdjaEdges[pe->GetTail()->ID()].size(); ++j) {
      Edge* po = m_vnodeAdjaEdges[pe->GetTail()->ID()][j];
      if (po != pe)
        vedgeAdjaEdges[pe->ID()].push_back(po);
    }

    for (int j = 0; j < m_vnodeAdjaEdges[pe->GetHead()->ID()].size(); ++j) {
      Edge* po = m_vnodeAdjaEdges[pe->GetHead()->ID()][j];
      if (po != pe)
        vedgeAdjaEdges[pe->ID()].push_back(po);
    }
  }

  // Ordering

  this->m_vedgeNextEdges.resize(this->m_vedges.size());
  this->m_vedgePrevEdges.resize(this->m_vedges.size());

  for (int i = 0; i < this->m_vedges.size(); ++i) {
    Edge* pe = this->m_vedges[i];

    for (int i = 0; i < m_vnodeAdjaEdges[pe->GetTail()->ID()].size(); ++i)
      if (m_vnodeAdjaEdges[pe->GetTail()->ID()][i] != pe) {
        if (m_vnodeAdjaEdges[pe->GetTail()->ID()][i]->GetHead() ==
            pe->GetTail()) {
          this->m_vedgePrevEdges[pe->ID()].push_back(
              m_vnodeAdjaEdges[pe->GetTail()->ID()][i]);
        }
      }

    for (int i = 0; i < m_vnodeAdjaEdges[pe->GetHead()->ID()].size(); ++i)
      if (m_vnodeAdjaEdges[pe->GetHead()->ID()][i] != pe) {
        if (m_vnodeAdjaEdges[pe->GetHead()->ID()][i]->GetTail() ==
            pe->GetHead()) {
          this->m_vedgeNextEdges[pe->ID()].push_back(
              m_vnodeAdjaEdges[pe->GetHead()->ID()][i]);
        }
      }
  }

  // Connections

  this->m_vnode2connMap.resize(this->m_vnodes.size(), -1);

  for (int i = 0; i < m_vnodes.size(); ++i) {
    Node* pn = this->m_vnodes[i];

    vector<Edge*>& vadjaEdges = m_vnodeAdjaEdges[pn->ID()];

    int numE = (int)vadjaEdges.size();

    if (numE == 1)
      continue;

    if (numE == 2) {
      Vector3d vt0 = vadjaEdges[0]->Tangent(this->m_vnodeTraits[0]);
      Vector3d vt1 = vadjaEdges[1]->Tangent(this->m_vnodeTraits[0]);

      if (acos(vt0.dot(vt1)) < M_PI / 6)
        continue;  // Not connection
    }

    Connection* pc = new Connection();
    pc->m_center = pn;
    pc->m_vedges.resize(numE);
    for (int i = 0; i < numE; ++i) {
      pc->m_vedges[i] = vadjaEdges[i];
      if (vadjaEdges[i]->GetTail() == pn)
        pc->m_vedges[i]->Traits().AddTrait(Tag::Tag_Sign_0, int(1));
      else
        pc->m_vedges[i]->Traits().AddTrait(Tag::Tag_Sign_0, int(-1));
    }

    this->m_vconns.push_back(pc);
    this->m_vnode2connMap[pn->ID()] = (int)this->m_vconns.size() - 1;
  }
}

Real Mesh_Edge::VolumeSpace(Tag s) const {
  if (this->DimSpace() == 1) {
    return this->VolumeBasis(s);
  }

  if (this->DimSpace() == 2) {
    // TODO: Compute enclosed volume

    throw PhySim::exception("Not implemented");
  }

  if (this->DimSpace() == 3) {
    // TODO: Compute enclosed volume

    throw PhySim::exception("Not implemented");
  }

  throw PhySim::exception("Unreachable section");
}

}  // namespace PhySim