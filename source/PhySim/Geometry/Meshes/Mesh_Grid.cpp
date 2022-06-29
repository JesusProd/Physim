//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Grid.h>

#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/IOUtils.h>
#include <PhySim/Utils/MeshUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_Grid::Mesh_Grid() : Mesh_Hexa() {
  this->UpdateMetadata();
}

Mesh_Grid::Mesh_Grid(const Mesh_Grid& toCopy) : Mesh_Hexa(toCopy) {
  throw PhySim::exception("Not implemented");

  this->UpdateMetadata();
}

Mesh_Grid::Mesh_Grid(const Vector3d& vorigin,
                     const Vector3d& vsize,
                     const Vector3i& vdims,
                     const iVector& voccu,
                     Discretization D,
                     const vector<Tag>& vnTraits)
    : Mesh_Hexa() {
  this->Init(vorigin, vsize, vdims, voccu, D, vnTraits);
}

void Mesh_Grid::Init(const Vector3d& vorigin,
                     const Vector3d& vsize,
                     const Vector3i& vdims,
                     const iVector& voccu,
                     Discretization D,
                     const vector<Tag>& vnTraits) {
  this->m_vorigin = vorigin;
  this->m_vsize = vsize;
  this->m_vdims = vdims;
  this->m_voccu = voccu;

  // Create active fine mesh

  MatrixXd mFineActiveNodes;
  MatrixXi mFineActiveElems;
  this->InitializeActiveMesh(
      m_vorigin, m_vsize, m_vdims, m_voccu, mFineActiveNodes, mFineActiveElems,
      this->m_vmapFull2Active_Nodes, this->m_vmapFull2Active_Elems,
      this->m_vmapActive2Full_Nodes, this->m_vmapActive2Full_Elems);

  Mesh_Hexa::Init(mFineActiveNodes, mFineActiveElems, D, vnTraits);

  this->UpdateMetadata();
}

void Mesh_Grid::InitializeActiveMesh(const Vector3d& vorigin,
                                     const Vector3d& vsize,
                                     const Vector3i& vdims,
                                     const iVector& voccup,
                                     MatrixXd& mNode,
                                     MatrixXi& mElem,
                                     iVector& vfull2Active_Node,
                                     iVector& vfull2Active_Elem,
                                     iVector& vactive2Full_Node,
                                     iVector& vactive2Full_Elem)

{
  // Create full fine node and hexahedron matrix

  MatrixXd mFullNodes;
  MatrixXi mFullElems;
  MeshUtils::create3DGridNodes(vdims.x() + 1, vdims.y() + 1, vdims.z() + 1,
                               vsize, mFullNodes);
  MeshUtils::create3DGridIndex_Hex8(vdims.x() + 1, vdims.y() + 1, vdims.z() + 1,
                                    mFullElems);
  mFullNodes.rowwise() += vorigin.transpose();

  // Collect active elements and nodes and create
  // the mapping between full grid and actives

  vfull2Active_Node.resize(mFullNodes.rows(), -1);
  vfull2Active_Elem.resize(mFullElems.rows(), -1);
  vactive2Full_Node.reserve(voccup.size());
  vactive2Full_Elem.reserve(voccup.size());

  int countNode = 0;
  int countElem = 0;
  std::list<Vector3d> vActiveNodes;
  std::list<VectorXi> vActiveElems;

  for (int i = 0; i < (int)voccup.size(); ++i) {
    vfull2Active_Elem[voccup[i]] = countElem;
    vactive2Full_Elem.push_back(voccup[i]);

    // Collect node indices

    VectorXi thisActiveElem(8);
    for (int j = 0; j < 8; ++j) {
      if (vfull2Active_Node[mFullElems(voccup[i], j)] == -1) {
        vfull2Active_Node[mFullElems(voccup[i], j)] = countNode;
        vactive2Full_Node.push_back(mFullElems(voccup[i], j));
        vActiveNodes.push_back(mFullNodes.row(mFullElems(voccup[i], j)));
        countNode++;
      }
      thisActiveElem(j) = vfull2Active_Node[mFullElems(voccup[i], j)];
    }
    vActiveElems.push_back(thisActiveElem);

    countElem++;
  }

  // Create active mesh

  mNode.resize(vActiveNodes.size(), 3);
  mElem.resize(vActiveElems.size(), 8);
  countNode = 0;
  countElem = 0;
  for (list<Vector3d>::iterator itCur = vActiveNodes.begin();
       itCur != vActiveNodes.end(); itCur++, countNode++)
    mNode.row(countNode) = (*itCur);
  for (list<VectorXi>::iterator itCur = vActiveElems.begin();
       itCur != vActiveElems.end(); itCur++, countElem++)
    mElem.row(countElem) = (*itCur);
}

void Mesh_Grid::InitializeMetaData(const Vector3i& vdims,
                                   const vector<Node*> vnodes,
                                   const vector<Poly*> velems,
                                   const iVector& vfull2active_elems,
                                   const iVector& vactive2full_elems,
                                   const iVector& vfull2active_nodes,
                                   const iVector& vactive2full_nodes) {
  vector<GridNodeMeta> vnodeMeta(vnodes.size());
  vector<GridElemMeta> velemMeta(velems.size());

  // NODES

  // Initialize node neighbor elements

  for (int i = 0; i < (int)vnodes.size(); ++i)
    vnodeMeta[i].vNeiElems.resize(8, NULL);

  for (int i = 0; i < (int)velems.size(); ++i)
    for (int j = 0; j < 8; ++j) {
      const Node* pNode = velems[i]->Nodes()[j];
      vnodeMeta[pNode->ID()].vNeiElems[j] = velems[i];
    }

  for (int i = 0; i < (int)vnodes.size(); ++i) {
    // Is it in the surface?

    vnodeMeta[i].surfaces = 0;
    for (int j = 0; j < 8; ++j)
      if (vnodeMeta[i].vNeiElems[j] == NULL)
        vnodeMeta[i].surfaces++;

    // Initialize full index

    vnodeMeta[i].fullIdx = vactive2full_nodes[i];
  }

  // ELEMS

  this->m_mdualEdges.clear();

  for (int i = 0; i < (int)velems.size(); ++i) {
    GridElemMeta& elemMeta = velemMeta[i];

    // Initialize elements full index

    elemMeta.fullIdx = vactive2full_elems[i];

    // Initialize elements neighbor elements

    Vector3i fullCoord = TransformRegularIndexToCoord(
        vdims.x(), vdims.y(), vdims.z(), vactive2full_elems[i]);

    int selfElemIdx = i;
    bool surface = false;

    // -X
    if (fullCoord.x() > 0) {
      int neigElemIdx = vfull2active_elems[TransformRegularCoordToIndex(
          vdims.x(), vdims.y(), vdims.z(),
          Vector3i(fullCoord.x() - 1, fullCoord.y(), fullCoord.z()))];
      if (neigElemIdx >= 0) {
        pair<int, int> key(neigElemIdx, selfElemIdx);
        if (m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = velems[neigElemIdx];
          dualEdge->elem1 = velems[selfElemIdx];
          dualEdge->Dir = 0;
          m_mdualEdges[key] = dualEdge;
        }
        elemMeta.vDualEdgesXm.push_back(m_mdualEdges[key]);
      } else
        surface = true;
    }

    // +X
    if (fullCoord.x() + 1 < vdims.x()) {
      int neigElemIdx = vfull2active_elems[TransformRegularCoordToIndex(
          vdims.x(), vdims.y(), vdims.z(),
          Vector3i(fullCoord.x() + 1, fullCoord.y(), fullCoord.z()))];
      if (neigElemIdx >= 0) {
        pair<int, int> key(selfElemIdx, neigElemIdx);
        if (m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = velems[selfElemIdx];
          dualEdge->elem1 = velems[neigElemIdx];
          dualEdge->Dir = 0;
          m_mdualEdges[key] = dualEdge;
        }
        elemMeta.vDualEdgesXp.push_back(m_mdualEdges[key]);
      } else
        surface = true;
    }

    // -Y
    if (fullCoord.y() > 0) {
      int neigElemIdx = vfull2active_elems[TransformRegularCoordToIndex(
          vdims.x(), vdims.y(), vdims.z(),
          Vector3i(fullCoord.x(), fullCoord.y() - 1, fullCoord.z()))];
      if (neigElemIdx >= 0) {
        pair<int, int> key(neigElemIdx, selfElemIdx);
        if (m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = velems[neigElemIdx];
          dualEdge->elem1 = velems[selfElemIdx];
          dualEdge->Dir = 1;
          m_mdualEdges[key] = dualEdge;
        }
        elemMeta.vDualEdgesYm.push_back(m_mdualEdges[key]);
      } else
        surface = true;
    }

    // +Y
    if (fullCoord.y() + 1 < m_vdims.y()) {
      int neigElemIdx = vfull2active_elems[TransformRegularCoordToIndex(
          vdims.x(), vdims.y(), vdims.z(),
          Vector3i(fullCoord.x(), fullCoord.y() + 1, fullCoord.z()))];
      if (neigElemIdx >= 0) {
        pair<int, int> key(selfElemIdx, neigElemIdx);
        if (m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = velems[selfElemIdx];
          dualEdge->elem1 = velems[neigElemIdx];
          dualEdge->Dir = 1;
          m_mdualEdges[key] = dualEdge;
        }
        elemMeta.vDualEdgesYp.push_back(m_mdualEdges[key]);
      } else
        surface = true;
    }

    // -Z
    if (fullCoord.z() > 0) {
      int neigElemIdx = vfull2active_elems[TransformRegularCoordToIndex(
          vdims.x(), vdims.y(), vdims.z(),
          Vector3i(fullCoord.x(), fullCoord.y(), fullCoord.z() - 1))];
      if (neigElemIdx >= 0) {
        pair<int, int> key(neigElemIdx, selfElemIdx);
        if (m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = velems[neigElemIdx];
          dualEdge->elem1 = velems[selfElemIdx];
          dualEdge->Dir = 2;
          m_mdualEdges[key] = dualEdge;
        }
        elemMeta.vDualEdgesZm.push_back(m_mdualEdges[key]);
      } else
        surface = true;
    }

    // +Z
    if (fullCoord.z() + 1 < m_vdims.z()) {
      int neigElemIdx = vfull2active_elems[TransformRegularCoordToIndex(
          vdims.x(), vdims.y(), vdims.z(),
          Vector3i(fullCoord.x(), fullCoord.y(), fullCoord.z() + 1))];
      if (neigElemIdx >= 0) {
        pair<int, int> key(selfElemIdx, neigElemIdx);
        if (m_mdualEdges.find(key) == m_mdualEdges.end()) {
          PtrS<DEdge> dualEdge(new DEdge());
          dualEdge->ID = (int)m_mdualEdges.size();
          dualEdge->elem0 = velems[selfElemIdx];
          dualEdge->elem1 = velems[neigElemIdx];
          dualEdge->Dir = 2;
          m_mdualEdges[key] = dualEdge;
        }
        elemMeta.vDualEdgesZp.push_back(m_mdualEdges[key]);
      } else
        surface = true;
    }

    // Is it in the surface?

    elemMeta.surface = surface;
  }

  // Initialize metadata

  for (int i = 0; i < (int)vnodes.size(); ++i)
    vnodes[i]->Traits().AddTrait<GridNodeMeta>(Tag::Tag_GridMeta, vnodeMeta[i]);

  for (int i = 0; i < (int)velems.size(); ++i)
    velems[i]->Traits().AddTrait<GridElemMeta>(Tag::Tag_GridMeta, velemMeta[i]);
}

void Mesh_Grid::RemoveIsolatedNodes() {
  bVector vused(this->m_vnodes.size(), false);
  for (int i = 0; i < (int)this->m_velems.size(); ++i)
    for (int j = 0; j < (int)this->m_velems[i]->NumNodes(); ++j)
      vused[this->m_velems[i]->Nodes()[j]->ID()] = true;

  list<Node*> lnodes;
  for (int i = 0; i < (int)this->m_vnodes.size(); ++i)
    if (!vused[i])
      lnodes.push_back(this->m_vnodes[i]);

  this->RemoveNodes(lnodes);
}

void Mesh_Grid::RemoveNodes(const list<Node*>& lnodes) {
  for (auto it = lnodes.begin(); it != lnodes.end(); ++it)
    (*it)->ID() = -1;  // Mark those elems to be removed!

  // Remove elems

  int IDcollapse = 0;

  for (int i = 0; i < (int)this->m_vnodes.size(); ++i) {
    if (this->m_vnodes[i]->ID() == -1) {
      IDcollapse++;
      m_vnodes.erase(m_vnodes.begin() + i--);
    } else {
      m_vnodes[i]->ID() -= IDcollapse;
    }
  }
}

void Mesh_Grid::RemoveElems(const list<Poly*>& lelems) {
  for (auto it = lelems.begin(); it != lelems.end(); ++it)
    (*it)->ID() = -1;  // Mark those elems to be removed!

  // Remove elems

  int IDcollapse = 0;

  set<DEdge*> sedges;

  for (int i = 0; i < (int)this->m_velems.size(); ++i) {
    if (this->m_velems[i]->ID() == -1) {
      GridElemMeta& meta =
          this->m_velems[i]->Trait<GridElemMeta>(Tag::Tag_GridMeta);
      IDcollapse++;
      m_velems.erase(m_velems.begin() + i--);
      for (auto it = meta.vDualEdgesXm.begin(); it != meta.vDualEdgesXm.end();
           ++it)
        sedges.insert((*it).get());
      for (auto it = meta.vDualEdgesXp.begin(); it != meta.vDualEdgesXp.end();
           ++it)
        sedges.insert((*it).get());
      for (auto it = meta.vDualEdgesYm.begin(); it != meta.vDualEdgesYm.end();
           ++it)
        sedges.insert((*it).get());
      for (auto it = meta.vDualEdgesYp.begin(); it != meta.vDualEdgesYp.end();
           ++it)
        sedges.insert((*it).get());
      for (auto it = meta.vDualEdgesZm.begin(); it != meta.vDualEdgesZm.end();
           ++it)
        sedges.insert((*it).get());
      for (auto it = meta.vDualEdgesZp.begin(); it != meta.vDualEdgesZp.end();
           ++it)
        sedges.insert((*it).get());
    } else {
      m_velems[i]->ID() -= IDcollapse;
    }
  }

  this->m_vcells.clear();
  for (auto it = this->m_velems.begin(); it != this->m_velems.end(); ++it)
    this->m_vcells.push_back((Cell*)(*it));

  // Remove edges

  list<DEdge*> ledges;
  ledges.insert(ledges.begin(), sedges.begin(), sedges.end());
  this->RemoveEdges(ledges);
}

void Mesh_Grid::RemoveEdges(const list<DEdge*>& ledges) {
  // Create vector of sorted dual edges

  vector<PtrS<DEdge>> vedges(m_mdualEdges.size());
  for (auto it = m_mdualEdges.begin(); it != m_mdualEdges.end(); ++it)
    vedges[it->second->ID] = it->second;

  for (auto it = ledges.begin(); it != ledges.end(); ++it)
    (*it)->ID = -1;  // Mark those edges to be removed!

  // Remove edges and collapse IDs

  int IDcollapse = 0;

  for (int i = 0; i < (int)vedges.size(); ++i) {
    if (vedges[i]->ID == -1) {
      GridElemMeta& meta0 =
          vedges[i]->elem0->Trait<GridElemMeta>(Tag::Tag_GridMeta);
      GridElemMeta& meta1 =
          vedges[i]->elem1->Trait<GridElemMeta>(Tag::Tag_GridMeta);
      meta0.surface = true;
      meta1.surface = true;
      IDcollapse++;
      vector<PtrS<DEdge>>::iterator it;
      switch (vedges[i]->Dir) {
        case 0:
          it = find(meta0.vDualEdgesXp.begin(), meta0.vDualEdgesXp.end(),
                    vedges[i]);
          if (it != meta0.vDualEdgesXp.end())
            meta0.vDualEdgesXp.erase(it);
          it = find(meta1.vDualEdgesXm.begin(), meta1.vDualEdgesXm.end(),
                    vedges[i]);
          if (it != meta1.vDualEdgesXm.end())
            meta1.vDualEdgesXm.erase(it);
          // meta0.vDualEdgesYp.erase(find(meta0.vDualEdgesXp.begin(),
          // meta0.vDualEdgesXp.end(), vedges[i]));
          // meta1.vDualEdgesYm.erase(find(meta1.vDualEdgesXm.begin(),
          // meta1.vDualEdgesXm.end(), vedges[i]));
          break;
        case 1:
          it = find(meta0.vDualEdgesYp.begin(), meta0.vDualEdgesYp.end(),
                    vedges[i]);
          if (it != meta0.vDualEdgesYp.end())
            meta0.vDualEdgesYp.erase(it);
          it = find(meta1.vDualEdgesYm.begin(), meta1.vDualEdgesYm.end(),
                    vedges[i]);
          if (it != meta1.vDualEdgesYm.end())
            meta1.vDualEdgesYm.erase(it);
          // meta0.vDualEdgesYp.erase(find(meta0.vDualEdgesYp.begin(),
          // meta0.vDualEdgesYp.end(), vedges[i]));
          // meta1.vDualEdgesYm.erase(find(meta1.vDualEdgesYm.begin(),
          // meta1.vDualEdgesYm.end(), vedges[i]));
          break;
        case 2:
          it = find(meta0.vDualEdgesZp.begin(), meta0.vDualEdgesZp.end(),
                    vedges[i]);
          if (it != meta0.vDualEdgesZp.end())
            meta0.vDualEdgesZp.erase(it);
          it = find(meta1.vDualEdgesZm.begin(), meta1.vDualEdgesZm.end(),
                    vedges[i]);
          if (it != meta1.vDualEdgesZm.end())
            meta1.vDualEdgesZm.erase(it);
          // meta0.vDualEdgesZp.erase(find(meta0.vDualEdgesZp.begin(),
          // meta0.vDualEdgesZp.end(), vedges[i]));
          // meta1.vDualEdgesZm.erase(find(meta1.vDualEdgesZm.begin(),
          // meta1.vDualEdgesZm.end(), vedges[i]));
          break;
      }
      vedges.erase(vedges.begin() + i--);
    } else {
      vedges[i]->ID -= IDcollapse;
    }
  }

  // Recompute the map

  this->m_mdualEdges.clear();
  for (auto it = vedges.begin(); it != vedges.end(); ++it)
    this->m_mdualEdges[pair<int, int>((*it)->elem0->ID(), (*it)->elem1->ID())] =
        (*it);
}

vector<Node*> Mesh_Grid::ClusterNodes(map<Poly*, vector<Node*>>& mcandidates) {
  // Traverse creating clusters of nodes that are the same one

  vector<set<Node*>> vclusters;

  for (auto itCur = mcandidates.begin(); itCur != mcandidates.end(); ++itCur) {
    // Substitute some of this nodes

    GridElemMeta& gridElemMeta =
        itCur->first->Trait<GridElemMeta>(Tag::Tag_GridMeta);

    // X-
    {
      const vector<PtrS<DEdge>>& vNeiEdges = gridElemMeta.vDualEdgesXm;
      for (int i = 0; i < (int)vNeiEdges.size(); ++i) {
        int vthisSide[4] = {0, 3, 4, 7};
        int vthatSide[4] = {1, 2, 5, 6};
        Poly* pThatElem = vNeiEdges[i]->elem0;
        for (int j = 0; j < 4; ++j) {
          Node* pThisNode = itCur->second[vthisSide[j]];
          Node* pThatNode = NULL;
          if (mcandidates.find(pThatElem) == mcandidates.end())
            pThatNode = pThatElem->Nodes()[vthatSide[j]];
          else
            pThatNode = mcandidates[pThatElem][vthatSide[j]];
          assert((pThisNode->Traits().Vector3d(Tag::Tag_Position_X) -
                  pThatNode->Traits().Vector3d(Tag::Tag_Position_X))
                     .norm() < 1e-6);
          bool found = false;
          for (auto it = vclusters.begin(); it != vclusters.end(); ++it)
            if ((*it).find(pThisNode) != (*it).end() ||
                (*it).find(pThatNode) != (*it).end()) {
              (*it).insert(pThisNode);
              (*it).insert(pThatNode);
              found = true;
              break;
            }
          if (!found) {
            vclusters.push_back(set<Node*>());
            vclusters.back().insert(pThisNode);
            vclusters.back().insert(pThatNode);
          }
        }
      }
    }
    // X+
    {
      const vector<PtrS<DEdge>>& vNeiEdges = gridElemMeta.vDualEdgesXp;
      for (int i = 0; i < (int)vNeiEdges.size(); ++i) {
        int vthisSide[4] = {1, 2, 5, 6};
        int vthatSide[4] = {0, 3, 4, 7};
        Poly* pThatElem = vNeiEdges[i]->elem1;
        for (int j = 0; j < 4; ++j) {
          Node* pThisNode = itCur->second[vthisSide[j]];
          Node* pThatNode = NULL;
          if (mcandidates.find(pThatElem) == mcandidates.end())
            pThatNode = pThatElem->Nodes()[vthatSide[j]];
          else
            pThatNode = mcandidates[pThatElem][vthatSide[j]];
          assert((pThisNode->Traits().Vector3d(Tag::Tag_Position_X) -
                  pThatNode->Traits().Vector3d(Tag::Tag_Position_X))
                     .norm() < 1e-6);
          bool found = false;
          for (auto it = vclusters.begin(); it != vclusters.end(); ++it)
            if ((*it).find(pThisNode) != (*it).end() ||
                (*it).find(pThatNode) != (*it).end()) {
              (*it).insert(pThisNode);
              (*it).insert(pThatNode);
              found = true;
              break;
            }
          if (!found) {
            vclusters.push_back(set<Node*>());
            vclusters.back().insert(pThisNode);
            vclusters.back().insert(pThatNode);
          }
        }
      }
    }
    // Y-
    {
      const vector<PtrS<DEdge>>& vNeiEdges = gridElemMeta.vDualEdgesYm;
      for (int i = 0; i < (int)vNeiEdges.size(); ++i) {
        int vthisSide[4] = {0, 1, 4, 5};
        int vthatSide[4] = {3, 2, 7, 6};
        Poly* pThatElem = vNeiEdges[i]->elem0;
        for (int j = 0; j < 4; ++j) {
          Node* pThisNode = itCur->second[vthisSide[j]];
          Node* pThatNode = NULL;
          if (mcandidates.find(pThatElem) == mcandidates.end())
            pThatNode = pThatElem->Nodes()[vthatSide[j]];
          else
            pThatNode = mcandidates[pThatElem][vthatSide[j]];
          assert((pThisNode->Traits().Vector3d(Tag::Tag_Position_X) -
                  pThatNode->Traits().Vector3d(Tag::Tag_Position_X))
                     .norm() < 1e-6);
          bool found = false;
          for (auto it = vclusters.begin(); it != vclusters.end(); ++it)
            if ((*it).find(pThisNode) != (*it).end() ||
                (*it).find(pThatNode) != (*it).end()) {
              (*it).insert(pThisNode);
              (*it).insert(pThatNode);
              found = true;
              break;
            }
          if (!found) {
            vclusters.push_back(set<Node*>());
            vclusters.back().insert(pThisNode);
            vclusters.back().insert(pThatNode);
          }
        }
      }
    }
    // Y+
    {
      const vector<PtrS<DEdge>>& vNeiEdges = gridElemMeta.vDualEdgesYp;
      for (int i = 0; i < (int)vNeiEdges.size(); ++i) {
        int vthisSide[4] = {3, 2, 7, 6};
        int vthatSide[4] = {0, 1, 4, 5};
        Poly* pThatElem = vNeiEdges[i]->elem1;
        for (int j = 0; j < 4; ++j) {
          Node* pThisNode = itCur->second[vthisSide[j]];
          Node* pThatNode = NULL;
          if (mcandidates.find(pThatElem) == mcandidates.end())
            pThatNode = pThatElem->Nodes()[vthatSide[j]];
          else
            pThatNode = mcandidates[pThatElem][vthatSide[j]];
          assert((pThisNode->Traits().Vector3d(Tag::Tag_Position_X) -
                  pThatNode->Traits().Vector3d(Tag::Tag_Position_X))
                     .norm() < 1e-6);
          bool found = false;
          for (auto it = vclusters.begin(); it != vclusters.end(); ++it)
            if ((*it).find(pThisNode) != (*it).end() ||
                (*it).find(pThatNode) != (*it).end()) {
              (*it).insert(pThisNode);
              (*it).insert(pThatNode);
              found = true;
              break;
            }
          if (!found) {
            vclusters.push_back(set<Node*>());
            vclusters.back().insert(pThisNode);
            vclusters.back().insert(pThatNode);
          }
        }
      }
    }

    // Z-
    {
      const vector<PtrS<DEdge>>& vNeiEdges = gridElemMeta.vDualEdgesZm;
      for (int i = 0; i < (int)vNeiEdges.size(); ++i) {
        int vthisSide[4] = {0, 1, 2, 3};
        int vthatSide[4] = {4, 5, 6, 7};
        Poly* pThatElem = vNeiEdges[i]->elem0;
        for (int j = 0; j < 4; ++j) {
          Node* pThisNode = itCur->second[vthisSide[j]];
          Node* pThatNode = NULL;
          if (mcandidates.find(pThatElem) == mcandidates.end())
            pThatNode = pThatElem->Nodes()[vthatSide[j]];
          else
            pThatNode = mcandidates[pThatElem][vthatSide[j]];
          assert((pThisNode->Traits().Vector3d(Tag::Tag_Position_X) -
                  pThatNode->Traits().Vector3d(Tag::Tag_Position_X))
                     .norm() < 1e-6);
          bool found = false;
          for (auto it = vclusters.begin(); it != vclusters.end(); ++it)
            if ((*it).find(pThisNode) != (*it).end() ||
                (*it).find(pThatNode) != (*it).end()) {
              (*it).insert(pThisNode);
              (*it).insert(pThatNode);
              found = true;
              break;
            }
          if (!found) {
            vclusters.push_back(set<Node*>());
            vclusters.back().insert(pThisNode);
            vclusters.back().insert(pThatNode);
          }
        }
      }
    }
    // Z+
    {
      const vector<PtrS<DEdge>>& vNeiEdges = gridElemMeta.vDualEdgesZp;
      for (int i = 0; i < (int)vNeiEdges.size(); ++i) {
        int vthisSide[4] = {4, 5, 6, 7};
        int vthatSide[4] = {0, 1, 2, 3};
        Poly* pThatElem = vNeiEdges[i]->elem1;
        for (int j = 0; j < 4; ++j) {
          Node* pThisNode = itCur->second[vthisSide[j]];
          Node* pThatNode = NULL;
          if (mcandidates.find(pThatElem) == mcandidates.end())
            pThatNode = pThatElem->Nodes()[vthatSide[j]];
          else
            pThatNode = mcandidates[pThatElem][vthatSide[j]];
          assert((pThisNode->Traits().Vector3d(Tag::Tag_Position_X) -
                  pThatNode->Traits().Vector3d(Tag::Tag_Position_X))
                     .norm() < 1e-6);
          bool found = false;
          for (auto it = vclusters.begin(); it != vclusters.end(); ++it)
            if ((*it).find(pThisNode) != (*it).end() ||
                (*it).find(pThatNode) != (*it).end()) {
              (*it).insert(pThisNode);
              (*it).insert(pThatNode);
              found = true;
              break;
            }
          if (!found) {
            vclusters.push_back(set<Node*>());
            vclusters.back().insert(pThisNode);
            vclusters.back().insert(pThatNode);
          }
        }
      }
    }
  }

  // Select, for each cluster, one representing node

  vector<Node*> vnewNodes;
  vnewNodes.reserve(vclusters.size());

  vector<Node*> vRepresentatives(vclusters.size());

  for (int i = 0; i < (int)vclusters.size(); ++i) {
    Node* pRepresentative = NULL;
    for (auto itSet = vclusters[i].begin();
         itSet != vclusters[i].end() && pRepresentative == NULL; ++itSet)
      if ((*itSet)->ID() != -1)
        pRepresentative = (*itSet);

    if (pRepresentative == NULL) {
      pRepresentative = (*vclusters[i].begin());
      pRepresentative->ID() = (int)m_vnodes.size();
      m_vnodes.push_back(pRepresentative);
      vnewNodes.push_back(pRepresentative);
    }

    vRepresentatives[i] = pRepresentative;
  }

  // Asign final nodes considering the representative node
  // of each cluster. Look for created nodes in the cluster
  // and assign the representative node to the element

  for (auto it = mcandidates.begin(); it != mcandidates.end(); ++it) {
    for (int i = 0; i < 8; ++i) {
      Node* pRepresentative = NULL;
      for (int j = 0; j < (int)vclusters.size() && pRepresentative == NULL; ++j)
        if (vclusters[j].find(it->second[i]) != vclusters[j].end())
          pRepresentative = vRepresentatives[j];

      if (pRepresentative != NULL)
        it->first->Nodes()[i] = pRepresentative;
      else {
        if (it->second[i]->ID() == -1) {
          it->second[i]->ID() = (int)m_vnodes.size();
          m_vnodes.push_back(it->second[i]);
          vnewNodes.push_back(it->second[i]);
          it->first->Nodes()[i] = it->second[i];
        }
      }
    }
  }

  // Delete not replaced nodes

  for (auto it = mcandidates.begin(); it != mcandidates.end(); ++it)
    for (int i = 0; i < 8; ++i)
      if (it->second[i]->ID() == -1)
        delete it->second[i];

  return vnewNodes;
}

Mesh_Grid::TopologyChanges Mesh_Grid::CutMesh(
    const list<PtrS<EdgeCut>>& lcutList) {
  // Remove dual edges affected by the cut

  list<DEdge*> ltoRemove;
  for (auto it = lcutList.begin(); it != lcutList.end(); ++it)
    ltoRemove.push_back((*it)->pEdge);  // Add to edge removal

  this->RemoveEdges(ltoRemove);

  // Store elements involved in the cut

  set<Poly*> sallElems;
  set<Poly*> scutElems;
  set<Node*> scutNodes;

  for (auto itCur = lcutList.begin(); itCur != lcutList.end(); itCur++) {
    sallElems.insert((*itCur)->vallElems.begin(), (*itCur)->vallElems.end());
    scutElems.insert((*itCur)->vcutElems.begin(), (*itCur)->vcutElems.end());
    scutNodes.insert((*itCur)->vcutNodes.begin(), (*itCur)->vcutNodes.end());
  }

  // Create map with potential new nodes for each of the elements potentially
  // involved in the cut. Some of this nodes will be replaced by neighbor nodes

  map<Poly*, vector<Node*>> mcandidates;

  for (auto itCur = scutElems.begin(); itCur != scutElems.end(); ++itCur) {
    // Only nodes being cut are potentially new, the rest of the nodes will
    // remain being the nodes that already belong to the affected elements

    vector<Node*> velemCandidates(8);
    for (int i = 0; i < 8; ++i) {
      if (scutNodes.find((*itCur)->Nodes()[i]) != scutNodes.end()) {
        Node* pOldNode = (*itCur)->Nodes()[i];
        Node* pNewNode = new Node(*pOldNode);
        velemCandidates[i] = pNewNode;
        velemCandidates[i]->ID() = -1;

        assert(pOldNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).surfaces ==
               pNewNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).surfaces);

        assert(pOldNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).fullIdx ==
               pNewNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).fullIdx);

        assert(pOldNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).vNeiElems ==
               pNewNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).vNeiElems);

        pNewNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).surfaces = -1;

        assert(pOldNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).surfaces !=
               pNewNode->Trait<GridNodeMeta>(Tag::Tag_GridMeta).surfaces);
      } else
        velemCandidates[i] = (*itCur)->Nodes()[i];
    }

    mcandidates[(Cell_Hexa*)(*itCur)] = velemCandidates;
  }

  vector<Node*> vnewNodes = this->ClusterNodes(mcandidates);

  // Update node neighbor elements

  set<Node*> snodeToUpdate;
  for (auto it = sallElems.begin(); it != sallElems.end(); ++it)
    snodeToUpdate.insert((*it)->Nodes().begin(), (*it)->Nodes().end());

  for (auto it = snodeToUpdate.begin(); it != snodeToUpdate.end(); ++it)
    assert((*it)->ID() != -1);

  // snodeToUpdate.insert(this->m_vnodes.begin(), this->m_vnodes.end());

  this->RecomputeNodeNeighborElements(snodeToUpdate);

  // Collect the topology changes

  TopologyChanges changes;
  changes.vnewNodes.resize(vnewNodes.size());
  for (int i = 0; i < (int)vnewNodes.size(); ++i)
    changes.vnewNodes[i] = vnewNodes[i]->ID();

  return changes;
}

void Mesh_Grid::RecomputeNodeNeighborElements(const set<Node*>& snodes) {
  set<Poly*> selems;
  for (auto it = snodes.begin(); it != snodes.end(); ++it) {
    GridNodeMeta& nodeMeta = (*it)->Trait<GridNodeMeta>(Tag::Tag_GridMeta);
    for (int i = 0; i < 8; ++i)
      if (nodeMeta.vNeiElems[i] != NULL) {
        selems.insert(nodeMeta.vNeiElems[i]);
        nodeMeta.vNeiElems[i] = NULL;
      }
    nodeMeta.surfaces = -1;
  }

  for (auto it = selems.begin(); it != selems.end(); ++it)
    for (int i = 0; i < 8; ++i)
      (*it)->Nodes()[i]->Trait<GridNodeMeta>(Tag::Tag_GridMeta).vNeiElems[i] =
          (*it);

  for (auto it = snodes.begin(); it != snodes.end(); ++it) {
    GridNodeMeta& nodeMeta = (*it)->Trait<GridNodeMeta>(Tag::Tag_GridMeta);
    nodeMeta.surfaces = 0;
    for (int j = 0; j < 8; ++j)
      if (nodeMeta.vNeiElems[j] == NULL)
        nodeMeta.surfaces++;
      else {
        GridElemMeta& elemMeta =
            nodeMeta.vNeiElems[j]->Trait<GridElemMeta>(Tag::Tag_GridMeta);
        switch (j) {
          case 0:
            if (elemMeta.vDualEdgesXm.empty() ||
                elemMeta.vDualEdgesYm.empty() || elemMeta.vDualEdgesZm.empty())
              nodeMeta.surfaces++;
            break;
          case 1:
            if (elemMeta.vDualEdgesXp.empty() ||
                elemMeta.vDualEdgesYm.empty() || elemMeta.vDualEdgesZm.empty())
              nodeMeta.surfaces++;
            break;
          case 2:
            if (elemMeta.vDualEdgesXp.empty() ||
                elemMeta.vDualEdgesYp.empty() || elemMeta.vDualEdgesZm.empty())
              nodeMeta.surfaces++;
            break;
          case 3:
            if (elemMeta.vDualEdgesXm.empty() ||
                elemMeta.vDualEdgesYp.empty() || elemMeta.vDualEdgesZm.empty())
              nodeMeta.surfaces++;
            break;
          case 4:
            if (elemMeta.vDualEdgesXm.empty() ||
                elemMeta.vDualEdgesYm.empty() || elemMeta.vDualEdgesZp.empty())
              nodeMeta.surfaces++;
            break;
          case 5:
            if (elemMeta.vDualEdgesXp.empty() ||
                elemMeta.vDualEdgesYm.empty() || elemMeta.vDualEdgesZp.empty())
              nodeMeta.surfaces++;
            break;
          case 6:
            if (elemMeta.vDualEdgesXp.empty() ||
                elemMeta.vDualEdgesYp.empty() || elemMeta.vDualEdgesZp.empty())
              nodeMeta.surfaces++;
            break;
          case 7:
            if (elemMeta.vDualEdgesXm.empty() ||
                elemMeta.vDualEdgesYp.empty() || elemMeta.vDualEdgesZp.empty())
              nodeMeta.surfaces++;
            break;
        }
      }
  }
}

Mesh_Grid::TopologyChanges Mesh_Grid::CutMesh_Rect(Real length,
                                                   Real height,
                                                   const Matrix3d& mR,
                                                   const Vector3d& vt,
                                                   Tag vs) {
  // Compute potential cuts

  list<PtrS<EdgeCut>> lcuts;
  this->CollectCuts_Rect(length, height, mR, vt, vs, lcuts);

  return CutMesh(lcuts);
}

void Mesh_Grid::CollectCuts_Rect(Real length,
                                 Real height,
                                 const Matrix3d& mR,
                                 const Vector3d& vt,
                                 Tag vs,
                                 list<PtrS<EdgeCut>>& cutList) {
  for (auto itCur = this->m_mdualEdges.begin();
       itCur != this->m_mdualEdges.end(); itCur++) {
    PtrS<EdgeCut> pCut = this->CheckDualEdge_Rect((*itCur).second.get(), length,
                                                  height, mR, vt, vs);
    if (pCut != NULL)
      cutList.push_back(pCut);
  }
}

PtrS<Mesh_Grid::EdgeCut> Mesh_Grid::CheckDualEdge_Rect(DEdge* pDualEdge,
                                                       Real length,
                                                       Real height,
                                                       const Matrix3d& mR,
                                                       const Vector3d& vt,
                                                       Tag s) {
  Vector3d x0 = pDualEdge->elem0->Centroid(s);
  Vector3d x1 = pDualEdge->elem1->Centroid(s);
  x0 = mR.transpose() * (x0 - vt);
  x1 = mR.transpose() * (x1 - vt);
  if (x0.x() >= -length / 2 && x0.x() <= length / 2 && x0.y() >= -height / 2 &&
      x0.y() <= height / 2 && x1.x() >= -length / 2 && x1.x() <= length / 2 &&
      x1.y() >= -height / 2 && x1.y() <= height / 2 && x0.z() * x1.z() <= 0) {
    PtrS<EdgeCut> pCut(new EdgeCut());
    pCut->side0 = (int)(x0.z() / abs(x0.z()));
    pCut->side1 = (int)(x1.z() / abs(x1.z()));
    pCut->pEdge = pDualEdge;

    this->CollectCutAffectedNodesAndElems(mR, vt, s, pCut);

    return pCut;
  }
  return PtrS<EdgeCut>();
}

Mesh_Grid::TopologyChanges Mesh_Grid::CutMesh_Disc(Real radius,
                                                   const Matrix3d& mR,
                                                   const Vector3d& vt,
                                                   Tag vs) {
  // Compute potential cuts

  list<PtrS<EdgeCut>> lcuts;
  this->CollectCuts_Disc(radius, mR, vt, vs, lcuts);

  return CutMesh(lcuts);
}

void Mesh_Grid::CollectCuts_Disc(Real radius,
                                 const Matrix3d& mR,
                                 const Vector3d& vt,
                                 Tag vs,
                                 list<PtrS<EdgeCut>>& cutList) {
  for (auto itCur = this->m_mdualEdges.begin();
       itCur != this->m_mdualEdges.end(); itCur++) {
    PtrS<EdgeCut> pCut =
        this->CheckDualEdge_Disc((*itCur).second.get(), radius, mR, vt, vs);
    if (pCut != NULL)
      cutList.push_back(pCut);
  }
}

PtrS<Mesh_Grid::EdgeCut> Mesh_Grid::CheckDualEdge_Disc(DEdge* pDualEdge,
                                                       Real radius,
                                                       const Matrix3d& mR,
                                                       const Vector3d& vt,
                                                       Tag s) {
  // Not implemented yet
  Vector3d c0 = pDualEdge->elem0->Centroid(s);
  Vector3d c1 = pDualEdge->elem1->Centroid(s);
  c0 = mR.transpose() * (c0 - vt);
  c1 = mR.transpose() * (c1 - vt);
  if (c0.x() * c0.x() <= radius && c0.y() * c0.y() <= radius &&
      c1.x() * c1.x() <= radius && c1.y() * c1.y() <= radius &&
      ((c0.z() >= 0 && c1.z() < 0) || (c0.z() >= 0 && c1.z() < 0))) {
    PtrS<EdgeCut> pCut(new EdgeCut());
    pCut->side0 = (int)(c0.z() / abs(c0.z()));
    pCut->side1 = (int)(c1.z() / abs(c1.z()));
    pCut->pEdge = pDualEdge;

    this->CollectCutAffectedNodesAndElems(mR, vt, s, pCut);

    return pCut;
  }
  return PtrS<EdgeCut>();
}

void Mesh_Grid::CollectCutAffectedNodesAndElems(const Matrix3d& mR,
                                                const Vector3d& vt,
                                                Tag s,
                                                PtrS<EdgeCut> pCut) {
  // Collected affected nodes

  if (pCut->pEdge->Dir == 0) {
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[1]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[2]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[6]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[5]);
  }
  if (pCut->pEdge->Dir == 1) {
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[2]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[3]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[7]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[6]);
  }
  if (pCut->pEdge->Dir == 2) {
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[4]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[5]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[6]);
    pCut->vcutNodes.push_back(pCut->pEdge->elem0->Nodes()[7]);
  }

  // Collect all elements

  set<Poly*> selems;

  for (auto i = pCut->vcutNodes.begin(); i != pCut->vcutNodes.end(); ++i) {
    const GridNodeMeta& gridNodeMeta = (*i)->Trait<GridNodeMeta>(Tag::Tag_GridMeta);
    selems.insert(gridNodeMeta.vNeiElems.begin(), gridNodeMeta.vNeiElems.end());
  }

  if (selems.find(NULL) != selems.end())
    selems.erase(NULL);  // Not neighbor

  pCut->vallElems.insert(pCut->vallElems.begin(), selems.begin(), selems.end());

  // Filter cut elements

  vector<Poly*> vfiltered;
  vfiltered.reserve(pCut->vallElems.size());
  for (auto i = pCut->vallElems.begin(); i != pCut->vallElems.end(); i++) {
    Vector3d c = (*i)->Centroid(s);
    c = mR.transpose() * (c - vt);
    if (c.z() < 0)
      vfiltered.push_back(*i);
  }
  pCut->vcutElems = vfiltered;
}

Mesh_Grid::~Mesh_Grid(void) {
  this->FreeMetadata();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Grid");
#endif
}

void Mesh_Grid::FreeMetadata() {
  this->m_mdualEdges.clear();
  this->DelNodesTrait(Tag::Tag_GridMeta);
  this->DelElemsTrait(Tag::Tag_GridMeta);
}

void Mesh_Grid::UpdateMetadata() {
  this->FreeMetadata();

  // Initialize fine mesh meta

  this->InitializeMetaData(
      m_vdims, this->m_vnodes, this->m_velems, this->m_vmapFull2Active_Elems,
      this->m_vmapActive2Full_Elems, this->m_vmapFull2Active_Nodes,
      this->m_vmapActive2Full_Nodes);
}

}  // namespace PhySim