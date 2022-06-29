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

#include <PhySim/Geometry/Meshes/Mesh.h>
#include <PhySim/Geometry/Polytopes/Edge.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Edge : public Mesh {
 public:
  struct Connection {
    Node* m_center;
    vector<Edge*> m_vedges;
  };

 protected:
  vector<Connection*> m_vconns;
  vector<vector<Edge*>> m_vnodeAdjaEdges;
  vector<vector<Edge*>> m_vedgeNextEdges;
  vector<vector<Edge*>> m_vedgePrevEdges;
  iVector m_vnode2connMap;

 public:
  Mesh_Edge();
  Mesh_Edge(const Mesh_Edge& toCopy);
  Mesh_Edge(const MatrixXd& mV,
            const MatrixXi& mE,
            Discretization D = Discretization_Edge2,
            const vector<Tag>& vnodeTraits = vector<Tag>());
  void Init(const MatrixXd& mV,
            const MatrixXi& mE,
            Discretization D = Discretization_Edge2,
            const vector<Tag>& vnodeTraits = vector<Tag>());
  virtual ~Mesh_Edge(void);

  static void Merge(PtrS<Mesh_Edge>& pMesh,
                    const vector<PtrS<Mesh_Edge>>& vmeshes,
                    Tag trait,
                    Real tol);

  virtual Mesh* Clone() const override { return new Mesh_Edge(*this); }

  virtual vector<Node*> NextNodes(Node* pn) const;
  virtual vector<Node*> PrevNodes(Node* pn) const;
  virtual vector<Edge*> NextEdges(Node* pn) const;
  virtual vector<Edge*> PrevEdges(Node* pn) const;

  virtual vector<Edge*> PrevEdges(Edge* pe) const;
  virtual vector<Edge*> NextEdges(Edge* pe) const;
  virtual vector<Node*> NextNodes(Edge* pe) const;
  virtual vector<Node*> PrevNodes(Edge* pe) const;

  virtual int NumConns() const { return (int)this->m_vconns.size(); }

  virtual const vector<Connection*>& Connections() const {
    return this->m_vconns;
  }

  virtual Edge* GetEdge(int i) { return static_cast<Edge*>(this->m_velems[i]); }

  virtual int GetNodeConnection(int nodeID) const {
    return this->m_vnode2connMap[nodeID];
  }
  virtual bool IsConnectionNode(int nodeID) const {
    return this->m_vnode2connMap[nodeID] != -1;
  }

  virtual Real VolumeSpace(Tag s) const override;

 private:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim
