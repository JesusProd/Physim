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

namespace PhySim {
using namespace std;
using namespace Eigen;

class Mesh_Face : public Mesh {
 public:
  struct HE_Node;
  struct HE_Edge;
  struct HE_Face;
  struct HE_Half;
  struct HE_Half {
    HE_Edge* m_pheEdge;
    HE_Node* m_pheNode;
    HE_Face* m_pheFace;
    HE_Half* m_pheNext;
    HE_Half* m_phePair;
    HE_Half() {
      this->m_pheEdge = NULL;
      this->m_pheNode = NULL;
      this->m_pheNext = NULL;
      this->m_phePair = NULL;
      this->m_pheFace = NULL;
    }
  };
  struct HE_Edge {
    Edge* m_pHandle;
    HE_Half* m_pheHalf0;
    HE_Half* m_pheHalf1;
    HE_Edge() {
      this->m_pHandle = NULL;
      this->m_pheHalf0 = NULL;
      this->m_pheHalf1 = NULL;
    }
  };
  struct HE_Node {
    Node* m_pHandle;
    HE_Half* m_pheHalf;
    HE_Node() {
      this->m_pHandle = NULL;
      this->m_pheHalf = NULL;
    }
  };
  struct HE_Face {
    Face* m_pHandle;
    HE_Half* m_pheHalf;
  };

 protected:
  vector<HE_Edge*> m_vheEdges;
  vector<HE_Node*> m_vheNodes;
  vector<HE_Face*> m_vheFaces;
  vector<HE_Half*> m_vheHalfs;

 public:
  Mesh_Face();
  Mesh_Face(const Mesh_Face& toCopy);
  Mesh_Face(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Tri3,
            const vector<Tag>& vnTraits = vector<Tag>());

  void Init(const MatrixXd& mV,
            const MatrixXi& mF,
            Discretization D = Discretization::Tri3,
            const vector<Tag>& vnTraits = vector<Tag>());

  virtual ~Mesh_Face(void);

  virtual int NumHENodes() const { return (int)this->m_vheNodes.size(); }
  virtual int NumHEEdges() const { return (int)this->m_vheEdges.size(); }
  virtual int NumHEFaces() const { return (int)this->m_vheFaces.size(); }
  virtual int NumHEHalfs() const { return (int)this->m_vheHalfs.size(); }
  virtual HE_Node* GetHENode(int i) { return this->m_vheNodes[i]; }
  virtual HE_Edge* GetHEEdge(int i) { return this->m_vheEdges[i]; }
  virtual HE_Face* GetHEFace(int i) { return this->m_vheFaces[i]; }
  virtual HE_Half* GetHEHalf(int i) { return this->m_vheHalfs[i]; }

  virtual Real ComputeArea(Tag s);
  virtual Real Perimeter(Tag s);

  virtual void GetEdgeHinge_Nodes(const Edge* pEdge, vector<Node*>& vhinge);

  virtual void GetNodeRing_Nodes(const Node* pNode,
                                 vector<Node*>& vring,
                                 int D = 1);
  virtual void GetNodeRing_Edges(const Node* pNode,
                                 vector<Edge*>& vring,
                                 int D = 1);
  virtual void GetNodeRing_Faces(const Node* pNode,
                                 vector<Face*>& vring,
                                 int D = 1);

  virtual void GetNodeRing_InputHE(const Node* pNode, vector<HE_Half*>&);
  virtual void GetNodeRing_OutputHE(const Node* pNode, vector<HE_Half*>&);

  virtual void ComputeFaceToNodeMap_Angle(MatrixSd& mM, Tag s);
  virtual void ComputeFaceToNodeMap_Area(MatrixSd& mM, Tag s);

  virtual Real ComputeDihedralAngle(const Edge* pEdge, Tag s);

  virtual Matrix2d ComputeShapeOperator(const Node* pNode, Tag s);
  virtual Matrix2d ComputeShapeOperator(const Edge* pEdge, Tag s);
  virtual Matrix2d ComputeShapeOperator(const Face* pFace, Tag s);

  virtual Matrix2d ComputePlanarStrain(const Face* pFace,
                                       Tag s0,
                                       Tag sx,
                                       const VectorXd& vp);

  virtual Matrix2d ComputeBendingStrain(const Node* pNode, Tag s0, Tag sx);
  virtual Matrix2d ComputeBendingStrain(const Edge* pEdge, Tag s0, Tag sx);
  virtual Matrix2d ComputeBendingStrain(const Face* pFace, Tag s0, Tag sx);

  virtual bool IsBoundary(const Node* pNode) override;
  virtual bool IsBoundary(const Edge* pEdge) override;
  virtual bool IsBoundary(const Face* pFace, bool node = true) override;

  virtual void GetBoundaryNodes(vector<Node*>& vbnode) override;
  virtual void GetBoundaryEdges(vector<Edge*>& vbedge) override;
  virtual void GetBoundaryFaces(vector<Face*>& vbface,
                                bool node = true) override;

  virtual void GetInternalNodes(vector<Node*>& vbnode) override;
  virtual void GetInternalEdges(vector<Edge*>& vbedge) override;
  virtual void GetInternalFaces(vector<Face*>& vbface,
                                bool node = true) override;

  virtual void GetBoundaryLoop(vector<Node*>& vbloop);

 private:
  void UpdateMetadata();
  void FreeMetadata();
};

}  // namespace PhySim
