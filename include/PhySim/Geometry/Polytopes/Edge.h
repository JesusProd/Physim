//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC Madrid
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Polytopes/Poly.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Edge : public Poly {
 public:
  Edge(int ID, const vector<Node*>& vnodes);
  ~Edge(void);

  inline Node* GetTail(void) const { return m_vnodes[0]; }
  inline void SetOrigin(Node* node) { m_vnodes[0] = node; }

  inline Node* GetHead(void) const { return m_vnodes[1]; }
  inline void SetHead(Node* node) { m_vnodes[1] = node; }

  inline int DimBasis() const override { return 1; }

  virtual Real Length(Tag s) const { return this->VolumeSpace(s); }

  virtual Real VolumeSpace(Tag s) const;
  virtual Vector3d Vector(Tag s) const;
  virtual Vector3d Tangent(Tag s) const;

  virtual void TransformNat2Iso(VectorXd& b, MatrixXd& A, Tag s) const override;
  virtual void TransformIso2Nat(VectorXd& b, MatrixXd& A, Tag s) const override;

  /**
   * Computes the rigid transformation from natural coordinates
   * to the local tangential coordinates of this edge, assuming
   * the edge is planar.
   */
  virtual void TransformNat2Bas(VectorXd& b, MatrixXd& A, Tag s) const {
    throw exception("Not implemented");
  }

  /**
   * Computes the rigid transformation from the local tangential
   * coordinates of this edge, to the natural coordinates assuming
   * the edge is planar.
   */
  virtual void TransformBas2Nat(VectorXd& b, MatrixXd& A, Tag s) const {
    throw exception("Not implemented");
  }
};
}  // namespace PhySim
