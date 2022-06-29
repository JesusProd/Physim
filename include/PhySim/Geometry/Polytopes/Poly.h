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

#include <PhySim/Geometry/Geometry.h>

namespace PhySim {

using namespace std;
using namespace Eigen;

class IShapeFunction;

class Poly : public Geometry {
 protected:
  PtrS<IShapeFunction> m_pSFunction;

  Mesh* m_pMesh;

 public:
  Poly();
  Poly(const Poly& poly);
  Poly(int ID);
  virtual ~Poly();

 public:
  virtual int DimSpace() const override { return 3; }

  virtual inline void SetMesh(Mesh* m) { m_pMesh = m; }
  virtual inline Mesh* GetMesh() const { return m_pMesh; }

  virtual Real VolumeBasis(Tag s = Tag_Position_X) const override {
    return this->VolumeSpace(s);
  };
  virtual Real VolumeSpace(Tag s = Tag_Position_X) const override;
  virtual Vector3d Centroid(Tag s = Tag_Position_X) const override;
  virtual Matrix3d Rotation(Tag f, Tag t) const;
  virtual MatrixXd DeformationGradient(const VectorXd& vp, Tag f, Tag t) const;
  virtual void MassProperties(Tag s,
                              Real rho,
                              Real& mass,
                              Vector3d& vcom,
                              Matrix3d& mI) const;

  // Subelment

  /**
   * Set the positions of the subelement nodes according to the
   * particular discretization. For instance, 10-node quadratic
   * tetrahedron have 6 subelement nodes, one at each edge.
   *
   * @param s The space where positions are set.
   */
  virtual void InitSubelementPositions(Tag s);

  // Interpolation

  virtual PtrS<IShapeFunction> GetShapeFunction() const {
    return this->m_pSFunction;
  }
  virtual void SetShapeFunction(PtrS<IShapeFunction> pSF) {
    this->m_pSFunction = pSF;
  }

  /**
   * Interpolates the position of the nodes in the chosen space
   * at the specified point. The point must be in iso-parametric
   * coordinates.
   *
   * @param vp The point where to interpolate node position.
   * @param s The space in which node position is interpolated.
   */
  virtual VectorXd InterpolateValue(const VectorXd& vpoint, Tag s) const;

  virtual VectorXd InterpolateDeformation(const VectorXd& vpoint,
                                          Tag s0,
                                          Tag sx) const;

  // Parametric Space

  /**
   * Check if the specified iso-parametric coordinates are ok.
   *
   * @param vp The iso-parametric coordinates.
   */
  virtual bool IsValidParametric(const VectorXd& vp) const;

  /**
   * Computes the affine transformation to go from natural coordinates
   * in the specified space to coordinates of the iso-parametric polytope.
   * The transformation is provided in the form i = b + A*n.
   *
   * @param vb The independent vector of the affine transformation.
   * @param mA The corresponding matrix of the affine transformation.
   * @param s The space from which the coordinates are transformed.
   */
  virtual void TransformNat2Iso(VectorXd& vb, MatrixXd& mA, Tag s) const;

  /**
   * Computes the affine transformation to go from coordinates
   * of the iso-parametric polytope to the natural coordinates in
   * the specified space. The transformation is provided in the
   * form n = b + A*i.
   *
   * @param vb The independent vector of the affine transformation.
   * @param mA The corresponding matrix of the affine transformation.
   * @param s The space to which the coordinates are transformed.
   */
  virtual void TransformIso2Nat(VectorXd& vb, MatrixXd& mA, Tag s) const;

  // Embedding

  /**
   * Compute the embedded node corresponding to the specified
   * 3D point. The point must be in natural coordinates in the
   * specified space.
   *
   * @param vp Coordinates of the point in natural coordinates.
   * @param s The space in which node position is interpolated.
   */
  virtual PtrS<Embedding> ComputeEmbedding(const Vector3d& vx,
                                           Tag s = Tag_Position_X) override;

  /**
   * Compute the projected node corresponding to the specified
   * 3D point. The point must be in natural coordinates in the
   * specified space. The difference between this and the
   * embedding is that if the point is not embedded in the
   * polytope, the projection corresponds to the closest node.
   *
   * @param vp Coordinates of the point in natural coordinates.
   * @param s The space in which node position is interpolated.
   */
  virtual PtrS<Embedding> ComputeProjection(const Vector3d& vx,
                                            Tag s = Tag_Position_X) override;
};

}  // namespace PhySim