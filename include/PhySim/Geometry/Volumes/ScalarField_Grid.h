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
#include <PhySim/Geometry/Volumes/ScalarField.h>

namespace PhySim {
template <typename T>
class ScalarField_Grid : public ScalarField<T> {
 public:
  using Tensor3s = Tensor<T, 3>;

 public:
  ScalarField_Grid()
      : m_vOrigin(Vector3d::Zero()), m_vDelta(Vector3d::Ones()) {}

  ScalarField_Grid(const Vector3i& vShape)
      : m_vOrigin(Vector3d::Zero()), m_vDelta(Vector3d::Ones()) {
    Setup(vShape);
  }

  ScalarField_Grid(const Vector3i& vShape,
                   const Vector3d& vOrigin,
                   const Vector3d& vDelta) {
    Setup(vShape, vOrigin, vDelta);
  }

 public:
  void Setup(const Vector3i& vShape) {
    assert(vShape.minCoeff() >= 1 &&
           "Invalid dimensions for grid. The shape must be greater or equal "
           "than 1 in all dimensions!");

    m_tGrid.resize(vShape[0], vShape[1], vShape[2]);
  }

  void Setup(const Vector3i& vShape,
             const Vector3d& vOrigin,
             const Vector3d& vDelta) {
    assert(vShape.minCoeff() >= 1 &&
           "Invalid dimensions for grid. The shape must be greater or equal "
           "than 1 in all dimensions!");

    m_tGrid.resize(vShape[0], vShape[1], vShape[2]);
    m_vOrigin = vOrigin;
    m_vDelta = vDelta;
  }

 public:
  Vector3i Shape() const {
    return {int(m_tGrid.dimension(0)), int(m_tGrid.dimension(1)),
            int(m_tGrid.dimension(2))};
  }

  Vector3d Origin() const { return m_vOrigin; }

  Vector3d Delta() const { return m_vDelta; }

 public:
  Tensor3s& Grid() { return m_tGrid; }

  const Tensor3s& Grid() const { return m_tGrid; }

 public:
  Scalar Sample(const Vector3d& vPoint) const override {
    constexpr bool isFPointType = std::is_floating_point<Scalar>::value;

    // Round to nearest neighbor if data type is not real.
    if (!isFPointType) {
      return At(IndexOf(vPoint, true));
    }

    // Otherwise, perform trilinear interpolation over field data.
    else {
      Vector3d vboxMax =
          m_vOrigin + this->Shape().cast<Real>().cwiseProduct(m_vDelta);
      if (vPoint.x() < m_vOrigin.x() || vPoint.x() > vboxMax.x() ||
          vPoint.y() < m_vOrigin.y() || vPoint.y() > vboxMax.y() ||
          vPoint.z() < m_vOrigin.z() || vPoint.z() > vboxMax.z())
        return -1;

      const Vector3i vShape = Shape();
      const Vector3d vCoord = (vPoint - m_vOrigin).cwiseQuotient(m_vDelta) -
                              Vector3d::Constant(0.5);
      const Vector3i vMinIndex = ClampIndex(vCoord.cast<int>());
      const Vector3i vMaxIndex =
          ClampIndex(vCoord.cast<int>() + Vector3i::Ones());
      const Vector3s vUpWeights =
          vCoord.array().floor().matrix().cast<Scalar>();
      const Vector3s vLoWeights = Vector3s::Ones() - vUpWeights;
      const Scalar w000 = vLoWeights[0] * vLoWeights[1] * vLoWeights[2];
      const Scalar w100 = vUpWeights[0] * vLoWeights[1] * vLoWeights[2];
      const Scalar w010 = vLoWeights[0] * vUpWeights[1] * vLoWeights[2];
      const Scalar w110 = vUpWeights[0] * vUpWeights[1] * vLoWeights[2];
      const Scalar w001 = vLoWeights[0] * vLoWeights[1] * vUpWeights[2];
      const Scalar w101 = vUpWeights[0] * vLoWeights[1] * vUpWeights[2];
      const Scalar w011 = vLoWeights[0] * vUpWeights[1] * vUpWeights[2];
      const Scalar w111 = vUpWeights[0] * vUpWeights[1] * vUpWeights[2];

      const Scalar s000 = At({vMinIndex[0], vMinIndex[1], vMinIndex[2]});
      const Scalar s100 = At({vMaxIndex[0], vMinIndex[1], vMinIndex[2]});
      const Scalar s010 = At({vMinIndex[0], vMaxIndex[1], vMinIndex[2]});
      const Scalar s110 = At({vMaxIndex[0], vMaxIndex[1], vMinIndex[2]});
      const Scalar s001 = At({vMinIndex[0], vMinIndex[1], vMaxIndex[2]});
      const Scalar s101 = At({vMaxIndex[0], vMinIndex[1], vMaxIndex[2]});
      const Scalar s011 = At({vMinIndex[0], vMaxIndex[1], vMaxIndex[2]});
      const Scalar s111 = At({vMaxIndex[0], vMaxIndex[1], vMaxIndex[2]});

      return w000 * s000 + w100 * s100 + w010 * s010 + w110 * s110 +
             w001 * s001 + w101 * s101 + w011 * s011 + w111 * s111;
    }
  }

  Vector3s Gradient(const Vector3d& vPoint) const override {
    constexpr bool isNumericType = std::is_arithmetic<Scalar>::value;
    constexpr bool isFPointType = std::is_floating_point<Scalar>::value;
    Vector3s vGrad({}, {}, {});

    // If the underlying type is not numeric, we don't know how to compute the
    // gradient. Only act on types we know that will produce a meaningful
    // gradient.
    if (isNumericType) {
      const Vector3i vShape = Shape();
      const Vector3i vIndex = IndexOf(vPoint, true);
      const Scalar value = m_tGrid(vIndex[0], vIndex[1], vIndex[2]);

      for (int i = 0; i < 3; ++i) {
        // If shape along the i-th axis is 1, it's impossible to compute a
        // gradient.
        if (vShape[i] <= 1) {
          vGrad[i] = Scalar(0);
          continue;
        }

        // Compute perturbed indices for finite differences.
        Vector3i vFwdIndex = vIndex;
        vFwdIndex[i] += 1;
        Vector3i vBwdIndex = vIndex;
        vBwdIndex[i] -= 1;
        if (vFwdIndex[i] < 0)
          vFwdIndex[i] = 0;
        if (vBwdIndex[i] >= vShape[i])
          vBwdIndex[i] = vShape[i] - 1;

        // Obtain values for differentiation.
        const Scalar fwdValue =
            vFwdIndex[i] == vIndex[i] ? value : At(vFwdIndex);
        const Scalar bwdValue =
            vBwdIndex[i] == vIndex[i] ? value : At(vBwdIndex);

        // Compute differences.
        vGrad[i] = fwdValue - bwdValue;
        if (isFPointType)
          vGrad[i] /= Scalar(m_vDelta[i] * (vFwdIndex[i] - vBwdIndex[i]));
      }
    }

    return vGrad;
  }

 public:
  Scalar& At(const Vector3i& vIndex) {
    return m_tGrid(vIndex[0], vIndex[1], vIndex[2]);
  }

  const Scalar& At(const Vector3i& vIndex) const {
    return m_tGrid(vIndex[0], vIndex[1], vIndex[2]);
  }

  Scalar& operator[](const Vector3i& vIndex) { return At(vIndex); }

  const Scalar& operator[](const Vector3i& vIndex) const { return At(vIndex); }

  Vector3i IndexOf(const Vector3d& vPoint, bool clamp = false) const {
    const Vector3i vShape = Shape();
    Vector3i vIndex =
        ((vPoint - m_vOrigin).cwiseQuotient(m_vDelta) - Vector3d::Constant(0.5))
            .cast<int>();

    if (clamp == true)
      vIndex =
          vIndex.cwiseMax(Vector3i::Zero()).cwiseMin(vShape - Vector3i::Ones());

    return vIndex;
  }

  Vector3d PointOf(const Vector3i& vIndex) const {
    return (vIndex.cast<Real>() + Vector3d::Constant(0.5))
               .cwiseProduct(m_vDelta) +
           m_vOrigin;
  }

 private:
  Vector3i ClampIndex(const Vector3i& vIndex) const {
    const Vector3i vShape = Shape();
    return vIndex.cwiseMin(vShape - Vector3i::Ones())
        .cwiseMax(Vector3i::Zero());
  }

 public:
  Vector3d m_vOrigin;
  Vector3d m_vDelta;
  Tensor3s m_tGrid;
};
}  // namespace PhySim
