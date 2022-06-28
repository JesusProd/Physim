//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Interpolations.h>

#include <PhySim/Utils/MeshUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

void Interpolation::Interpolate(const MatrixXd& mV,
                                const MatrixXd& mP,
                                MatrixXd& mI) const {
  assert(this->NumPoints() == mV.rows());
  assert(this->DimBasis() == mP.cols());

  for (int i = 0; i < mP.rows(); ++i) {
    VectorXd vW;
    this->Compute_Weights(mP.row(i), vW);
    assert(vW.size() == this->NumPoints());

    mI.row(i).setZero();
    for (int j = 0; j < NumPoints(); ++j)
      mI.row(i) += vW(j) * mV.row(j);
  }
}

Interpolation_Chen::Interpolation_Chen(const vector<vector<Matrix3d>>& vmNij,
                                       int subdiv) {
  // Store weights
  m_vmNij = vmNij;

  // Deformation spaces

  assert(subdiv >= 1);
  this->m_subdiv = 1;
  this->m_fineElemsPerDime = (int)pow(2, subdiv);
  this->m_fineNodesPerDime = m_fineElemsPerDime + 1;
  this->m_fineNodesPerElem = (int)pow(m_fineNodesPerDime, 3);
  this->m_fineElemsPerElem = (int)pow(m_fineElemsPerDime, 3);

  ostringstream str;

  // Creates the nodes of the 3D grid of NxNxN
  // vertices where N is the number of fine nodes
  // per dimension of the coarse element. The size
  // is (2,2,2) and centered in the origin

  MeshUtils::create3DGridNodes(
      this->m_fineNodesPerDime, this->m_fineNodesPerDime,
      this->m_fineNodesPerDime, Vector3d(2, 2, 2), this->m_mVFine);

  this->m_mVFine.rowwise() -= Vector3d(1, 1, 1).transpose();

  // Creates the indices of the 3D grid of NxNxN
  // vertices where N is the number of fine nodes
  // per dimension

  MeshUtils::create3DGridIndex_Hex8(this->m_fineNodesPerDime,
                                    this->m_fineNodesPerDime,
                                    this->m_fineNodesPerDime, this->m_mHFine);

  // Creates the indices of the 3D grid of 2x2x2
  // vertices, that is, a single hexahedron. This
  // is to be used to map hexahedron nodes to the
  // sorted list of coarse nodes

  MeshUtils::create3DGridIndex_Hex8(2, 2, 2, this->m_vHCoar);
};

void Interpolation_Chen::Interpolate(const MatrixXd& mV,
                                     const MatrixXd& mP,
                                     MatrixXd& mI) const {
  assert(mV.rows() == this->NumPoints());
  assert(mP.cols() == this->DimBasis());

  mI.resize(mP.rows(), mV.cols());

  for (int k = 0; k < mP.rows(); ++k) {
    // Compute parametric coordinates and
    // interpolation weights within fine

    VectorXd vW;
    const VectorXd& vp = mP.row(k);
    int s = ChooseSubelement(vp);
    const VectorXd& x0 = m_mVFine.row(m_mHFine(s, 0));
    const VectorXd& x1 = m_mVFine.row(m_mHFine(s, 1));
    const VectorXd& x3 = m_mVFine.row(m_mHFine(s, 3));
    const VectorXd& x4 = m_mVFine.row(m_mHFine(s, 4));
    Vector3d vs(2 * (vp.x() - x0.x()) / (x1.x() - x0.x()) - 1,
                2 * (vp.y() - x0.y()) / (x3.y() - x0.y()) - 1,
                2 * (vp.z() - x0.z()) / (x4.z() - x0.z()) - 1);
    Interpolation_Trilinear::Instance()->Compute_Weights(vs, vW);

    mI.row(k).setZero();
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        mI.row(k) += m_vmNij[m_vHCoar(0, i)][m_mHFine(k, j)] * vW(j) *
                     mV.row(i).transpose();
  }
}

int Interpolation_Chen::ChooseSubelement(const VectorXd& vp) const {
  int n = 2;
  int x = (int)(std::floor((vp(0) + 1) * n / 2));
  int y = (int)(std::floor((vp(1) + 1) * n / 2));
  int z = (int)(std::floor((vp(2) + 1) * n / 2));
  return x * n * n + y * n + z;
}

}  // namespace PhySim