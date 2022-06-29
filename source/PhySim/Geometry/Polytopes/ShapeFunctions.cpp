//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

#include <PhySim/Utils/IOUtils.h>
#include <PhySim/Utils/MathUtils.h>
#include <PhySim/Utils/MeshUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

PtrS<ShapeFunction_Node> ShapeFunction_Node::PINSTANCE =
    PtrS<ShapeFunction_Node>(NULL);
PtrS<ShapeFunction_Tri3> ShapeFunction_Tri3::PINSTANCE =
    PtrS<ShapeFunction_Tri3>(NULL);
PtrS<ShapeFunction_Tri6> ShapeFunction_Tri6::PINSTANCE =
    PtrS<ShapeFunction_Tri6>(NULL);
PtrS<ShapeFunction_Tet4> ShapeFunction_Tet4::PINSTANCE =
    PtrS<ShapeFunction_Tet4>(NULL);
PtrS<ShapeFunction_Tet10> ShapeFunction_Tet10::PINSTANCE =
    PtrS<ShapeFunction_Tet10>(NULL);
PtrS<ShapeFunction_Quad4> ShapeFunction_Quad4::PINSTANCE =
    PtrS<ShapeFunction_Quad4>(NULL);
PtrS<ShapeFunction_Quad8> ShapeFunction_Quad8::PINSTANCE =
    PtrS<ShapeFunction_Quad8>(NULL);
PtrS<ShapeFunction_Hex8> ShapeFunction_Hex8::PINSTANCE =
    PtrS<ShapeFunction_Hex8>(NULL);
PtrS<ShapeFunction_Hex20> ShapeFunction_Hex20::PINSTANCE =
    PtrS<ShapeFunction_Hex20>(NULL);

void ShapeFunction::Compute_MatrixX(
    MatrixXd& mNX,
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  pData->m_pElemPoly->GetNodesTrait(mNX, pData->m_tagX);
  mNX.transposeInPlace();
}

void ShapeFunction::Compute_Matrix0(
    MatrixXd& mN0,
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  int dimSpace = pData->m_pElemPoly->DimSpace();
  int dimBasis = pData->m_pElemPoly->DimBasis();
  int numNodes = pData->m_pElemPoly->NumNodes();

  pData->m_pElemPoly->GetNodesTrait(mN0, pData->m_tag0);
  mN0.transposeInPlace();

  // If 2D element embedded in 3D space convert
  // the node coordinates to tangential space

  if (dimSpace != 2 && dimBasis == 2) {
    const Face* pFace = static_cast<const Face*>(pData->m_pElemPoly);

    MatrixXd mA;
    VectorXd vb;
    pFace->TransformNat2Bas(vb, mA, Tag::Tag_Position_0);

    mN0 = (mA * mN0).eval();
  }

  // If 1D element embedded in 3D space convert
  // the node coordinates to tangential space

  if (dimSpace != 1 && pData->m_pElemPoly->DimBasis() == 1) {
    const Edge* pEdge = static_cast<const Edge*>(pData->m_pElemPoly);

    MatrixXd mA;
    VectorXd vb;
    pEdge->TransformNat2Bas(vb, mA, Tag::Tag_Position_0);

    mN0 = (mA * mN0).eval();
  }
}

void ShapeFunction_LERP::InitDeformationAtSamples(
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  int dimSpace = pData->m_pElemPoly->DimSpace();
  int dimBasis = pData->m_pElemPoly->DimBasis();
  int numNodes = pData->m_pElemPoly->NumNodes();

  MatrixXd mN0;

  this->Compute_Matrix0(mN0, pData);

  int numQ = (int)pData->m_vpoints.size();

  pData->m_vmDFDx.resize(numQ);
  pData->m_vmJ.resize(numQ);
  pData->m_vmF.resize(numQ);

  for (int i = 0; i < numQ; ++i) {
    MatrixXd mBi;

    this->Compute_Partial(pData->m_vpoints[i], mBi);

    // UpdateMechanics Jacobian w.r.t. isoparametric

    pData->m_vmJ[i] = (mN0 * mBi);

    // Precompute BH0i = Bi * (N0 * Bi)^-1 to compute Fi = Nx * BH0i

    MathUtils::Tensor3<Real, 12, 3, 3> mDFDxTensor;
    MatrixXd mBH0i = mBi * pData->m_vmJ[i].inverse();

    for (int d = 0; d < 12; ++d) {
      Matrix<Real, 3, 4> mNx;
      mNx.setConstant(0);
      mNx.data()[d] = 1;
      mDFDxTensor[d] = mNx * mBH0i;
    }

    MathUtils::Vectorize<Real, 12, 3, 3>(mDFDxTensor, pData->m_vmDFDx[i]);
  }

  pData->m_isInit = true;
}

void ShapeFunction_LERP::UpdateDeformationAtSamples(
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  MatrixXd mNx;
  this->Compute_MatrixX(mNx, pData);
  VectorXd vNx;
  MathUtils::Vectorize<Real, 3, 4>(mNx, vNx);

  int numQ = (int)pData->m_vpoints.size();

  pData->m_vmF.resize(numQ);

  for (int i = 0; i < numQ; ++i) {
    MatrixXd mBi;
    this->Compute_Partial(pData->m_vpoints[i], mBi);

    Matrix3r mF;
    VectorXr vF = pData->m_vmDFDx[i] * vNx;
    MathUtils::Unvectorize<Real, 3, 3>(vF, mF);
    pData->m_vmF[i] = mF;
  }
}

void ShapeFunction_LERP::InterpolateDeformation(const Poly* pPoly,
                                                Tag trait0,
                                                Tag traitX,
                                                const MatrixXd& mP,
                                                MatrixXd& mI) const {
  MatrixXd mNTrait;
  pPoly->GetNodesTrait(mNTrait, traitX);
  InterpolateValue(mNTrait, mP, mI);
}

void ShapeFunction_LERP::InterpolateDeformation(const MatrixXd& mN0,
                                                const MatrixXd& mNx,
                                                const MatrixXd& mP,
                                                MatrixXd& mI) const {
  InterpolateValue(mNx, mP, mI);
}

void ShapeFunction_LERP::InterpolateValue(const Poly* pPoly,
                                          Tag trait,
                                          const MatrixXd& mP,
                                          MatrixXd& mI) const {
  MatrixXd mNTrait;
  pPoly->GetNodesTrait(mNTrait, trait);
  InterpolateValue(mNTrait, mP, mI);
}

void ShapeFunction_LERP::InterpolateValue(const MatrixXd& mN,
                                          const MatrixXd& mP,
                                          MatrixXd& mI) const {
  mI.resize(mP.rows(), mN.cols());

  for (int i = 0; i < mP.rows(); ++i) {
    mI.row(i).setZero();

    VectorXd vW;
    this->Compute_Values(mP.row(i), vW);
    for (int j = 0; j < NumPoints(); ++j)
      mI.row(i) += vW(j) * mN.row(j);
  }
}

ShapeFunction_Chen::ShapeFunction_Chen(const vector<vector<Matrix3d>>& vmNij,
                                       const VectorXi& vCoarsening) {
  this->m_vCoarsening = vCoarsening;

  // Store weights
  m_vmNij = vmNij;

  this->m_fineNodesPerElem =
      (vCoarsening.x() + 1) * (vCoarsening.y() + 1) * (vCoarsening.z() + 1);
  this->m_fineElemsPerElem =
      (vCoarsening.x() + 0) * (vCoarsening.y() + 0) * (vCoarsening.z() + 0);

  ostringstream str;

  // Creates the nodes of the 3D grid of NxNxN
  // vertices where N is the number of fine nodes
  // per dimension of the coarse element. The size
  // is (2,2,2) and centered in the origin

  MeshUtils::create3DGridNodes(
      this->m_vCoarsening.x() + 1, this->m_vCoarsening.y() + 1,
      this->m_vCoarsening.z() + 1, Vector3d(2, 2, 2), this->m_mVFine);

  this->m_mVFine.rowwise() -= Vector3d(1, 1, 1).transpose();

  // Creates the indices of the 3D grid of NxNxN
  // vertices where N is the number of fine nodes
  // per dimension

  MeshUtils::create3DGridIndex_Hex8(
      this->m_vCoarsening.x() + 1, this->m_vCoarsening.y() + 1,
      this->m_vCoarsening.z() + 1, this->m_mHFine);

  // Creates the indices of the 3D grid of 2x2x2
  // vertices, that is, a single hexahedron. This
  // is to be used to map hexahedron nodes to the
  // sorted list of coarse nodes

  MeshUtils::create3DGridIndex_Hex8(2, 2, 2, this->m_vHCoar);
};

void ShapeFunction_Chen::InitDeformationAtSamples(
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  MatrixXd mN0;
  this->Compute_Matrix0(mN0, pData);
  pData->m_vSizes.x() = mN0.col(1).x() - mN0.col(0).x();
  pData->m_vSizes.y() = mN0.col(3).y() - mN0.col(0).y();
  pData->m_vSizes.z() = mN0.col(4).z() - mN0.col(0).z();

  // Compute subelements for each quadrature
  int numQuad = (int)pData->m_vpoints.size();

  pData->m_vSubEle.resize(numQuad);
  for (int i = 0; i < numQuad; ++i)
    pData->m_vSubEle(i) = this->ChooseSubelement(pData->m_vpoints[i]);

  // Prepare sample for computing the element rotation

  pData->m_pRotationSamples->m_pElemPoly = pData->m_pElemPoly;
  pData->m_pRotationSamples->m_tag0 = pData->m_tag0;
  pData->m_pRotationSamples->m_tagX = pData->m_tagX;
  pData->m_pRotationSamples->m_vpoints.push_back(Vector3d(0.0, 0.0, 0.0));
  ShapeFunction_Hex8::InitDeformationAtSamples(pData->m_pRotationSamples);
  pData->m_mR = Matrix3d::Identity();

  Matrix3d mJ = Matrix3d::Identity();
  mJ(0, 0) = pData->m_vSizes.x() / 2;
  mJ(1, 1) = pData->m_vSizes.y() / 2;
  mJ(2, 2) = pData->m_vSizes.z() / 2;

  // Computing vDNDX and DXDp at each of the quadrature points

  pData->m_vmJ.resize(numQuad);
  pData->m_vDNDX.resize(numQuad);
  for (int k = 0; k < numQuad; ++k) {
    const VectorXd& vp = pData->m_vpoints[k];

    // Compute parametric fine element

    MatrixXd mVFineHexa(3, 8);
    for (int i = 0; i < 8; ++i)
      mVFineHexa.col(i) =
          m_mVFine.row(m_mHFine(pData->m_vSubEle(k), i)).transpose();

    // Compute parametric within fine

    Vector3d vs(2 * (vp.x() - mVFineHexa.col(0).x()) /
                        (mVFineHexa.col(1).x() - mVFineHexa.col(0).x()) -
                    1,
                2 * (vp.y() - mVFineHexa.col(0).y()) /
                        (mVFineHexa.col(3).y() - mVFineHexa.col(0).y()) -
                    1,
                2 * (vp.z() - mVFineHexa.col(0).z()) /
                        (mVFineHexa.col(4).z() - mVFineHexa.col(0).z()) -
                    1);

    assert(vs(0) >= -1 && vs(0) <= 1);
    assert(vs(1) >= -1 && vs(1) <= 1);
    assert(vs(2) >= -1 && vs(2) <= 1);

    // Compute DNDX of the fine hexahedron

    MatrixXd mDNDX;
    ShapeFunction_Hex8::Instance()->Compute_Partial(vs, mDNDX);
    mDNDX.col(0) *= 2 * this->m_vCoarsening.x() / pData->m_vSizes.x();
    mDNDX.col(1) *= 2 * this->m_vCoarsening.y() / pData->m_vSizes.y();
    mDNDX.col(2) *= 2 * this->m_vCoarsening.z() / pData->m_vSizes.z();
    pData->m_vDNDX[k].resize(8);
    for (int i = 0; i < 8; ++i)
      pData->m_vDNDX[k][i] = mDNDX.row(i);

    // Set the Jacobian DXDp at coarse

    pData->m_vmJ[k] = mJ;
  }

  pData->m_isInit = true;
}

void ShapeFunction_Chen::UpdateKinematicsAtSamples(
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  // Compute element rotation

  ShapeFunction_Hex8::Instance()->UpdateDeformationAtSamples(
      pData->m_pRotationSamples);

  Matrix3d mF = pData->m_pRotationSamples->Get_F()[0];
  JacobiSVD<Matrix3d> SVD(mF, ComputeFullU | ComputeFullV);
  MatrixXd mR = SVD.matrixU() * SVD.matrixV().transpose();

  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[TRACE] Updated Chen's rotation. norm(R0 - Rnew) = %f",
                    (mR - pData->m_mR).norm());

  pData->m_mR = mR;
}

void ShapeFunction_Chen::UpdateDeformationAtSamples(
    PtrS<IShapeFunction::IDeformationData> pDataParent) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  // Compute displacements matrices

  MatrixXd mN0;
  MatrixXd mNx;
  this->Compute_Matrix0(mN0, pData);
  this->Compute_MatrixX(mNx, pData);

  // Compute the unrotated displacements

  pData->m_mU = pData->m_mR.transpose() * mNx - mN0;

  // Compute DxD0 and DFDx at each quadrature point

  int numQuadPoints = (int)pData->m_vpoints.size();
  pData->m_vmF.resize(numQuadPoints);
  pData->m_vmDFDx.resize(numQuadPoints);
  for (int ig = 0; ig < numQuadPoints; ++ig) {
    this->Compute_F(pData->m_vmF[ig], pData, ig);
    this->Compute_DFDx(pData->m_vmDFDx[ig], pData, ig);
  }
}

void ShapeFunction_Chen::Compute_F(
    MatrixXd& mF,
    PtrS<IShapeFunction::IDeformationData> pDataParent,
    int k) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  MatrixXd mNx;
  MatrixXd mN0;
  this->Compute_MatrixX(mNx, pData);
  this->Compute_Matrix0(mN0, pData);

  int eleIdx = pData->m_vSubEle[k];
  const Matrix3d& mR = pData->m_mR;
  const vector<Vector3d>& vDNDX = pData->m_vDNDX[k];
  const vector<vector<Matrix3d>>& vmNij = m_vmNij;

  mF = mR;
  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++)
      mF += pData->m_mR *
            (vmNij[m_vHCoar(0, i)][m_mHFine(eleIdx, j)] * pData->m_mU.col(i)) *
            vDNDX[j].transpose();
}

void ShapeFunction_Chen::Compute_DFDx(
    MatrixXd& mDFDx,
    PtrS<IShapeFunction::IDeformationData> pDataParent,
    int k) const {
  PtrS<DeformationData> pData =
      dynamic_pointer_cast<DeformationData>(pDataParent);

  mDFDx.resize(9, 24);
  mDFDx.setZero();

  int eleIdx = pData->m_vSubEle[k];
  const Matrix3d& mR = pData->m_mR;
  const vector<Vector3d>& vDNDX = pData->m_vDNDX[k];
  const vector<vector<Matrix3d>>& vmNij = m_vmNij;

  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++) {
      Matrix3d mA =
          mR * vmNij[m_vHCoar(0, i)][m_mHFine(eleIdx, j)] * mR.transpose();

      for (int r0 = 0; r0 < 3; ++r0)
        for (int r1 = 0; r1 < 3; ++r1) {
          Vector3d vtotal = mA.row(r0) * vDNDX[j](r1);
          mDFDx(3 * r0 + r1, 3 * i + 0) += vtotal(0);
          mDFDx(3 * r0 + r1, 3 * i + 1) += vtotal(1);
          mDFDx(3 * r0 + r1, 3 * i + 2) += vtotal(2);
        }
    }
}

int ShapeFunction_Chen::ChooseSubelement(const VectorXd& vp) const {
  int x = (int)(std::floor((vp(0) + 1) * this->m_vCoarsening.x() / 2));
  int y = (int)(std::floor((vp(1) + 1) * this->m_vCoarsening.y() / 2));
  int z = (int)(std::floor((vp(2) + 1) * this->m_vCoarsening.z() / 2));
  return x * this->m_vCoarsening.y() * this->m_vCoarsening.z() +
         y * this->m_vCoarsening.z() + z;
};

void ShapeFunction_Chen::InterpolateDeformation(const MatrixXd& mN0,
                                                const MatrixXd& mNx,
                                                const MatrixXd& mP,
                                                MatrixXd& mI) const {
  assert(mN0.rows() == this->NumPoints());
  assert(mNx.rows() == this->NumPoints());
  assert(mP.cols() == this->DimBasis());

  // Compute undeformed

  MatrixXd mX;
  ShapeFunction_Hex8::InterpolateValue(mN0, mP, mX);

  // Compute rotation

  MatrixXd mB;
  this->Compute_Partial(Vector3d(0, 0, 0), mB);
  Real sizeX = mN0.row(1).x() - mN0.row(0).x();
  Real sizeY = mN0.row(3).y() - mN0.row(0).y();
  Real sizeZ = mN0.row(4).z() - mN0.row(0).z();
  Matrix3d mF = mNx.transpose() * mB;
  mF.col(0) *= 2 / sizeX;
  mF.col(1) *= 2 / sizeY;
  mF.col(2) *= 2 / sizeZ;
  JacobiSVD<Matrix3d> SVD(mF, ComputeFullU | ComputeFullV);
  Matrix3d mR = SVD.matrixU() * SVD.matrixV().transpose();

  MatrixXd mU = (mR.transpose() * mNx.transpose() - mN0.transpose());

  mI.resize(mP.rows(), mNx.cols());

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
    this->Compute_Values(vs, vW);

    mI.row(k).setZero();
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        mI.row(k) += m_vmNij[m_vHCoar(0, i)][m_mHFine(k, j)] * vW(j) *
                     mU.row(i).transpose();

    mI.row(k) = mR * (mX.row(k) + mI.row(k)).transpose();
  }
}

void ShapeFunction_Chen::InterpolateValue(const MatrixXd& mN,
                                          const MatrixXd& mP,
                                          MatrixXd& mI) const {
  assert(mN.rows() == this->NumPoints());
  assert(mP.cols() == this->DimBasis());

  mI.resize(mP.rows(), mN.cols());

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
    this->Compute_Values(vs, vW);

    mI.row(k).setZero();
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        mI.row(k) += m_vmNij[m_vHCoar(0, i)][m_mHFine(k, j)] * vW(j) *
                     mN.row(i).transpose();
  }
}

}  // namespace PhySim