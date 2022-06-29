//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/MeshUtils.h>

#include <PhySim/Geometry/Meshes/Mesh_Tri.h>
#include <PhySim/Physics/Elements/EnergyElement_DiscreteShells.h>
#include <PhySim/Physics/Simulables/Simulable_ThinShell.h>
#include <PhySim/Solvers/OptimProblem_BasicStatic.h>
#include <PhySim/Solvers/OptimSolver_USQP_LS.h>
#include <PhySim/Utils/Utils.h>

#include <igl/boundary_loop.h>
#include <igl/lscm.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace MeshUtils {
void create2DGridNodes(int N, int M, const Vector2d& sizes, MatrixXd& mV) {
  int countV = 0;
  int countH = 0;
  mV.resize(N * M, 2);

  // Create vertices

  for (int i = 0; i < N; ++i)
    for (int j = 0; j < M; ++j) {
      mV.row(countV++) = Vector2d(i * sizes(0), j * sizes(1));
    }
}

void create3DGridNodes(int N,
                       int M,
                       int O,
                       const Vector3d& sizes,
                       MatrixXd& mV) {
  int countV = 0;
  mV.resize(N * M * O, 3);

  Vector3d sizesElem =
      Vector3d(sizes(0) / (N - 1), sizes(1) / (M - 1), sizes(2) / (O - 1));

  // Create vertices

  for (int i = 0; i < N; ++i)
    for (int j = 0; j < M; ++j)
      for (int k = 0; k < O; ++k) {
        mV.row(countV++) =
            Vector3d(i * sizesElem(0), j * sizesElem(1), k * sizesElem(2));
      }
}

void create2DGridIndex_Tri3(int N, int M, MatrixXi& mF) {
  mF.resize((N - 1) * (M - 1) * 2, 3);
  int countF = 0;
  for (int i = 0; i < N - 1; ++i)
    for (int j = 0; j < M - 1; ++j) {
      mF.row(countF++) = Vector3i(i * M + j, i * M + j + 1,
                                  (i + 1) * M + j);  // Triangle 1
      mF.row(countF++) = Vector3i((i + 1) * M + j, i * M + j + 1,
                                  (i + 1) * M + j + 1);  // Triangle 2
    }
}

void create2DGridIndex_Quad4(int N, int M, MatrixXi& mF) {
  mF.resize((N - 1) * (M - 1), 4);
  int countF = 0;
  for (int i = 0; i < N - 1; ++i)
    for (int j = 0; j < M - 1; ++j) {
      mF.row(countF++) = Vector4i(i * N + j, i * N + j + 1, (i + 1) * N + j + 1,
                                  (i + 1) * N + j);
    }
}

void create3DGridIndex_Tri3(int N, int M, int O, MatrixXi& mT) {
  MatrixXi mH;

  create3DGridIndex_Hex8(N, M, O, mH);

  int countF = 0;
  int numFaces = 2 * 6 * mH.rows();
  mT.resize(numFaces, 3);
  for (int i = 0; i < mH.rows(); ++i) {
    // Side 0
    mT.row(countF++) = Vector3i(mH(i, 0), mH(i, 3), mH(i, 2));
    mT.row(countF++) = Vector3i(mH(i, 0), mH(i, 2), mH(i, 1));

    // Side 1
    mT.row(countF++) = Vector3i(mH(i, 1), mH(i, 2), mH(i, 6));
    mT.row(countF++) = Vector3i(mH(i, 1), mH(i, 6), mH(i, 5));

    // Side 2
    mT.row(countF++) = Vector3i(mH(i, 5), mH(i, 6), mH(i, 7));
    mT.row(countF++) = Vector3i(mH(i, 5), mH(i, 7), mH(i, 4));

    // Side 3
    mT.row(countF++) = Vector3i(mH(i, 4), mH(i, 7), mH(i, 3));
    mT.row(countF++) = Vector3i(mH(i, 4), mH(i, 3), mH(i, 0));

    // Top
    mT.row(countF++) = Vector3i(mH(i, 3), mH(i, 7), mH(i, 6));
    mT.row(countF++) = Vector3i(mH(i, 3), mH(i, 6), mH(i, 2));

    // Bottom
    mT.row(countF++) = Vector3i(mH(i, 4), mH(i, 0), mH(i, 1));
    mT.row(countF++) = Vector3i(mH(i, 4), mH(i, 1), mH(i, 5));
  }
}

void create3DGridIndex_Hex8(int N, int M, int O, MatrixXi& mH) {
  int slideSize = M * O;
  int lineSize = O;

  mH.resize((N - 1) * (M - 1) * (O - 1), 8);
  int countH = 0;
  for (int i = 0; i < N - 1; ++i)
    for (int j = 0; j < M - 1; ++j)
      for (int k = 0; k < O - 1; ++k) {
        int offset = i * slideSize + j * lineSize + k;

        mH(countH, 0) = offset;
        mH(countH, 1) = offset + slideSize;
        mH(countH, 2) = offset + slideSize + lineSize;
        mH(countH, 3) = offset + lineSize;
        mH(countH, 4) = offset + 1;
        mH(countH, 5) = offset + slideSize + 1;
        mH(countH, 6) = offset + slideSize + lineSize + 1;
        mH(countH, 7) = offset + lineSize + 1;
        countH++;
      }
}

void extract2DGridQuad4Surface(const MatrixXd& mV,
                               const MatrixXi& mH,
                               VectorXi& vmap,
                               MatrixXi& mS,
                               MatrixXd& mN) {}

void extract3DGridHex8Surface_Tri3(const MatrixXd& mV,
                                   const MatrixXi& mH,
                                   VectorXi& vmap,
                                   MatrixXi& mS,
                                   MatrixXd& mN) {
  MatrixXi mSquads;
  extract3DGridHex8Surface_Quad4(mV, mH, vmap, mSquads, mN);
  splitQuadMeshInTris(mSquads, mS);
}

void extract3DGridHex8Surface_Quad4(const MatrixXd& mV,
                                    const MatrixXi& mH,
                                    VectorXi& vmap,
                                    MatrixXi& mS,
                                    MatrixXd& mN) {
  VectorXi vdegree = VectorXi::Zero(mV.rows());

  for (int i = 0; i < mH.rows(); ++i) {
    for (int j = 0; j < 8; ++j)
      vdegree(mH(i, j))++;
  }

  // Select boundary

  int count = 0;
  iVector vboundaryToWhole;
  iVector vwholeToBoundary;
  vwholeToBoundary.resize(mV.rows(), -1);
  vboundaryToWhole.reserve(mV.rows());

  for (int i = 0; i < mV.rows(); ++i) {
    if (vdegree[i] != 8) {
      vwholeToBoundary[i] = count++;
      vboundaryToWhole.push_back(i);
    }
  }

  vector<VectorXi> vquads;
  vquads.reserve(6 * mH.rows());
  for (int i = 0; i < mH.rows(); ++i) {
    int i0 = mH(i, 0);
    int i1 = mH(i, 1);
    int i2 = mH(i, 2);
    int i3 = mH(i, 3);
    int i4 = mH(i, 4);
    int i5 = mH(i, 5);
    int i6 = mH(i, 6);
    int i7 = mH(i, 7);
    if (vwholeToBoundary[mH(i, 0)] != -1 && vwholeToBoundary[mH(i, 1)] != -1 &&
        vwholeToBoundary[mH(i, 2)] != -1 && vwholeToBoundary[mH(i, 3)] != -1)
      vquads.push_back(
          Vector4i(vwholeToBoundary[mH(i, 0)], vwholeToBoundary[mH(i, 1)],
                   vwholeToBoundary[mH(i, 2)], vwholeToBoundary[mH(i, 3)]));
    //
    if (vwholeToBoundary[mH(i, 1)] != -1 && vwholeToBoundary[mH(i, 5)] != -1 &&
        vwholeToBoundary[mH(i, 6)] != -1 && vwholeToBoundary[mH(i, 2)] != -1)
      vquads.push_back(
          Vector4i(vwholeToBoundary[mH(i, 1)], vwholeToBoundary[mH(i, 5)],
                   vwholeToBoundary[mH(i, 6)], vwholeToBoundary[mH(i, 2)]));
    //
    if (vwholeToBoundary[mH(i, 5)] != -1 && vwholeToBoundary[mH(i, 4)] != -1 &&
        vwholeToBoundary[mH(i, 7)] != -1 && vwholeToBoundary[mH(i, 6)] != -1)
      vquads.push_back(
          Vector4i(vwholeToBoundary[mH(i, 5)], vwholeToBoundary[mH(i, 4)],
                   vwholeToBoundary[mH(i, 7)], vwholeToBoundary[mH(i, 6)]));
    //
    if (vwholeToBoundary[mH(i, 4)] != -1 && vwholeToBoundary[mH(i, 0)] != -1 &&
        vwholeToBoundary[mH(i, 3)] != -1 && vwholeToBoundary[mH(i, 7)] != -1)
      vquads.push_back(
          Vector4i(vwholeToBoundary[mH(i, 4)], vwholeToBoundary[mH(i, 0)],
                   vwholeToBoundary[mH(i, 3)], vwholeToBoundary[mH(i, 7)]));
    //
    if (vwholeToBoundary[mH(i, 3)] != -1 && vwholeToBoundary[mH(i, 2)] != -1 &&
        vwholeToBoundary[mH(i, 6)] != -1 && vwholeToBoundary[mH(i, 7)] != -1)
      vquads.push_back(
          Vector4i(vwholeToBoundary[mH(i, 3)], vwholeToBoundary[mH(i, 2)],
                   vwholeToBoundary[mH(i, 6)], vwholeToBoundary[mH(i, 7)]));
    //
    if (vwholeToBoundary[mH(i, 4)] != -1 && vwholeToBoundary[mH(i, 5)] != -1 &&
        vwholeToBoundary[mH(i, 1)] != -1 && vwholeToBoundary[mH(i, 0)] != -1)
      vquads.push_back(
          Vector4i(vwholeToBoundary[mH(i, 4)], vwholeToBoundary[mH(i, 5)],
                   vwholeToBoundary[mH(i, 1)], vwholeToBoundary[mH(i, 0)]));
  }

  // Build vertex map

  vmap = Utils::toEigen(vboundaryToWhole);

  // Build faces/normal vector

  mS = MatrixXi::Zero(vquads.size(), 4);
  mN = MatrixXd::Zero(vmap.size(), 3);

  for (int i = 0; i < mS.rows(); ++i) {
    mS.row(i) = vquads[i];

    Vector3d ve0 = mV.row(vmap[vquads[i][1]]) - mV.row(vmap[vquads[i][0]]);
    Vector3d ve1 = mV.row(vmap[vquads[i][2]]) - mV.row(vmap[vquads[i][0]]);
    Vector3d vn = ve0.cross(ve1).normalized();
    for (int j = 0; j < 4; ++j)
      mN.row(vquads[i][j]) += vn;
  }

  mN.rowwise().normalize();
}

void extractFromHexaMesh_Tri(const MatrixXi& mH, MatrixXi& mT) {
  mT.resize(mH.rows() * 12, 3);

  int countTri = 0;

  for (int i = 0; i < mH.rows(); ++i) {
    // Right
    mT.row(countTri++) = Vector3i(mH(i, 1), mH(i, 2), mH(i, 6));
    mT.row(countTri++) = Vector3i(mH(i, 1), mH(i, 6), mH(i, 5));

    // Left
    mT.row(countTri++) = Vector3i(mH(i, 0), mH(i, 4), mH(i, 7));
    mT.row(countTri++) = Vector3i(mH(i, 0), mH(i, 7), mH(i, 3));

    // Top
    mT.row(countTri++) = Vector3i(mH(i, 3), mH(i, 6), mH(i, 2));
    mT.row(countTri++) = Vector3i(mH(i, 3), mH(i, 7), mH(i, 6));

    // Bottom
    mT.row(countTri++) = Vector3i(mH(i, 0), mH(i, 1), mH(i, 5));
    mT.row(countTri++) = Vector3i(mH(i, 0), mH(i, 5), mH(i, 4));

    // Front
    mT.row(countTri++) = Vector3i(mH(i, 5), mH(i, 6), mH(i, 7));
    mT.row(countTri++) = Vector3i(mH(i, 5), mH(i, 7), mH(i, 4));

    // Back
    mT.row(countTri++) = Vector3i(mH(i, 3), mH(i, 2), mH(i, 1));
    mT.row(countTri++) = Vector3i(mH(i, 3), mH(i, 1), mH(i, 0));
  }
}

void extractFromHexaMesh_Quad(const MatrixXi& mH, MatrixXi& mQ) {
  mQ.resize(mH.rows() * 6, 4);

  int countQuad = 0;

  for (int i = 0; i < mH.rows(); ++i) {
    // Right
    mQ.row(countQuad++) = Vector4i(mH(i, 1), mH(i, 2), mH(i, 6), mH(i, 5));

    // Left
    mQ.row(countQuad++) = Vector4i(mH(i, 0), mH(i, 4), mH(i, 7), mH(i, 4));

    // Top
    mQ.row(countQuad++) = Vector4i(mH(i, 3), mH(i, 7), mH(i, 6), mH(i, 2));

    // Bottom
    mQ.row(countQuad++) = Vector4i(mH(i, 0), mH(i, 1), mH(i, 5), mH(i, 4));

    // Front
    mQ.row(countQuad++) = Vector4i(mH(i, 5), mH(i, 6), mH(i, 7), mH(i, 4));

    // Back
    mQ.row(countQuad++) = Vector4i(mH(i, 3), mH(i, 2), mH(i, 1), mH(i, 0));
  }
}

void extractFromHexaMesh_Edge(const MatrixXi& mH, MatrixXi& mE) {
  mE.resize(mH.rows() * 12, 2);

  int countEdge = 0;

  for (int i = 0; i < mH.rows(); ++i) {
    // X
    mE.row(countEdge++) = Vector2i(mH(i, 6), mH(i, 2));
    mE.row(countEdge++) = Vector2i(mH(i, 5), mH(i, 1));
    mE.row(countEdge++) = Vector2i(mH(i, 4), mH(i, 0));
    mE.row(countEdge++) = Vector2i(mH(i, 7), mH(i, 3));

    // Y
    mE.row(countEdge++) = Vector2i(mH(i, 4), mH(i, 7));
    mE.row(countEdge++) = Vector2i(mH(i, 5), mH(i, 6));
    mE.row(countEdge++) = Vector2i(mH(i, 1), mH(i, 2));
    mE.row(countEdge++) = Vector2i(mH(i, 0), mH(i, 3));

    // Z
    mE.row(countEdge++) = Vector2i(mH(i, 3), mH(i, 2));
    mE.row(countEdge++) = Vector2i(mH(i, 7), mH(i, 6));
    mE.row(countEdge++) = Vector2i(mH(i, 4), mH(i, 5));
    mE.row(countEdge++) = Vector2i(mH(i, 0), mH(i, 1));
  }
}

void filterIsolatedNodes(const MatrixXd& mVin,
                         const MatrixXi& mEin,
                         MatrixXd& mVout,
                         MatrixXi& mEout,
                         VectorXi& vmap) {
  iVector vold2New(mVin.rows(), -1);

  mEout = mEin;

  // Build new element matrix to new indices

  int count = 0;

  for (int i = 0; i < mEin.rows(); ++i) {
    for (int j = 0; j < mEin.cols(); ++j) {
      if (vold2New[mEin(i, j)] == -1)
        vold2New[mEin(i, j)] = count++;

      mEout(i, j) = vold2New[mEin(i, j)];
    }
  }

  // Build new filtered node matrix and map

  vmap.resize(count);

  mVout.resize(count, mVin.cols());

  for (int i = 0; i < mVin.rows(); ++i) {
    if (vold2New[i] == -1)
      continue;  // Not

    vmap(vold2New[i]) = i;

    mVout.row(vold2New[i]) = mVin.row(i);
  }
}

void filterSelectedNodes(const MatrixXd& mVin,
                         const VectorXi& vsel,
                         MatrixXd& mVout) {
  mVout.resize(vsel.size(), mVin.cols());
  for (int i = 0; i < (int)vsel.size(); ++i)
    mVout.row(i) = mVin.row(vsel[i]);
}

void filterSelectedNodes(const MatrixXd& mVin,
                         const MatrixXi& mEin,
                         const VectorXi& vsel,
                         MatrixXd& mVout,
                         MatrixXi& mEout) {
  iVector vstencil(mVin.rows(), -1);
  mVout.resize(vsel.size(), mVin.cols());
  for (int i = 0; i < (int)vsel.size(); ++i) {
    mVout.row(i) = mVin.row(vsel[i]);
    vstencil[vsel[i]] = i;
  }

  vector<VectorXi> vEout;
  vEout.reserve(mEin.rows());
  for (int i = 0; i < mEin.rows(); ++i) {
    VectorXi newEle(mEin.cols());
    bool isActive = true;
    for (int j = 0; j < mEin.cols(); ++j) {
      if (vstencil[mEin(i, j)] == -1) {
        isActive = false;
        break;
      } else {
        newEle[j] = vstencil[mEin(i, j)];
      }
    }

    if (isActive)
      vEout.push_back(newEle);
  }

  mEout.resize(vEout.size(), mEin.cols());
  for (int i = 0; i < mEout.rows(); ++i)
    mEout.row(i) = vEout[i];
}

void splitQuadMeshInTris(const MatrixXi& mEin, MatrixXi& mEout) {
  assert(mEin.cols() == 4);

  mEout.resize(2 * mEin.rows(), 3);

  for (int i = 0; i < mEin.rows(); ++i) {
    // Tri 1
    mEout(2 * i + 0, 0) = mEin(i, 0);
    mEout(2 * i + 0, 1) = mEin(i, 1);
    mEout(2 * i + 0, 2) = mEin(i, 3);

    // Tri 2
    mEout(2 * i + 1, 0) = mEin(i, 3);
    mEout(2 * i + 1, 1) = mEin(i, 1);
    mEout(2 * i + 1, 2) = mEin(i, 2);
  }
}

void splitHexaMeshInTets(const MatrixXi& mEin, MatrixXi& mEout) {
  throw PhySim::exception("Not implemented");

  assert(mEin.cols() == 8);

  mEout.resize(5 * mEin.rows(), 4);

  for (int i = 0; i < mEin.rows(); ++i) {
    // TODO
  }
}

void combineMeshes(const vector<MatrixXd>& vmV,
                   const vector<MatrixXi>& vmF,
                   int M,
                   MatrixXd& mVout,
                   MatrixXi& mFout) {
  assert(vmV.size() == vmF.size());
  int offset = 0;
  vector<Vector3d> vvertices;
  vector<VectorXi> vindices;

  for (size_t i = 0; i < vmV.size(); ++i) {
    vvertices.reserve(vvertices.capacity() + vmV[i].rows());
    vindices.reserve(vindices.capacity() + vmF[i].rows());
    VectorXi voff = VectorXi::Ones(M) * offset;

    for (int j = 0; j < vmV[i].rows(); ++j)
      vvertices.push_back(vmV[i].row(j));

    for (int j = 0; j < vmF[i].rows(); ++j)
      vindices.push_back(vmF[i].row(j) + voff.transpose());

    offset += vmV[i].rows();
  }

  mVout.resize(vvertices.size(), 3);
  mFout.resize(vindices.size(), M);
  for (int i = 0; i < mVout.rows(); ++i)
    mVout.row(i) = vvertices[i];
  for (int i = 0; i < mFout.rows(); ++i)
    mFout.row(i) = vindices[i];
}

bool triangulate(const MatrixXd& mV,
                 const MatrixXi& mE,
                 const MatrixXd& mH,
                 MatrixXd& mVout,
                 MatrixXi& mFout,
                 const string& options) {
  // igl::triangle::triangulate(mV, mE, mH, options, mVout, mFout);
  return false;
}

bool tetrahedralize(const MatrixXd& mV,
                    const MatrixXi& mF,
                    const MatrixXd& mH,
                    MatrixXd& mVout,
                    MatrixXi& mFout,
                    MatrixXi& mTout,
                    const string& options) {
  // return igl::copyleft::tetgen::tetrahedralize(mV, mF, options,
  // mVout, mTout, mFout) == 0;
  return false;
}

bool create2DRectangle(Real sizeX,
                       Real sizeY,
                       int numeX,
                       int numeY,
                       MatrixXd& mVout,
                       MatrixXi& mFout,
                       const string& options) {
  int numVB = 2 * numeX + 2 * numeY - 4;

  vector<Vector2d> vboundary;
  vboundary.reserve(numVB);

  Real incX = sizeX / (numeX - 1);
  Real incY = sizeY / (numeY - 1);

  Real iniX = -sizeX / 2;
  Real iniY = -sizeY / 2;
  Real finX = +sizeX / 2;
  Real finY = +sizeY / 2;

  // Create boundary

  vboundary.push_back(Vector2d(iniX, iniY));

  // Side 0

  for (int i = 1; i < numeX; ++i)
    vboundary.push_back(Vector2d(iniX + incX * i, iniY));

  // Side 1

  for (int i = 1; i < numeY; ++i)
    vboundary.push_back(Vector2d(finX, iniY + incY * i));

  // Side 2

  for (int i = 1; i < numeX; ++i)
    vboundary.push_back(Vector2d(finX - i * incX, finY));

  // Side 3

  for (int i = 1; i < numeY; ++i)
    vboundary.push_back(Vector2d(iniX, finY - i * incY));

  // Prepare matrices

  MatrixXd mBV(numVB, 2);
  MatrixXi mBE(numVB, 2);

  for (int i = 0; i < numVB; ++i) {
    mBV(i, 0) = vboundary[i].x();
    mBV(i, 1) = vboundary[i].y();

    if (i != numVB - 1) {
      mBE(i, 0) = i;
      mBE(i, 1) = i + 1;
    } else {
      mBE(i, 0) = i;
      mBE(i, 1) = 0;
    }
  }

  // Return triangulation

  MatrixXd mVout2D;

  if (triangulate(mBV, mBE, MatrixXd(), mVout2D, mFout, options)) {
    mVout = MatrixXd::Zero(mVout2D.rows(), 3);
    mVout.block(0, 0, mVout2D.rows(), 1) = mVout2D.col(0);
    mVout.block(0, 2, mVout2D.rows(), 1) = mVout2D.col(1);

    return true;
  }

  return false;
}

bool create2DCircle(Real radius,
                    int numS,
                    MatrixXd& mV,
                    MatrixXi& mF,
                    const string& options) {
  // TODO
  return true;
}

void mergeDuplicatedNodes(const MatrixXd& mV,
                          const MatrixXi& mE,
                          MatrixXd& mVmerged,
                          MatrixXi& mEmerged,
                          VectorXi& vmaps,
                          double tol) {
  // Merge vertices

  vmaps = -1 * VectorXi::Ones(mV.rows());

  int count = 0;  // Merged

  vector<VectorXd> vmergedV;
  vector<VectorXi> vmergedE;

  for (int i = 0; i < (int)mV.size(); ++i) {
    for (int k = 0; k < (int)vmergedV.size(); ++k) {
      if ((mV.row(i) - vmergedV[k]).squaredNorm() <= tol) {
        vmaps[i] = k;
        break;  // Found!
      }
    }

    if (vmaps[i] == -1) {
      vmaps[i] = count++;

      // Add new merged point to vector
      vmergedV.push_back(mV.row(i));
    }
  }

  // Merge elements

  for (int i = 0; i < (int)mE.rows(); ++i) {
    VectorXi vei = mE.row(i);
    for (int j = 0; j < vei.size(); ++j)
      vei(j) = vmaps(vei(j));  // Map!

    vmergedE.push_back(vei);
  }

  // Collect matrices

  mEmerged.resize(vmergedE.size(), vmergedE[0].size());
  mVmerged.resize(vmergedV.size(), vmergedV[0].size());

  for (int i = 0; i < mVmerged.rows(); ++i)
    mVmerged.row(i) = vmergedV[i];
}

void mergeDuplicatedNodes(const vector<MatrixXd>& vmV,
                          const vector<MatrixXi>& vmE,
                          MatrixXd& mVmerged,
                          MatrixXi& mEmerged,
                          vector<VectorXi>& vmaps,
                          double tol) {
  // Merge vertices

  vmaps.resize(vmV.size());

  int count = 0;  // Merged

  vector<VectorXd> vmergedV;
  vector<VectorXi> vmergedE;

  for (int i = 0; i < (int)vmV.size(); ++i) {
    // Check each mesh node againt merged ones
    vmaps[i] = -1 * VectorXi::Ones(vmV[i].rows());

    for (int j = 0; j < (int)vmV[i].rows(); ++j) {
      for (int k = 0; k < (int)vmergedV.size(); ++k) {
        Vector3d newPoint = vmV[i].row(j);
        Vector3d merPoint = vmergedV[k];
        if ((newPoint - merPoint).squaredNorm() <= tol) {
          vmaps[i][j] = k;
          break;  // Found!
        }
      }

      if (vmaps[i][j] == -1) {
        vmaps[i][j] = count++;

        // Add new merged point to vector
        vmergedV.push_back(vmV[i].row(j));
      }
    }
  }

  // Merge elements

  for (int i = 0; i < (int)vmE.size(); ++i) {
    const MatrixXi& mE = vmE[i];

    for (int j = 0; j < (int)mE.rows(); ++j) {
      VectorXi vei = mE.row(j);
      for (int k = 0; k < vei.size(); ++k)
        vei(k) = vmaps[i](vei(k));  // Map!

      vmergedE.push_back(vei);
    }
  }

  // Collect matrices

  mVmerged.resize(vmergedV.size(), vmergedV[0].size());
  mEmerged.resize(vmergedE.size(), vmergedE[0].size());

  for (int i = 0; i < mVmerged.rows(); ++i)
    mVmerged.row(i) = vmergedV[i];

  for (int i = 0; i < mEmerged.rows(); ++i)
    mEmerged.row(i) = vmergedE[i];
}

bool remeshTriMesh_CVT(Mesh_Tri& meshIn,
                       Tag traitPar,
                       Mesh_Tri& meshOut,
                       const string& options) {
  MatrixXi mFold;
  MatrixXd mVold;
  meshIn.GetNodesTrait(mVold, traitPar);
  meshIn.GetElemMatrix(mFold);

  // Build boundary loop mesh

  VectorXi vbloop;

  igl::boundary_loop(mFold, vbloop);

  int numVB = vbloop.size();
  MatrixXd mVB(numVB, 2);
  MatrixXi mEB(numVB, 2);
  for (int i = 0; i < numVB; ++i) {
    mVB(i, 0) = mVold(vbloop(i), 0);
    mVB(i, 1) = mVold(vbloop(i), 1);

    if (i != numVB - 1)
      mEB.row(i) = Vector2i(i, i + 1);
    else
      mEB.row(i) = Vector2i(i, 0);
  }

  // Triangulate parameterization

  MatrixXd mVnew2D;
  MatrixXd mVnew3D;
  MatrixXi mFnew;
  bool success =
      MeshUtils::triangulate(mVB, mEB, MatrixXd(), mVnew2D, mFnew, options);

  if (!success)
    return false;

  mVnew3D.resize(mVnew2D.rows(), 3);
  mVnew3D.block(0, 2, mVnew2D.rows(), 1).setZero();
  mVnew3D.block(0, 0, mVnew2D.rows(), 2) = mVnew2D;

  // Compute new points embedding

  vector<PtrS<Embedding>> vVebd(mVnew3D.rows());

  meshIn.ComputeEmbedding(mVnew3D, vVebd, traitPar);

  // Create new mesh and points

  meshOut.Init(mVnew3D, mFnew, Discretization::Tri3, meshIn.NodeTraits());

  for (int i = 0; i < (int)meshIn.NodeTraits().size(); ++i) {
    if (meshIn.NodeTraits()[i] == traitPar)
      continue;

    MatrixXd mVnewTrait = mVnew3D;

    for (int j = 0; j < (int)vVebd.size(); ++j) {
      if (vVebd[j]->Valid()) {
        mVnewTrait.row(j) = vVebd[j]->InterpolateValue(meshIn.NodeTraits()[i]);
      } else {
        // Went wrong
        return false;
      }
    }

    meshOut.SetNodesTrait(mVnewTrait, meshIn.NodeTraits()[i]);
  }

  return true;
}

bool parameterizeTriMesh_Harmonic(Mesh_Tri& mesh, Tag traitIn, Tag traitOut) {
  // TODO
  return false;
}

bool parameterizeTriMesh_Conformal(Mesh_Tri& mesh, Tag traitIn, Tag traitOut) {
  MatrixXi mF;
  MatrixXd mVX;
  MatrixXd mV0;
  mesh.GetNodesTrait(mVX, traitIn);
  mesh.GetNodesTrait(mV0, traitOut);
  mesh.GetElemMatrix(mF);

  // Get boundary and fix one point

  VectorXi vbloop;
  igl::boundary_loop(mF, vbloop);
  VectorXi vbfixIdx(2, 1);
  MatrixXd vbfixVal(2, 2);
  vbfixIdx(0) = vbloop(0);
  vbfixIdx(1) = vbloop(1);
  Vector3d vfix0 = mVX.row(vbloop(0));
  Vector3d vfix1 = mVX.row(vbloop(1));
  Real L = (vfix1 - vfix0).norm();
  vbfixVal.row(0) = Vector2d(0, 0);
  vbfixVal.row(1) = Vector2d(L, 0);

  // Compute conformal parameterization

  MatrixXd mUV;
  bool success = igl::lscm(mVX, mF, vbfixIdx, vbfixVal, mUV);
  mV0.block(0, 2, mV0.rows(), 1).setZero();
  mV0.block(0, 0, mV0.rows(), 2) = mUV;

  // Center in the origin

  VectorXd vm = mV0.colwise().mean();
  mV0 = mV0.rowwise() - vm.transpose();

  mesh.SetNodesTrait(mV0, traitOut);

  return success;
}

bool parameterizeTriMesh_ARAP(Mesh_Tri& mesh, Tag traitIn, Tag vtraitOut) {
  return false;
}

bool parameterizeTriMesh_Global(Mesh_Tri& mesh, Tag traitIn, Tag vtraitOut) {
  // TODO
  return false;
}

bool parameterizeTriMesh_Shell(Mesh_Tri& mesh,
                               Tag traitIn,
                               Tag traitOut,
                               Real young,
                               Real poisson) {
  MatrixXd mVX;
  MatrixXd mV0;
  MatrixXi mF;
  mesh.GetNodesTrait(mVX, traitIn);
  mesh.GetNodesTrait(mV0, traitOut);
  mesh.GetElemMatrix(mF);

  // Compute default thickness

  Real t = sqrt(mesh.ComputeArea(traitIn)) * 1e-3;

  // Create thin-shell model

  PtrS<Simulable_ThinShell> pModel(new Simulable_ThinShell());

  vector<Tag> vnTraits;
  vnTraits.push_back(Tag::Position_X);
  vnTraits.push_back(Tag::Position_0);
  vnTraits.push_back(Tag::Velocity);
  Discretization D = Discretization::Tri3;
  pModel->SetupOptions().m_pMesh.reset(new Mesh_Tri(mVX, mF, D, vnTraits));
  pModel->SetupOptions().m_pMesh->SetNodesTrait(mVX * 0, Tag::Velocity);

  pModel->SetupOptions().m_material.InitLinearFromYoungPoisson(young, poisson,
                                                               1e3);
  pModel->SetupOptions().m_material.AddParameter(ParameterSet::Param_BendingK,
                                                 1e3);
  pModel->SetupOptions().m_material.AddParameter(ParameterSet::Param_Thickness,
                                                 t);
  // pModel->SetupOptions().m_material.PhysicalModel() =
  // ParameterSet::GetModel::Model_StVK; pModel->SetupOptions().m_numQuadPoints
  // = 1;

  pModel->Setup();

  // Set planar rest configuration

  for (int i = 0; i < pModel->GetEnergyElements_ShellHinge().size(); ++i)
    pModel->GetEnergyElements_ShellHinge()[i]->SetRestTheta(0.0);

  pModel->DirtyMechanics();

  // Create static problem and solver

  OptimSolverOptions options;
  options.qpSolverType = QPSolverType::Newton;
  options.lSearchType = LSearchType::WolfeStrong;
  options.lsearch_numTry = 50;
  options.lSearch_alpha = .5;
  options.tolMaxError = 1e-6;
  options.numMaxIters = 1000;

  PtrS<OptimProblem_BasicStatic> pProblem(
      new OptimProblem_BasicStatic(pModel.get()));
  PtrS<OptimSolver_USQP_LS> pSolver(
      new OptimSolver_USQP_LS(pProblem.get(), options));

  const OptimState& state = pSolver->SolveFull();

  if (state.m_result == OSResult::SUCCESS) {
    pModel->GetMesh().GetNodesTrait(mV0, Tag::Position_X);

    // Rotate to XY plane

    Vector3d x0 = pModel->GetMesh().Faces()[0]->Nodes()[0]->Traits().Vector3d(
        Tag::Position_X);
    Vector3d x1 = pModel->GetMesh().Faces()[0]->Nodes()[1]->Traits().Vector3d(
        Tag::Position_X);
    Vector3d x2 = pModel->GetMesh().Faces()[0]->Nodes()[2]->Traits().Vector3d(
        Tag::Position_X);
    Vector3d vi = (x1 - x0).normalized();
    Vector3d temp = (x2 - x0).normalized();
    Vector3d vj = vi.cross(temp).normalized();
    Vector3d vk = vj.cross(vi).normalized();

    Matrix3d mR;
    mR.col(0) = vi;
    mR.col(1) = vk;
    mR.col(2) = vj;

    mV0 = (mV0.rowwise() - x0.transpose()) * mR;
    assert(mV0.col(2).isZero());

    // Set resulting parameterization
    mesh.SetNodesTrait(mV0, traitOut);

    return true;
  } else {
    return false;
  }
}

void CreateTetMesh_Grid(MatrixXr& mV,
                        MatrixXi& mE,
                        const Vector3r& scale,
                        const Vector3i& dims) {
  int numNodeX = dims[0] + 1;
  int numNodeY = dims[1] + 1;
  int numNodeZ = dims[2] + 1;
  int numNodes = numNodeX * numNodeY * numNodeZ;
  int numEles = dims[0] * dims[1] * dims[2] * 5;
  Vector3r cubeScale = scale;
  cubeScale[0] /= (Real)dims[0];
  cubeScale[1] /= (Real)dims[1];
  cubeScale[2] /= (Real)dims[2];

  // Create coorinates in X, Y, Z order
  mV.resize(3, numNodes);
  int countNodes = 0;
  for (int k = 0; k < numNodeZ; ++k)
    for (int j = 0; j < numNodeY; ++j)
      for (int i = 0; i < numNodeX; ++i)
        mV.col(countNodes++) =
            Vector3r{i * cubeScale[0], j * cubeScale[1], k * cubeScale[2]};

  // Create connectivity
  mE.resize(4, numEles);
  int countEles = 0;
  int lineSize = numNodeX;              // Size of a line of X nodes
  int slideSize = numNodeX * numNodeY;  // Size of a XY slide
  for (int k = 0; k < dims[2]; ++k)
    for (int j = 0; j < dims[1]; ++j)
      for (int i = 0; i < dims[0]; ++i) {
        // Index of node 0
        int offset = slideSize * k + lineSize * j + i;

        std::vector<int> disps = {offset,             // Node 0
                                  offset + 1,         // Node 1
                                  offset + lineSize,  // ...
                                  offset + lineSize + 1,
                                  offset + slideSize,
                                  offset + slideSize + 1,
                                  offset + slideSize + lineSize,
                                  offset + slideSize + lineSize + 1};  // Node 7

        //
        //         6 ------- 7
        //       / |       / |
        //      /  |      /  |
        //     4 ------- 5   |
        //     |   2 ----|-- 3
        //     |  /      |  /
        //     | /       | /
        //     0 ------- 1
        //
        mE.col(countEles++) =
            Vector4i(disps[0], disps[1], disps[2], disps[4]);  // corner vert 0
        mE.col(countEles++) =
            Vector4i(disps[6], disps[7], disps[4], disps[2]);  // corner vert 6
        mE.col(countEles++) =
            Vector4i(disps[5], disps[4], disps[7], disps[1]);  // corner vert 5
        mE.col(countEles++) =
            Vector4i(disps[3], disps[2], disps[1], disps[7]);  // corner vert 3
        mE.col(countEles++) =
            Vector4i(disps[1], disps[2], disps[4], disps[7]);  // interior one
      }
}

void CreateTetMesh_Cube(MatrixXr& mV, MatrixXi& mE, const Vector3r& scale) {
  // A solid unit cube with one corner at (0,0,0)
  //
  //         6 ------- 7
  //       / |       / |
  //      /  |      /  |
  //     4 ------- 5   |
  //     |   2 ----|-- 3
  //     |  /      |  /
  //     | /       | /
  //     0 ------- 1
  //
  mV.resize(3, 8);
  mV.col(0) = Vector3r{0.0, 0.0, 0.0};                 // 0
  mV.col(1) = Vector3r{scale[0], 0.0, 0.0};            // 1
  mV.col(2) = Vector3r{0.0, scale[1], 0.0};            // 2
  mV.col(3) = Vector3r{scale[0], scale[1], 0.0};       // 3
  mV.col(4) = Vector3r{0.0, 0.0, scale[2]};            // 4
  mV.col(5) = Vector3r{scale[0], 0.0, scale[2]};       // 5
  mV.col(6) = Vector3r{0.0, scale[1], scale[2]};       // 6
  mV.col(7) = Vector3r{scale[0], scale[1], scale[2]};  // 7

  mE.resize(4, 5);
  mE.col(0) = Vector4i{0, 1, 2, 4};  // corner vert 0
  mE.col(1) = Vector4i{6, 7, 4, 2};  // corner vert 6
  mE.col(2) = Vector4i{5, 4, 7, 1};  // corner vert 5
  mE.col(3) = Vector4i{3, 2, 1, 7};  // corner vert 4
  mE.col(4) = Vector4i{1, 2, 4, 7};  // interior one
}

void CreateTetMesh_TwoSharingFace(MatrixXr& mV,
                                  MatrixXi& mE,
                                  const Vector3r& scale) {
  // Two tetrahedra sharing a face
  mV.resize(3, 5);
  mV.col(0) = Vector3r{0.0, 0.0, 0.0};            // 0
  mV.col(1) = Vector3r{scale[0], 0.0, 0.0};       // 1
  mV.col(2) = Vector3r{0.0, scale[1], 0.0};       // 2
  mV.col(3) = Vector3r{0.0, 0.0, scale[2]};       // 3
  mV.col(4) = Vector3r{scale[0], 0.0, scale[2]};  // 4

  mE.resize(4, 2);
  mE.col(0) = Vector4i{0, 1, 2, 3};
  mE.col(1) = Vector4i{1, 2, 3, 4};
}

void CreateTetMesh_TwoSharingEdge(MatrixXr& mV,
                                  MatrixXi& mE,
                                  const Vector3r& scale) {
  // Two tetrahedra sharing an edge
  mV.resize(3, 6);
  mV.col(0) = Vector3r{0.0, 0.0, 0.0};                 // 0
  mV.col(1) = Vector3r{scale[0], 0.0, 0.0};            // 1
  mV.col(2) = Vector3r{0.0, scale[1], 0.0};            // 2
  mV.col(3) = Vector3r{0.0, 0.0, scale[2]};            // 3
  mV.col(4) = Vector3r{scale[0], 0.0, scale[2]};       // 4
  mV.col(5) = Vector3r{scale[0], scale[1], scale[2]};  // 5

  mE.resize(4, 2);
  mE.col(0) = Vector4i{0, 1, 2, 3};
  mE.col(1) = Vector4i{2, 1, 4, 5};
}

void CreateTetMesh_TwoSharingNode(MatrixXr& mV,
                                  MatrixXi& mE,
                                  const Vector3r& scale) {
  // Two tetrahedra sharing a node
  mV.resize(3, 7);
  mV.col(0) = Vector3r{0.0, 0.0, 0.0};                 // 0
  mV.col(1) = Vector3r{scale[0], 0.0, 0.0};            // 1
  mV.col(2) = Vector3r{0.0, scale[1], 0.0};            // 2
  mV.col(3) = Vector3r{0.0, 0.0, scale[2]};            // 3
  mV.col(4) = Vector3r{scale[0], 0.0, scale[2]};       // 4
  mV.col(5) = Vector3r{scale[0], scale[1], scale[2]};  // 5
  mV.col(6) = Vector3r{0.0, scale[1], scale[2]};       // 6

  mE.resize(4, 2);
  mE.col(0) = Vector4i{0, 1, 2, 3};
  mE.col(1) = Vector4i{2, 6, 4, 5};
}

void CreateTetMesh_SingleTet(MatrixXr& mV,
                             MatrixXi& mE,
                             const Vector3r& scale) {
  // Unit tet with one corner at (0,0,0)
  //
  //     3 _ _2
  //     | \ / \
            //     |  / \ \
            //     |/     \\
            //     0 ----- 1
  //
  mV.resize(3, 4);
  mV.col(0) = Vector3r{0.0, 0.0, 0.0};       // 0
  mV.col(1) = Vector3r{scale[0], 0.0, 0.0};  // 1
  mV.col(2) = Vector3r{0.0, scale[1], 0.0};  // 2
  mV.col(3) = Vector3r{0.0, 0.0, scale[2]};  // 3
  mE.resize(4, 1);
  mE.col(0) = Vector4i{0, 1, 2, 3};
}

void ComputeTriMesh_TetMeshInternal(MatrixXi& mFout, const MatrixXi& mTin) {
  // Each tetrahedron defines 4 faces. With
  // normals pointing outwards this faces are:
  //
  //     3 _ _2
  //     | \ / \
            //     |  / \ \
            //     |/     \\
            //     0 ----- 1
  //
  // 2, 1, 0 -> Normal (0, -1, 0)
  // 0, 1, 3 -> Normal (0, 0, -1)
  // 3, 2, 0 -> Normal (-1, 0, 0)
  // 1, 2, 3 -> Normal (1,1,1)*sqrt(3)

  mFout.resize(3, 4 * mTin.cols());
  int countFaces = 0;
  for (int i = 0; i < mTin.cols(); ++i) {
    mFout.col(countFaces++) = Vector3i{mTin(2, i), mTin(1, i), mTin(0, i)};
    mFout.col(countFaces++) = Vector3i{mTin(0, i), mTin(1, i), mTin(3, i)};
    mFout.col(countFaces++) = Vector3i{mTin(3, i), mTin(2, i), mTin(0, i)};
    mFout.col(countFaces++) = Vector3i{mTin(1, i), mTin(2, i), mTin(3, i)};
  }
}

void ComputeTriMesh_TetMeshSurface(MatrixXi& mFout, const MatrixXi& mTin) {
  // Each tetrahedron defines 4 faces. With
  // normals pointing outwards this faces are:
  //
  //     3 _ _2
  //     | \ / \
            //     |  / \ \
            //     |/     \\
            //     0 ----- 1
  //
  // 2, 1, 0 -> Normal (0, -1, 0)
  // 0, 1, 3 -> Normal (0, 0, -1)
  // 3, 2, 0 -> Normal (-1, 0, 0)
  // 1, 2, 3 -> Normal (1,1,1)*sqrt(3)

  // Surface faces are those appearing only once
  struct FaceInfo {
    Vector3i indices;
    int repetitions;
  };

  map<iVector, FaceInfo> facesSet;

  mFout.resize(3, 4 * mTin.cols());
  int countFaces = 0;
  for (int i = 0; i < mTin.cols(); ++i) {
    vector<Vector3i> tetFaces(4);
    tetFaces[0] = Vector3i{mTin(2, i), mTin(1, i), mTin(0, i)};
    tetFaces[1] = Vector3i{mTin(0, i), mTin(1, i), mTin(3, i)};
    tetFaces[2] = Vector3i{mTin(3, i), mTin(2, i), mTin(0, i)};
    tetFaces[3] = Vector3i{mTin(1, i), mTin(2, i), mTin(3, i)};
    for (auto face : tetFaces) {
      iVector indices{face.x(), face.y(), face.z()};
      sort(indices.begin(), indices.end());
      auto itFace = facesSet.find(indices);
      if (itFace == facesSet.end())
        facesSet[indices] = FaceInfo{face, 1};
      else
        itFace->second.repetitions++;
    }
  }

  // Add one face for each unique item
  vector<Vector3i> surfaceFaces;
  for (auto itFace = facesSet.begin(); itFace != facesSet.end(); ++itFace)
    if (itFace->second.repetitions == 1)
      surfaceFaces.push_back(itFace->second.indices);

  // Build final connectivity matrix
  mFout.resize(3, surfaceFaces.size());
  memcpy(mFout.data(), surfaceFaces.data(),
         sizeof(int) * 3 * surfaceFaces.size());
}
}  // namespace MeshUtils
}  // namespace PhySim