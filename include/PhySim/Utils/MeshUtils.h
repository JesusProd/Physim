//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, FRL
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

namespace PhySim {

using namespace std;
using namespace Eigen;

class Mesh_Tri;

namespace MeshUtils {

void create2DGridNodes(int N, int M, const Vector2d& sizes, MatrixXd& mV);
void create3DGridNodes(int N,
                       int M,
                       int O,
                       const Vector3d& sizes,
                       MatrixXd& mV);

void create2DGridIndex_Tri3(int N, int M, MatrixXi& mE);
void create2DGridIndex_Quad4(int N, int M, MatrixXi& mE);
void create3DGridIndex_Tri3(int N, int M, int O, MatrixXi& mT);
void create3DGridIndex_Hex8(int N, int M, int O, MatrixXi& mE);

void extract2DGridQuad4Surface(const MatrixXd& mV,
                               const MatrixXi& mH,
                               VectorXi& vmap,
                               MatrixXi& mS,
                               MatrixXd& mN);
void extract3DGridHex8Surface_Tri3(const MatrixXd& mV,
                                   const MatrixXi& mH,
                                   VectorXi& vmap,
                                   MatrixXi& mS,
                                   MatrixXd& mN);
void extract3DGridHex8Surface_Quad4(const MatrixXd& mV,
                                    const MatrixXi& mH,
                                    VectorXi& vmap,
                                    MatrixXi& mS,
                                    MatrixXd& mN);

void extractFromHexaMesh_Tri(const MatrixXi& mH, MatrixXi& mT);
void extractFromHexaMesh_Quad(const MatrixXi& mH, MatrixXi& mQ);
void extractFromHexaMesh_Edge(const MatrixXi& mH, MatrixXi& mE);

bool triangulate(const MatrixXd& mV,
                 const MatrixXi& mE,
                 const MatrixXd& mH,
                 MatrixXd& mVout,
                 MatrixXi& mFout,
                 const string& options = "a0.005q");
bool tetrahedralize(const MatrixXd& mV,
                    const MatrixXi& mE,
                    const MatrixXd& mH,
                    MatrixXd& mVout,
                    MatrixXi& mFout,
                    MatrixXi& mTout,
                    const string& options = "pq1.414");

bool create2DRectangle(Real sizeX,
                       Real sizeY,
                       int numeX,
                       int numeY,
                       MatrixXd& mVout,
                       MatrixXi& mFout,
                       const string& options = "a0.005q");
bool create2DCircle(Real radius,
                    int numS,
                    MatrixXd& mV,
                    MatrixXi& mF,
                    const string& options = "a0.005q");

void mergeDuplicatedNodes(const MatrixXd& vmV,
                          const MatrixXi& vmE,
                          MatrixXd& mVmerged,
                          MatrixXi& mEmerged,
                          VectorXi& vmaps,
                          double tol = 1e-6);
void mergeDuplicatedNodes(const vector<MatrixXd>& vmV,
                          const vector<MatrixXi>& vmE,
                          MatrixXd& mVmerged,
                          MatrixXi& mEmerged,
                          vector<VectorXi>& vmaps,
                          double tol = 1e-6);

void filterSelectedNodes(const MatrixXd& mVin,
                         const VectorXi& vsel,
                         MatrixXd& mVout);
void filterSelectedNodes(const MatrixXd& mVin,
                         const MatrixXi& mEin,
                         const VectorXi& vsel,
                         MatrixXd& mVout,
                         MatrixXi& mEout);
void filterIsolatedNodes(const MatrixXd& mVin,
                         const MatrixXi& mEin,
                         MatrixXd& mVOut,
                         MatrixXi& mEout,
                         VectorXi& vmap);

void splitQuadMeshInTris(const MatrixXi& mEin, MatrixXi& mEout);
void splitHexaMeshInTets(const MatrixXi& mEin, MatrixXi& mEout);

void combineMeshes(const vector<MatrixXd>& vmV,
                   const vector<MatrixXi>& vmF,
                   int M,
                   MatrixXd& mVout,
                   MatrixXi& mFout);

bool remeshTriMesh_CVT(Mesh_Tri& meshIn,
                       Tag vtraitParam,
                       Mesh_Tri& meshOut,
                       const string& options = "a0.005q");

bool parameterizeTriMesh_Harmonic(Mesh_Tri& mesh, Tag vtraitIn, Tag vtraitOut);
bool parameterizeTriMesh_Conformal(Mesh_Tri& mesh, Tag vtraitIn, Tag vtraitOut);
bool parameterizeTriMesh_ARAP(Mesh_Tri& mesh, Tag vtraitIn, Tag vtraitOut);
bool parameterizeTriMesh_Global(Mesh_Tri& mesh, Tag vtraitIn, Tag vtraitOut);
bool parameterizeTriMesh_Shell(Mesh_Tri& mesh,
                               Tag vtraitIn,
                               Tag vtraitOut,
                               Real young = 1e3,
                               Real poisson = 0.25);

// Create the coordinates and connectivity of a grid tetrahedral mesh
void CreateTetMesh_Grid(MatrixXr& mVout,
                        MatrixXi& mTout,
                        const Vector3r& scale,
                        const Vector3i& dims);

// Create the coordinates and connectivity of a cube tetrahedral mesh
void CreateTetMesh_Cube(MatrixXr& mVout,
                        MatrixXi& mTout,
                        const Vector3r& scale);

// Create the coordinates and connectivity of two tets sharing a face
void CreateTetMesh_TwoSharingFace(MatrixXr& mVout,
                                  MatrixXi& mTout,
                                  const Vector3r& scale);

// Create the coordinates and connectivity of two tets sharing an edge
void CreateTetMesh_TwoSharingEdge(MatrixXr& mVout,
                                  MatrixXi& mTout,
                                  const Vector3r& scale);

// Create the coordinates and connectivity of two tets sharing a node
void CreateTetMesh_TwoSharingNode(MatrixXr& mVout,
                                  MatrixXi& mTout,
                                  const Vector3r& scale);

// Create the coordinates and connectivity of a single tetrahedron
void CreateTetMesh_SingleTet(MatrixXr& mV,
                             MatrixXi& mTout,
                             const Vector3r& scale);

// Compute the connectivity of the internal 3D triangle mesh of a tetrahedral
// mesh
void ComputeTriMesh_TetMeshInternal(MatrixXi& mFout, const MatrixXi& mTin);

// Compute the connectivity of the surface 3D triangle mesh of a tetrahedral
// mesh
void ComputeTriMesh_TetMeshSurface(MatrixXi& mFout, const MatrixXi& mTin);

template <typename T, int N>
// Compute the axis-aligned BB of the specified set of points
void ComputePointsAABB(AABB<T, N>& outAABB, const Matrix<T, N, -1>& mV) {
  outAABB.m_minCor = mV.rowwise().minCoeff();
  outAABB.m_maxCor = mV.rowwise().maxCoeff();
}

template <typename T, int N>
// Copute an axis-aligned BB enclosing one side of the AABB enclosing a set of
// points. Side is specified through an integer value [side] with a [0,5] value
// meaning -X, +X, -Y, +Y, -Z, +Z. The [scale] vector indicates what proportion
// of the full AABB is considered in each dimension. For instance: for side 1
// and scale (0.1,1.0,1.0), the AABB encloses the 10% of the +X side of the big
// box.
void ComputePointsAABBSideAABB(AABB<T, N>& outAABB,
                               const Matrix<T, N, -1>& mV,
                               unsigned short side,
                               const Vector<T, N>& scale) {
  // For 2D, only [0,3] valid sides. Meaning -X, +X, -Y, +Y
  if (N == 2 && side > 3)
    throw PhySim::exception("Invalid cube side specification");

  // For 3D, only [0,5] valid sides. Meaning -X, +X, -Y, +Y, -Z, +Z
  if (N == 3 && side > 5)
    throw PhySim::exception("Invalid cube side specification");

  // Compute full mesh AABB
  AABB<T, N> fullAABB;
  ComputePointsAABB<Real, N>(fullAABB, mV);
  Vector3r ranges = fullAABB.m_maxCor - fullAABB.m_minCor;

  // Shift corners to avoid numerical issues
  Vector<T, N> eps = ranges * 1e-6;
  Vector<T, N> minCor = fullAABB.m_minCor - eps;
  Vector<T, N> maxCor = fullAABB.m_maxCor + eps;
  Vector<T, N> center = (minCor + maxCor) * 0.5;

  // Shrink aabb to the corresponding side
  bool scaleX = false, scaleY = false, scaleZ = false;
  switch (side) {
    case 0:
      maxCor[0] = minCor[0] + ranges[0] * scale[0];
      scaleY = true;
      scaleZ = true;
      break;
    case 1:
      minCor[0] = maxCor[0] - ranges[0] * scale[0];
      scaleY = true;
      scaleZ = true;
      break;
    case 2:
      maxCor[1] = minCor[1] + ranges[1] * scale[1];
      scaleX = true;
      scaleZ = true;
      break;
    case 3:
      minCor[1] = maxCor[1] - ranges[1] * scale[1];
      scaleX = true;
      scaleZ = true;
      break;
    case 4:
      maxCor[2] = minCor[2] + ranges[2] * scale[2];
      scaleY = true;
      scaleX = true;
      break;
    case 5:
      minCor[2] = maxCor[2] - ranges[2] * scale[2];
      scaleY = true;
      scaleX = true;
      break;
  }

  // Scale the other two axis accoringly
  auto rescale = [](Vector<T, N>& corner, const Vector<T, N>& center,
                    const Vector<T, N>& scale, int i) {
    corner[i] = (corner[i] - center[i]) * scale[i] + center[i];
  };

  if (scaleX) {
    rescale(minCor, center, scale, 0);
    rescale(maxCor, center, scale, 0);
  }
  if (scaleY) {
    rescale(minCor, center, scale, 1);
    rescale(maxCor, center, scale, 1);
  }
  if (scaleZ) {
    rescale(minCor, center, scale, 2);
    rescale(maxCor, center, scale, 2);
  }

  outAABB = AABB<T, N>(minCor, maxCor);
}

template <typename T, int N>
// Select the points of a point cloud contained in the specified axis-aligned
// bounding box
void SelectPointsAABB(vector<int>& indices,
                      const Matrix<T, N, -1>& mV,
                      const AABB<T, N>& aabb) {
  indices.clear();

  for (int p = 0; p < mV.cols(); ++p) {
    bool in = true;
    for (int d = 0; d < mV.rows() && in; ++d)
      if (mV(d, p) < aabb.m_minCor(d) || mV(d, p) > aabb.m_maxCor(d))
        in = false;
    if (in)
      indices.push_back(p);
  }
}
}  // namespace MeshUtils
}  // namespace PhySim