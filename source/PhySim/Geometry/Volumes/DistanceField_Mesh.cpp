//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================


#include <PhySim/Geometry/Volumes/DistanceField_Mesh.h>

#include <PhySim/Geometry/Meshes/Mesh_Face.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Utils/GeometryUtils.h>

#include <unordered_set>

using namespace PhySim;

void DistanceField_Mesh::Setup(const MatrixXd& mPoints, const MatrixXi& mTriangles, int LargestAxisSize, int Padding)
{
    m_mPoints = mPoints;
    m_mFaces = mTriangles;

    assert(m_mPoints.cols() == 3 && "Only 3D meshes are supported.");
    assert(m_mFaces.cols() == 3 && "Only triangular meshes are supported.");
    assert(LargestAxisSize > 0 && "Invalid largest axis size.");
    assert(Validate() && "The given mesh is not watertight.");
    
    // Determine bounding box of the input mesh.
    Vector3d vMinAABB = mPoints.colwise().minCoeff();
    Vector3d vMaxAABB = mPoints.colwise().maxCoeff();
    Vector3d vAABBSize = vMaxAABB - vMinAABB;

    // Choose largest axis size and determine delta.
    Vector3d vDelta = Vector3d::Ones() * vAABBSize.maxCoeff() / LargestAxisSize;

    // Determine shape and origin.
    Vector3i vShape;
    vShape[0] = int(ceil(vAABBSize[0] / vDelta[0])) + 2 * Padding;
    vShape[1] = int(ceil(vAABBSize[1] / vDelta[1])) + 2 * Padding;
    vShape[2] = int(ceil(vAABBSize[2] / vDelta[2])) + 2 * Padding;
    Vector3d vOrigin = vMinAABB - vDelta * Padding;

    // Allocate field data and initialize.
    m_sfDistances.Setup(vShape, vOrigin, vDelta);
    m_sfFaces.Setup(vShape, vOrigin, vDelta);
    m_sfDistances.Grid().setConstant(numeric_limits<Real>::infinity());
    m_sfFaces.Grid().setConstant(-1);

    // Now that everything is in place, we need to actually generate the
    // distance field. To do this, we follow the standard fast sweeping
    // algorithm used in the literature (for the 2D case, look for 8SSEDT).
    // This implementation is based on Christopher Batty's SDFGen algorithm.

    // First, initialize the volume by rasterizing the faces within a narrow band.
    Rasterize();

    // Then, use fast sweeping to fill in the adjacent data.
    for (size_t i = 0; i < 2; ++i)
    {
        Sweep(Vector3i(+1, +1, +1));
        Sweep(Vector3i(-1, -1, -1));
        Sweep(Vector3i(+1, +1, -1));
        Sweep(Vector3i(-1, -1, +1));
        Sweep(Vector3i(+1, -1, +1));
        Sweep(Vector3i(-1, +1, -1));
        Sweep(Vector3i(+1, -1, -1));
        Sweep(Vector3i(-1, +1, +1));
    }

    // Up to this point, we have safely determined the closest primitive and its distance
    // for each voxel in the volume. All that remains is to determine their sign so we
    // can discern between outside or inside of the volume.
    DetermineSigns();
}

Vector3i DistanceField_Mesh::Shape() const
{
    return m_sfDistances.Shape();
}

Vector3d DistanceField_Mesh::Origin() const
{
    return m_sfDistances.Origin();
}

Vector3d DistanceField_Mesh::Delta() const
{
    return m_sfDistances.Delta();
}

const MatrixXd& DistanceField_Mesh::Points() const
{
    return m_mPoints;
}

const MatrixXi& DistanceField_Mesh::Faces() const
{
    return m_mFaces;
}

const Tensor3d& DistanceField_Mesh::DistanceData() const
{
    return m_sfDistances.Grid();
}

const Tensor3i& DistanceField_Mesh::FaceData() const
{
    return m_sfFaces.Grid();
}

Real DistanceField_Mesh::Sample(const Vector3d& vPoint) const
{
    return m_sfDistances.Sample(vPoint);
}

Vector3d DistanceField_Mesh::Gradient(const Vector3d& vPoint) const
{
    return m_sfDistances.Gradient(vPoint);
}

Vector3i DistanceField_Mesh::IndexOf(const Vector3d& vPoint, bool clamp)
{
    return m_sfDistances.IndexOf(vPoint, clamp);
}

Vector3d DistanceField_Mesh::PointOf(const Vector3i& vIndex)
{
    return m_sfDistances.PointOf(vIndex);
}

void DistanceField_Mesh::Compare(const Vector3d& vPoint, const Vector3i& vIndex, const Vector3i& vDeltaIndex)
{
    Vector3i vOtherIndex = vIndex + vDeltaIndex;

    int i = m_sfFaces[vOtherIndex];
    if (i == -1) return;

    Vector3i vF = m_mFaces.row(i).transpose();
    Vector3d vA = m_mPoints.row(vF[0]).transpose();
    Vector3d vB = m_mPoints.row(vF[1]).transpose();
    Vector3d vC = m_mPoints.row(vF[2]).transpose();
    Real D = GeometryUtils::sqrDistanceToTriangle(vPoint, vA, vB, vC);

    if (D < m_sfDistances[vIndex])
    {
        m_sfDistances[vIndex] = D;
        m_sfFaces[vIndex] = i;
    }

}

bool DistanceField_Mesh::Validate()
{
    // TODO: Trust user... for now. ô_ó
    // TODO: Test if the mesh is manifold.
    // TODO: Test if the mesh is watertight.
    return true;
}

void DistanceField_Mesh::Rasterize()
{
    const size_t M = m_mFaces.rows();
    const Vector3i vShape = Shape();
    const int narrowBand = 2;

    for (size_t i = 0; i < M; ++i)
    {
        Vector3i vF = m_mFaces.row(i).transpose();
        Vector3d vA = m_mPoints.row(vF[0]).transpose();
        Vector3d vB = m_mPoints.row(vF[1]).transpose();
        Vector3d vC = m_mPoints.row(vF[2]).transpose();

        Vector3d vFaceMinAABB = vA.cwiseMin(vB).cwiseMin(vC);
        Vector3d vFaceMaxAABB = vA.cwiseMax(vB).cwiseMax(vC);
        Vector3i vFaceMinIndex = IndexOf(vFaceMinAABB) - Vector3i::Constant(narrowBand);
        Vector3i vFaceMaxIndex = IndexOf(vFaceMaxAABB) + Vector3i::Constant(narrowBand);
        vFaceMinIndex = vFaceMinIndex.cwiseMax(Vector3i::Zero());
        vFaceMaxIndex = vFaceMaxIndex.cwiseMin(vShape - Vector3i::Ones());

        for (int z = vFaceMinIndex[2]; z < vFaceMaxIndex[2]; ++z)
        for (int y = vFaceMinIndex[1]; y < vFaceMaxIndex[1]; ++y)
        for (int x = vFaceMinIndex[0]; x < vFaceMaxIndex[0]; ++x)
        {
            Vector3i vI(x, y, z);
            Vector3d vP = PointOf(vI);
            Real D = GeometryUtils::sqrDistanceToTriangle(vP, vA, vB, vC);
            
            if (D < m_sfDistances[vI] || m_sfFaces[vI] == -1)
            {
                m_sfDistances[vI] = D;
                m_sfFaces[vI] = int(i);
            }
        }
    }
}

void DistanceField_Mesh::DetermineSigns()
{
    // In order to build the DistanceFieldVolume, we need to compute the signed
    // distance to the closest surface for each voxel of the volume. Finding the
    // unsigned distance is easy (it can be done geometrically), however, signing
    // the distance is not that trivial.
    //
    // There are multiple approaches to do this. Either we find out whether or not
    // the voxel lies inside the mesh volume through Jordan's theorem (i.e. cast a
    // ray passing through the voxel and count intersections) or compute a local
    // sign along with the distance by performing a plane test with the triangle
    // normal.
    //
    // We follow the second approach as it allows to perform the computation
    // of both the distance and sign within a single step. However, when the
    // sampling point is close to a vertex or an edge, this approach might yield
    // an incorrect computation of the sign.
    //
    // Baerentzen and Anaes (Generating Signed Distance Fields From Triangle
    // Meshes, 2001) solve this by using the angle-weighted pseudonormal in
    // those cases. This requires to preprocess the geometry using a
    // representation that includes neighbouring information.

    const size_t N = m_mPoints.rows();
    const size_t M = m_mFaces.rows();
    const Vector3i vShape = Shape();

    PtrS<Mesh_Face> pMesh = NewS<Mesh_Face>(m_mPoints, m_mFaces);;
    MatrixXd mVertexNormals = MatrixXd::Zero(N, 3);
    MatrixXd mFaceNormals = MatrixXd::Zero(M, 3);
    MatrixXd mEdgeNormals = MatrixXd::Zero(3 * M, 3);
    
    // Compute per-triangle normals.
    for (size_t i = 0; i < M; ++i)
    {
        Vector3i vF = m_mFaces.row(i).transpose();
        Vector3d vA = m_mPoints.row(vF[0]).transpose();
        Vector3d vB = m_mPoints.row(vF[1]).transpose();
        Vector3d vC = m_mPoints.row(vF[2]).transpose();

        Vector3d vAB = vB - vA;
        Vector3d vAC = vC - vA;
        Vector3d vBC = vB - vC;
        vAB.normalize();
        vAC.normalize();
        vBC.normalize();

        Vector3d vN = vAB.cross(vAC).normalized();
        Real wA = acos(min(max(vAB.dot(vAC), -1.0), 1.0));
        Real wB = acos(min(max(-vBC.dot(vAB), -1.0), 1.0));
        Real wC = M_PI - wA - wB;

        mFaceNormals.row(i) = vN.transpose();
        mVertexNormals.row(vF[0]) += wA * vN.transpose();
        mVertexNormals.row(vF[1]) += wB * vN.transpose();
        mVertexNormals.row(vF[2]) += wC * vN.transpose();
    }
    
    // Compute the avge. edge normal.
    for (size_t i = 0; i < M; ++i)
    {
        Mesh_Face::HE_Half* pHE = pMesh->GetHEFace(int(i))->m_pheHalf;

        for (size_t j = 0; j < 3; ++j, pHE = pHE->m_pheNext)
        {
            size_t k = pHE->m_phePair->m_pheFace->m_pHandle->ID();
            mEdgeNormals.row(3 * i + j) = mFaceNormals.row(i) + mFaceNormals.row(k);
        }
    }

    // Normalize data.
    mVertexNormals.rowwise().normalize();
    mFaceNormals.rowwise().normalize();
    mEdgeNormals.rowwise().normalize();

    // With all this information, we are now ready to compute the signed distance for
    // all the points within the volume.
    for (int z = 0; z < vShape[2]; ++z)
    for (int y = 0; y < vShape[1]; ++y)
    for (int x = 0; x < vShape[0]; ++x)
    {
        int i = m_sfFaces[{x, y, z}];
        assert(i != -1 && "No face assigned to voxel.");

        Vector3d vP = PointOf(Vector3i(x, y, z));
        Vector3i vF = m_mFaces.row(i).transpose();
        Vector3d vA = m_mPoints.row(vF[0]).transpose();
        Vector3d vB = m_mPoints.row(vF[1]).transpose();
        Vector3d vC = m_mPoints.row(vF[2]).transpose();
        Vector3d vNA = mVertexNormals.row(vF[0]).transpose();
        Vector3d vNB = mVertexNormals.row(vF[1]).transpose();
        Vector3d vNC = mVertexNormals.row(vF[2]).transpose();
        Vector3d vNAB = mEdgeNormals.row(3 * i + 0).transpose();
        Vector3d vNBC = mEdgeNormals.row(3 * i + 1).transpose();
        Vector3d vNCA = mEdgeNormals.row(3 * i + 2).transpose();

        // TODO: Sign computation doesn't seem to be completely robust.
        // Implement this one instead: https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-34-signed-distance-fields-using-single-pass-gpu
        m_sfDistances[{x, y, z}] *= GeometryUtils::determineTriangleDistanceSign(vP, vA, vB, vC, vNA, vNB, vNC, vNAB, vNBC, vNCA);
    }

    // Done!
}

void DistanceField_Mesh::Sweep(const Vector3i& vDeltaIndex)
{
    const Vector3i vShape = Shape();

    // Determine range of coordinates.
    int x0, x1, y0, y1, z0, z1;
    if (vDeltaIndex[0] > 0) { x0 = 1; x1 = vShape[0]; } else { x0 = vShape[0] - 2; x1 = -1; }
    if (vDeltaIndex[1] > 0) { y0 = 1; y1 = vShape[1]; } else { y0 = vShape[1] - 2; y1 = -1; }
    if (vDeltaIndex[2] > 0) { z0 = 1; z1 = vShape[2]; } else { z0 = vShape[2] - 2; z1 = -1; }

    // For each point, check neighbors and update distance.
    for (int z = z0; z != z1; z += vDeltaIndex[2])
    for (int y = y0; y != y1; y += vDeltaIndex[1])
    for (int x = x0; x != x1; x += vDeltaIndex[0])
    {
        Vector3i vI = Vector3i(x, y, z);
        Vector3d vP = PointOf(vI);
        Compare(vP, vI, Vector3i(-vDeltaIndex[0], 0, 0));
        Compare(vP, vI, Vector3i(0, -vDeltaIndex[1], 0));
        Compare(vP, vI, Vector3i(0, 0, -vDeltaIndex[2]));
        Compare(vP, vI, Vector3i(-vDeltaIndex[0], -vDeltaIndex[1], 0));
        Compare(vP, vI, Vector3i(-vDeltaIndex[0], 0, -vDeltaIndex[2]));
        Compare(vP, vI, Vector3i(0, -vDeltaIndex[1], -vDeltaIndex[2]));
        Compare(vP, vI, Vector3i(-vDeltaIndex[0], -vDeltaIndex[1], -vDeltaIndex[2]));
    }
}
