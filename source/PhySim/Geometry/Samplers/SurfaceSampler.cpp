#include <PhySim/Geometry/Samplers/SurfaceSampler.h>
#include <PhySim/Geometry/Partition/KDTree.h>
#include <numeric>
using namespace PhySim;

void SurfaceSampler::Setup(const MatrixXd& mX, const MatrixXi& mF)
{
    const int numVerts = int(mX.rows());
    const int numFaces = int(mF.rows());

    // Store mesh vertices and topology.
    m_mX = mX;
    m_mF = mF;

    // Compute per-triangle area.
    m_vA.setZero(mF.rows());

    #pragma omp parallel for
    for (int i = 0; i < numFaces; ++i)
    {
        const Vector3i vF = mF.row(i).transpose();
        const Vector3d vXa = mX.row(vF[0]).transpose();
        const Vector3d vXb = mX.row(vF[1]).transpose();
        const Vector3d vXc = mX.row(vF[2]).transpose();
        const double A = 0.5 * (vXb - vXa).cross(vXc - vXa).norm();
        m_vA[i] = A;
    }

    // Compute total area.
    //m_totA = accumulate(m_vA.begin(), m_vA.end(), 0.0);
}

void SurfaceSampler::Seed()
{
    Seed(m_device());
}

void SurfaceSampler::Seed(default_random_engine::result_type seed)
{
    m_engine.seed(seed);
}


const MatrixXd& SurfaceSampler::Positions() const
{
    return m_mX;
}

const MatrixXi& SurfaceSampler::Faces() const
{
    return m_mF;
}

const VectorXd& SurfaceSampler::Areas() const
{
    return m_vA;
}

const double SurfaceSampler::TotalArea() const
{
    return m_totA;
}

SurfaceSamplingStatistics::SurfaceSamplingStatistics(const vector<SurfaceSample>& vSamples)
{
    Setup(vSamples);
}

void SurfaceSamplingStatistics::Setup(const vector<SurfaceSample>& vSamples)
{
    //const size_t N = vSamples.size();

    //// Store samples in matrix format.
    //m_mP.resize(N, 3);
    //
    //for (size_t i = 0; i < N; ++i)
    //    m_mP.row(i) = vSamples[i].vPosition.transpose();

    //// Store distances to closest neighbor for all samples.
    //m_vD.resize(N);
    //m_vD.setConstant(numeric_limits<double>::infinity());

    //KDTree tree;
    //KDPointCloudObject obj(m_mP);
    //KDTree::QueryResults res;
    //tree.Build(&obj, {});

    //for (int i = 0; i < N; ++i)
    //{
    //    tree.FindKNN(m_mP.row(i).transpose(), res, 2); // 2 neighbors, as the first one will be the point itself.
    //    m_vD[i] = sqrt(res[1].second);
    //}
}

const MatrixXd& SurfaceSamplingStatistics::Points() const
{
    return m_mP;
}

const VectorXd& SurfaceSamplingStatistics::Distances() const
{
    return m_vD;
}

Vector3d SurfaceSamplingStatistics::MinPoint() const
{
    return m_mP.colwise().minCoeff().transpose();
}

Vector3d SurfaceSamplingStatistics::MaxPoint() const
{
    return m_mP.colwise().maxCoeff().transpose();
}

Vector3d SurfaceSamplingStatistics::MeanPoint() const
{
    return m_mP.colwise().mean().transpose();
}

Real SurfaceSamplingStatistics::MinDistance() const
{
    return m_vD.minCoeff();
}

Real SurfaceSamplingStatistics::MaxDistance() const
{
    return m_vD.maxCoeff();
}

Real SurfaceSamplingStatistics::MeanDistance() const
{
    return m_vD.mean();
}
