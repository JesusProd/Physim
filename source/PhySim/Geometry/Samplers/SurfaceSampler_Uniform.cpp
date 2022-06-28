#include <PhySim/Geometry/Samplers/SurfaceSampler_Uniform.h>
using namespace PhySim;

SurfaceSampler_Uniform::SurfaceSampler_Uniform()
    : SurfaceSampler()
    , m_UProbDistribution(0.0, 1.0)
{
}

void SurfaceSampler_Uniform::Setup(const MatrixXd& mX, const MatrixXi& mF)
{
    SurfaceSampler::Setup(mX, mF);
    m_FProb = m_vA / m_totA;
    //m_FProbDistribution = discrete_distribution<size_t>(m_FProb.begin(), m_FProb.end());
}

void SurfaceSampler_Uniform::Sample(size_t M, vector<SurfaceSample>& vOutput) const
{
    // Allocate enough space for newly generated samples.
    vOutput.reserve(vOutput.size() + M);

    // Perform uniform distribution sampling over the input surface.
    for (int i = 0; i < M; ++i)
    {
        // Pick a triangle with probability proportial to its surface area.
        size_t t = m_FProbDistribution(m_engine);

        // Compute uniform distribution over [0, 1]x[0, 1] lower triangle
        double u = m_UProbDistribution(m_engine);
        double v = m_UProbDistribution(m_engine);
        double su0 = sqrt(u);
        double b0 = 1.0 - su0;
        double b1 = v * su0;
        double b2 = 1.0 - b0 - b1;

        // Compute point.
        const Vector3d vXa = m_mX.row(m_mF(t, 0)).transpose();
        const Vector3d vXb = m_mX.row(m_mF(t, 1)).transpose();
        const Vector3d vXc = m_mX.row(m_mF(t, 2)).transpose();
        const Vector3d vP = b0 * vXa + b1 * vXb + b2 * vXc;

        // Store sample.
        SurfaceSample sample;
        sample.Triangle = t;
        sample.vPosition = vP;
        sample.vBarCoords = Vector3d(b0, b1, b2);
        vOutput.push_back(sample);
    }
}
