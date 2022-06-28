//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

// Blue noise sampler based on Marmakoide's implementation of
// Cem Yuksel's paper "Sample Elimination for Generating Poisson
// Disk Sample Sets".
// https://github.com/marmakoide/mesh-blue-noise-sampling

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Samplers/SurfaceSampler.h>
#include <PhySim/Geometry/Samplers/SurfaceSampler_Uniform.h>
#include <PhySim/Geometry/Partition/KDTree.h>

namespace PhySim
{
    class SurfaceSampler_Poisson : public SurfaceSampler
    {
    public:
        SurfaceSampler_Poisson();

    public:
        void Setup(const MatrixXd& mX, const MatrixXi& mF) override;
        void Parameters(size_t ratio, double alpha, double beta, double gamma);
        void Sample(size_t M, std::vector<SurfaceSample>& vOutput) const override;
        void Seed(default_random_engine::result_type seed) override;

    private:
        void Elimination(size_t M, vector<SurfaceSample>& vInput, vector<SurfaceSample>& vOutput) const;

    private:
        size_t m_ratio;
        double m_alpha;
        double m_beta;
        double m_gamma;
        SurfaceSampler_Uniform m_USurfaceSampler;
    };
}