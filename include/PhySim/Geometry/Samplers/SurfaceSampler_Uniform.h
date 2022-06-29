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

#include <PhySim/Geometry/Samplers/SurfaceSampler.h>

namespace PhySim {
class SurfaceSampler_Uniform : public SurfaceSampler {
 public:
  SurfaceSampler_Uniform();

 public:
  void Setup(const MatrixXd& X, const MatrixXi& F);
  void Sample(size_t M, vector<SurfaceSample>& vOutput) const override;

 private:
  VectorXd m_FProb;
  discrete_distribution<size_t> m_FProbDistribution;
  uniform_real_distribution<double> m_UProbDistribution;
};
}  // namespace PhySim