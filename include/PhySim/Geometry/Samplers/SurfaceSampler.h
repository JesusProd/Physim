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

#include <random>

namespace PhySim {
struct SurfaceSample {
  size_t Triangle;
  Vector3d vBarCoords;
  Vector3d vPosition;
};

class SurfaceSampler {
 public:
  virtual ~SurfaceSampler() = default;

 public:
  virtual void Setup(const MatrixXd& mX, const MatrixXi& mF);
  virtual void Sample(size_t M, vector<SurfaceSample>& vOutput) const = 0;
  virtual void Seed(default_random_engine::result_type seed);
  void Seed();

 public:
  const MatrixXd& Positions() const;
  const MatrixXi& Faces() const;
  const VectorXd& Areas() const;
  const double TotalArea() const;

 protected:
  MatrixXd m_mX;
  MatrixXi m_mF;
  VectorXd m_vA;
  double m_totA;
  mutable random_device m_device;
  mutable default_random_engine m_engine;
};

class SurfaceSamplingStatistics {
 public:
  SurfaceSamplingStatistics() = default;
  SurfaceSamplingStatistics(const vector<SurfaceSample>& vSamples);

 public:
  void Setup(const vector<SurfaceSample>& vSamples);

 public:
  const MatrixXd& Points() const;
  const VectorXd& Distances() const;

  Vector3d MinPoint() const;
  Vector3d MaxPoint() const;
  Vector3d MeanPoint() const;

  Real MinDistance() const;
  Real MaxDistance() const;
  Real MeanDistance() const;

 private:
  MatrixXd m_mP;
  VectorXd m_vD;
};
}  // namespace PhySim