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

namespace PhySim {
using namespace std;
using namespace Eigen;

struct IDeformer {
 public:
  IDeformer() {}
  virtual ~IDeformer() {}
  virtual VectorXd Interpolate(const VectorXd& vx) const = 0;
};

struct Deformer_Linear : public IDeformer {
 protected:
  MatrixXd m_mA;

 public:
  Deformer_Linear(const MatrixXd& mA) { this->m_mA = mA; }

  virtual VectorXd Interpolate(const VectorXd& vx) const override {
    return this->m_mA * vx;
  }
};

struct Deformer_SineXY : public IDeformer {
 protected:
  Real m_amplitude;
  Real m_frequency;

 public:
  Deformer_SineXY(Real amplitude, Real frequency) {
    this->m_amplitude = amplitude;
    this->m_frequency = frequency;
  }

  virtual VectorXd Interpolate(const VectorXd& vx) const override {
    VectorXd vxT = vx;
    vxT.y() += m_amplitude * sin(vxT.x() * m_frequency);
    return vxT;
  }
};

struct Deformer_CosineXY : public IDeformer {
 protected:
  Real m_amplitude;
  Real m_frequency;

 public:
  Deformer_CosineXY(Real amplitude, Real frequency) {
    this->m_amplitude = amplitude;
    this->m_frequency = frequency;
  }

  virtual VectorXd Interpolate(const VectorXd& vx) const override {
    VectorXd vxT = vx;
    vxT.y() += m_amplitude * cos(vxT.x() * m_frequency);
    return vxT;
  }
};

}  // namespace PhySim