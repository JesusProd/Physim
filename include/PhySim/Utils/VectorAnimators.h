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

#include <PhySim/Utils/GeometryUtils.h>
#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class IVectorAnimator {
 protected:
  vector<VectorXd> m_vini;

 public:
  virtual string Name() const = 0;

  virtual int NumPoints() const = 0;

  virtual void SetInitial(const vector<VectorXd>& vval) { this->m_vini = vval; }
  virtual const vector<VectorXd>& GetInitial() const { return this->m_vini; }

  virtual void Evaluate(Real t, vector<VectorXd>& vval) const = 0;
};

class VectorAnimator_LerpValues : public IVectorAnimator {
 public:
  static const string NAME;

  vector<VectorXd> m_vvalIni;
  vector<VectorXd> m_vvalEnd;

  VectorAnimator_LerpValues(const vector<VectorXd>& vvalIni,
                            const vector<VectorXd>& vvalEnd) {
    // Check

    assert(vvalIni.size() == vvalEnd.size());

    for (int i = 0; i < (int)vvalIni.size(); ++i)
      assert(vvalIni[i].size() == vvalEnd[i].size());

    this->m_vvalIni = vvalIni;
    this->m_vvalEnd = vvalEnd;
  }

  virtual string Name() const override {
    return VectorAnimator_LerpValues::NAME;
  }

  virtual int NumPoints() const override { return (int)m_vvalIni.size(); }

  virtual void Evaluate(Real t, vector<VectorXd>& vval) const override {
    int numP = (int)this->m_vvalIni.size();

    vval.resize(numP);
    for (int i = 0; i < numP; ++i)
      vval[i] = m_vvalIni[i] + (m_vvalEnd[i] - m_vvalIni[i]) * t;
  }
};

class VectorAnimator_LerpT2D : public IVectorAnimator {
 public:
  static const string NAME;

  Vector2d m_vT;
  Vector2d m_vR;
  Vector2d m_vS;

  VectorAnimator_LerpT2D(const Vector2d& vT,
                         const Vector2d& vR,
                         const Vector2d& vS,
                         const vector<VectorXd>& vvalIni = vector<VectorXd>()) {
    this->m_vT = vT;
    this->m_vR = vR;
    this->m_vS = vS;

    this->m_vini = vvalIni;
  }

  virtual string Name() const override { return VectorAnimator_LerpT2D::NAME; }

  virtual int NumPoints() const override { return (int)m_vini.size(); }

  virtual void Evaluate(Real t, vector<VectorXd>& vval) const override {
    int numP = (int)this->m_vini.size();

    // TODO: implement 2D vector transform

    vval = m_vini;
  }
};

class VectorAnimator_LerpT3D : public IVectorAnimator {
 public:
  static const string NAME;

  Matrix4d m_mT;

  VectorAnimator_LerpT3D(const Vector3d& vT,
                         const Vector3d& vR,
                         const Vector3d& vS,
                         const vector<VectorXd>& vvalIni = vector<VectorXd>()) {
    Matrix3d mS = Matrix3d::Identity();
    mS(0, 0) = vS(0);
    mS(1, 1) = vS(1);
    mS(2, 2) = vS(2);
    Matrix3d mR = GeometryUtils::rotationEulerToMatrix(vR);
    this->m_mT = Matrix4d::Identity();
    this->m_mT.block(0, 3, 3, 1) = vT;
    this->m_mT.block(0, 0, 3, 3) = mR * mS;
    this->m_vini = vvalIni;
  }

  VectorAnimator_LerpT3D(const Matrix4d& mT,
                         const vector<VectorXd>& vvalIni = vector<VectorXd>()) {
    this->m_mT = mT - Matrix4d::Identity();

    this->m_vini = vvalIni;
  }

  virtual string Name() const override { return VectorAnimator_LerpT3D::NAME; }

  virtual int NumPoints() const override { return (int)m_vini.size(); }

  virtual void Evaluate(Real t, vector<VectorXd>& vval) const override {
    int numP = (int)this->m_vini.size();

    vval.resize(numP);

    Matrix4d mTlerp = Matrix4d::Identity() + this->m_mT * t;

    for (int i = 0; i < numP; ++i)
      vval[i] = mTlerp.block(0, 3, 3, 1) + mTlerp.block(0, 0, 3, 3) * m_vini[i];
  }
};

class VectorAnimator_SequenceT2D : public IVectorAnimator {
 public:
  static const string NAME;

  vector<Matrix3d> m_vmT;
  vector<double> m_times;

  VectorAnimator_SequenceT2D(const vector<Matrix3d>& vmT,
                             const vector<int>& vSteps,
                             const vector<VectorXd>& vvalIni) {
    m_vmT.resize(vmT.size());
    for (int i = 0; i < (int)vmT.size(); ++i)
      this->m_vmT[i] = vmT[i] - Matrix3d::Identity();

    int sumSteps = 0;
    this->m_times.resize(vSteps.size(), 0);
    for (int i = 0; i < (int)vSteps.size(); ++i) {
      sumSteps += (int)vSteps.size();
      this->m_times[i] = sumSteps;
    }
    for (int i = 0; i < (int)vSteps.size(); ++i)
      this->m_times[i] = this->m_times[i] / sumSteps;

    this->m_vini = vvalIni;
  }

  virtual string Name() const override {
    return VectorAnimator_SequenceT2D::NAME;
  }

  virtual int NumPoints() const override { return (int)m_vini.size(); }

  virtual void Evaluate(Real t, vector<VectorXd>& vval) const override {
    int numP = (int)this->m_vini.size();

    vval = m_vini;

    for (int i = 0; i < (int)this->m_vmT[i].size(); ++i) {
      if (t <= this->m_times[i]) {
        Matrix3d mTlerp = Matrix3d::Identity() + this->m_vmT[i] * t;
        for (int i = 0; i < numP; ++i)
          vval[i] =
              mTlerp.block(0, 3, 3, 1) + mTlerp.block(0, 0, 3, 3) * vval[i];

        break;
      } else {
        Matrix3d mTlerp = Matrix3d::Identity() + this->m_vmT[i];
        for (int i = 0; i < numP; ++i)
          vval[i] =
              mTlerp.block(0, 3, 3, 1) + mTlerp.block(0, 0, 3, 3) * vval[i];
      }
    }
  }
};

class VectorAnimator_SequenceT3D : public IVectorAnimator {
 public:
  static const string NAME;

  vector<Vector3d> m_vT;
  vector<Vector3d> m_vQ;
  vector<double> m_times;

  VectorAnimator_SequenceT3D(const vector<Matrix4d>& vmT,
                             const vector<int>& vSteps,
                             const vector<VectorXd>& vvalIni) {
    IOUtils::logTrace(Verbosity::V2_SoftDebug,
                      "\n[TRACE] Initializing Animator: T3D sequence (%d,%d)",
                      vmT.size(), vSteps.size());

    m_vT.resize(vmT.size());
    m_vQ.resize(vmT.size());
    for (int i = 0; i < (int)vmT.size(); ++i) {
      this->m_vT[i] = vmT[i].block(0, 3, 3, 1);
      Matrix3d mR = vmT[i].block(0, 0, 3, 3);
      this->m_vQ[i] = GeometryUtils::rotationMatrixToAxisAngle(mR);
      IOUtils::logTrace(Verbosity::V3_HardDebug,
                        "\n[TRACE] Translation: (%f,%f,%f)", m_vT[i](0),
                        m_vT[i](1), m_vT[i](2));
      IOUtils::logTrace(Verbosity::V3_HardDebug,
                        "\n[TRACE] Rotation: (%f,%f,%f)", m_vQ[i](0),
                        m_vQ[i](1), m_vQ[i](2));

      Matrix3d mT = mR - GeometryUtils::rotationAxisAngleToMatrix(m_vQ[i]);
      if ((mT).norm() > 1e-6) {
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[WARNING] Rotation Axis-Angle does "
                          "not match input matrix");
        ostringstream str;
        for (int k = 0; k < 3; ++k) {
          str << "\n";
          for (int l = 0; l < 3; ++l)
            str << mT(k, l) << " ";
        }
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[WARNING] Animator rotation "
                          "difference: %s",
                          str.str().c_str());
      }
    }

    int sumSteps = 0;
    this->m_times.resize((int)vSteps.size(), 0);
    for (int i = 0; i < (int)vSteps.size(); ++i) {
      sumSteps += (int)vSteps[i];
      this->m_times[i] = sumSteps;

      IOUtils::logTrace(Verbosity::V3_HardDebug, "\n[TRACE] Step number: %d",
                        vSteps[i]);
    }
    for (int i = 0; i < (int)vSteps.size(); ++i)
      this->m_times[i] = this->m_times[i] / sumSteps;

    this->m_vini = vvalIni;
  }

  virtual string Name() const override {
    return VectorAnimator_SequenceT3D::NAME;
  }

  virtual int NumPoints() const override { return (int)m_vini.size(); }

  virtual void Evaluate(Real t, vector<VectorXd>& vval) const override {
    int numP = (int)this->m_vini.size();

    vval = m_vini;

    IOUtils::logTrace(Verbosity::V2_SoftDebug,
                      "\n[TRACE] Evaluating animator for t = %f", t);

    for (int i = 0; i < (int)this->m_vT.size(); ++i) {
      IOUtils::logTrace(Verbosity::V3_HardDebug,
                        "\n[TRACE] Transformation of the sequence: %d", i);

      if (t < this->m_times[i]) {
        double ti;
        if (i == 0)
          ti = t / this->m_times[i];
        else
          ti = (t - this->m_times[i - 1]) /
               (this->m_times[i] - this->m_times[i - 1]);

        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] In progress (%f < %f), "
                          "ti = %f",
                          t, m_times[i], ti);

        Vector3d vt = this->m_vT[i] * ti;
        Vector3d vq = this->m_vQ[i] * ti;
        Matrix3d mR = GeometryUtils::rotationAxisAngleToMatrix(vq);
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Animator translation: "
                          "(%f,%f,%f)",
                          vt(0), vt(1), vt(2));
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Animator rotation: "
                          "(%f,%f,%f)",
                          vq(0), vq(1), vq(2));
        for (int j = 0; j < numP; ++j)
          vval[j] = vt + mR * vval[j];

        Matrix3d mA = GeometryUtils::rotationAxisAngleToMatrix(vq);
        ostringstream str;
        for (int k = 0; k < 3; ++k) {
          str << "\n";
          for (int l = 0; l < 3; ++l)
            str << mA(k, l) << " ";
        }
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Animator rotation: %s", str.str().c_str());

        break;
      } else {
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Completed (%f >= %f), ti = %f", t,
                          m_times[i], 1.0);

        Vector3d vt = this->m_vT[i];
        Vector3d vq = this->m_vQ[i];
        Matrix3d mR = GeometryUtils::rotationAxisAngleToMatrix(vq);
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Animator translation: "
                          "(%f,%f,%f)",
                          vt(0), vt(1), vt(2));
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Animator rotation: "
                          "(%f,%f,%f)",
                          vq(0), vq(1), vq(2));
        for (int j = 0; j < numP; ++j)
          vval[j] = vt + mR * vval[j];

        Matrix3d mA = GeometryUtils::rotationAxisAngleToMatrix(vq);
        ostringstream str;
        for (int k = 0; k < 3; ++k) {
          str << "\n";
          for (int l = 0; l < 3; ++l)
            str << mA(k, l) << " ";
        }
        IOUtils::logTrace(Verbosity::V3_HardDebug,
                          "\n[TRACE] Animator rotation: %s", str.str().c_str());
      }
    }
  }
};

}  // namespace PhySim