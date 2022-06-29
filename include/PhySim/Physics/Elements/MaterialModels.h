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

#include <PhySim/Utils/Serializer_XML.h>

#include <PhySim/Utils/MathUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

/**
 * IMaterialModel (abstract)
 *
 * Common interface for all FEM material models. It provides methods for
 * computing the energy density and its first derivative w.r.t. the deformation
 * gradient (Piola stress), and its second derivative w.r.t. the deformation
 * gradient.
 */
class IMaterialModel {
 public:
  IMaterialModel() {}
  virtual ~IMaterialModel() {}

  virtual const vector<string>& GetRequiredParameters() const = 0;

  /**
   * Returns the energy density for a given deformation gradient and material
   * parameters.
   */
  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& Fm,
                                    Real& U) const = 0;

  /**
   * Returns the Piola stress for a given deformation gradient and material
   * parameters.
   */
  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& Fm,
                          VectorXd& dUdF) const = 0;

  /**
   * Returns the stress derivative for a given deformation gradient and material
   * parameters.
   */
  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& Fv,
                             MatrixXd& dPdF) const = 0;
};

/**
 * Stabilized Neo-Hookean from used in Pixars
 */
class MaterialModel_3Din3D_SNH : public IMaterialModel {
 public:
  static PtrS<MaterialModel_3Din3D_SNH> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_3Din3D_SNH());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_3Din3D_SNH> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_3Din3D_SNH::m_vrequired;
  }

  static void GetInvariants(const Matrix3r& mF, Real& outIc, Real& outIj) {
    outIc = (mF.transpose() * mF).trace();
    outIj = mF.determinant();
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    const Real& lame_la = mat[ParameterSet::Param_Lame1];
    const Real& lame_mu = mat[ParameterSet::Param_Lame2];
    const Real& alpha = mat[ParameterSet::Param_SNHAlpha];
    Real Ic, Ij;
    GetInvariants(mF, Ic, Ij);

    U = (lame_mu / 2) * (Ic - 3) + (lame_la / 2 * (Ij - alpha) * (Ij - alpha)) -
        (lame_mu / 2 * std::log(Ic + 1));
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    const Matrix3r& mF3 = mF;

    const Real& lame_la = mat[ParameterSet::Param_Lame1];
    const Real& lame_mu = mat[ParameterSet::Param_Lame2];
    const Real& alpha = mat[ParameterSet::Param_SNHAlpha];
    Real Ic, Ij;
    GetInvariants(mF, Ic, Ij);

    // Coefficients for convenience
    Vector<Real, 6> coefs{lame_mu * (1 - 1 / (Ic + 1)),
                          lame_la * (Ij - alpha),
                          lame_mu * (1 - 1 / (Ic + 1)),
                          2 * lame_mu / std::pow(Ic + 1, 2),
                          (lame_la + lame_la * (Ij - alpha) / Ij),
                          lame_la * (Ij - alpha) / Ij};

    // The cofactor of the deformation gradient
    Matrix3r cofF = MathUtils::CofactorMatrix(mF3);
    Matrix3r PK1 = coefs[0] * mF + coefs[1] * cofF;
    MathUtils::Vectorize<Real, 3, 3>(PK1, dUdF);
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    const Matrix3r& mF3 = mF;

    const Real& lame_la = mat[ParameterSet::Param_Lame1];
    const Real& lame_mu = mat[ParameterSet::Param_Lame2];
    const Real& alpha = mat[ParameterSet::Param_SNHAlpha];
    Real Ic, Ij;
    GetInvariants(mF, Ic, Ij);

    // Useful coefficients
    Vector<Real, 6> coefs{lame_mu * (1 - 1 / (Ic + 1)),
                          lame_la * (Ij - alpha),
                          lame_mu * (1 - 1 / (Ic + 1)),
                          2 * lame_mu / std::pow(Ic + 1, 2),
                          (lame_la + lame_la * (Ij - alpha) / Ij),
                          lame_la * (Ij - alpha) / Ij};

    // Cofactor of the deformation gradient
    Matrix3r cofF = MathUtils::CofactorMatrix(mF3);
    MathUtils::Tensor4r<3, 3, 3, 3> dPK1dFtensor;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
          for (int l = 0; l < 3; ++l)
            dPK1dFtensor[i][j](k, l) = coefs[3] * mF(i, j) * mF(k, l) +
                                       coefs[4] * cofF(i, j) * cofF(k, l) -
                                       coefs[5] * cofF(i, l) * cofF(k, j);

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        dPK1dFtensor[i][j](i, j) += coefs[2];

    MathUtils::Vectorize<Real, 3, 3, 3, 3>(dPK1dFtensor, dPdF);
  };
};

/**
 * Saint-Venant Kirchhoff. 3D manifold embedded in 3D space.
 */
class MaterialModel_3Din3D_StVK : public IMaterialModel {
 public:
  static PtrS<MaterialModel_3Din3D_StVK> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_3Din3D_StVK());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_3Din3D_StVK> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_3Din3D_StVK::m_vrequired;
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

#include "../include/PhySim/Utils/Auto/FEMVol_StVK_Energy.mcg"

    U = t58;
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real vgx[9];

#include "../include/PhySim/Utils/Auto/FEMVol_StVK_Gradient.mcg"

    dUdF.resize(9);
    for (int i = 0; i < 9; ++i)
      dUdF(i) = vgx[i];  // Copy
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real mHx[9][9];

#include "../include/PhySim/Utils/Auto/FEMVol_StVK_Hessian.mcg"

    dPdF.resize(9, 9);
    for (int i = 0; i < 9; ++i)
      for (int j = 0; j < 9; ++j)
        dPdF(i, j) = mHx[i][j];  // Copy
  };
};

/**
 * Compresible Neo-Hookean. 3D manifold embedded in 3D space.
 */
class MaterialModel_3Din3D_CoNH : public IMaterialModel {
 public:
  static PtrS<MaterialModel_3Din3D_CoNH> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_3Din3D_CoNH());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_3Din3D_CoNH> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_3Din3D_CoNH::m_vrequired;
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

#include "../include/PhySim/Utils/Auto/FEMVol_CoNH_Energy.mcg"

    U = t31;
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real vgx[9];

#include "../include/PhySim/Utils/Auto/FEMVol_CoNH_Gradient.mcg"

    dUdF.resize(9);
    for (int i = 0; i < 9; ++i)
      dUdF(i) = vgx[i];  // Copy
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real mHx[9][9];

#include "../include/PhySim/Utils/Auto/FEMVol_CoNH_Hessian.mcg"

    dPdF.resize(9, 9);
    for (int i = 0; i < 9; ++i)
      for (int j = 0; j < 9; ++j)
        dPdF(i, j) = mHx[i][j];  // Copy
  };
};

/**
 * Compressible Mooney-Rivlin. 3D manifold embedded in 3D space.
 */
class MaterialModel_3Din3D_CoMR : public IMaterialModel {
 public:
  static PtrS<MaterialModel_3Din3D_CoMR> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_3Din3D_CoMR());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_3Din3D_CoMR> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_3Din3D_CoMR::m_vrequired;
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& bulk = mat[ParameterSet::Param_Bulk];
    const Real& mr01 = mat[ParameterSet::Param_Mooney01];
    const Real& mr10 = mat[ParameterSet::Param_Mooney10];

#include "../include/PhySim/Utils/Auto/FEMVol_CoMR_Energy.mcg"

    U = t61;
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& bulk = mat[ParameterSet::Param_Bulk];
    const Real& mr01 = mat[ParameterSet::Param_Mooney01];
    const Real& mr10 = mat[ParameterSet::Param_Mooney10];

    Real vgx[9];

#include "../include/PhySim/Utils/Auto/FEMVol_CoMR_Gradient.mcg"

    dUdF.resize(9);
    for (int i = 0; i < 9; ++i)
      dUdF(i) = vgx[i];  // Copy
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& bulk = mat[ParameterSet::Param_Bulk];
    const Real& mr01 = mat[ParameterSet::Param_Mooney01];
    const Real& mr10 = mat[ParameterSet::Param_Mooney10];

    Real mHx[9][9];

#include "../include/PhySim/Utils/Auto/FEMVol_CoMR_Hessian.mcg"

    dPdF.resize(9, 9);
    for (int i = 0; i < 9; ++i)
      for (int j = 0; j < 9; ++j)
        dPdF(i, j) = mHx[i][j];  // Copy
  };
};

/**
 * Saint-Venant Kirchhoff. 2D manifold embedded in 3D space.
 */
class MaterialModel_2Din3D_StVK : public IMaterialModel {
 public:
  static PtrS<MaterialModel_2Din3D_StVK> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_2Din3D_StVK());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_2Din3D_StVK> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_2Din3D_StVK::m_vrequired;
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

#include "../include/PhySim/Utils/Auto/FEMMem_StVK_Energy.mcg"

    U = t32;
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real vgx[6];

#include "../include/PhySim/Utils/Auto/FEMMem_StVK_Gradient.mcg"

    dUdF.resize(6);
    for (int i = 0; i < 6; ++i)
      dUdF(i) = vgx[i];  // Copy
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real mHx[6][6];

#include "../include/PhySim/Utils/Auto/FEMMem_StVK_Hessian.mcg"

    dPdF.resize(6, 6);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        dPdF(i, j) = mHx[i][j];  // Copy
  };
};

/**
 * Compressible Neo-Hookean. 2D manifold embedded in 3D space.
 */
class MaterialModel_2Din3D_CoNH : public IMaterialModel {
 public:
  static PtrS<MaterialModel_2Din3D_CoNH> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_2Din3D_CoNH());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_2Din3D_CoNH> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_2Din3D_CoNH::m_vrequired;
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

#include "../include/PhySim/Utils/Auto/FEMMem_CoNH_Energy.mcg"

    U = t32;
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real vgx[6];

#include "../include/PhySim/Utils/Auto/FEMMem_CoNH_Gradient.mcg"

    dUdF.resize(6);
    for (int i = 0; i < 6; ++i)
      dUdF(i) = vgx[i];  // Copy
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real mHx[6][6];

#include "../include/PhySim/Utils/Auto/FEMMem_CoNH_Hessian.mcg"

    dPdF.resize(6, 6);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        dPdF(i, j) = mHx[i][j];  // Copy
  };
};

/**
 * Incompressible Neo-Hookean. 2D manifold embedded in 3D space.
 */
class MaterialModel_2Din3D_InNH : public IMaterialModel {
 public:
  static PtrS<MaterialModel_2Din3D_InNH> Instance() {
    if (PINSTANCE == NULL)
      PINSTANCE.reset(new MaterialModel_2Din3D_InNH());
    return PINSTANCE;
  }

 private:
  static vector<string> m_vrequired;
  static PtrS<MaterialModel_2Din3D_InNH> PINSTANCE;

 public:
  virtual const vector<string>& GetRequiredParameters() const {
    return MaterialModel_2Din3D_InNH::m_vrequired;
  }

  virtual void ComputeEnergyDensity(const ParameterSet& mat,
                                    const MatrixXd& mF,
                                    Real& U) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

#include "../include/PhySim/Utils/Auto/FEMMem_InNH_Energy.mcg"

    U = t26;
  };

  virtual void ComputePK1(const ParameterSet& mat,
                          const MatrixXd& mF,
                          VectorXd& dUdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real vgx[6];

#include "../include/PhySim/Utils/Auto/FEMMem_InNH_Gradient.mcg"

    dUdF.resize(6);
    for (int i = 0; i < 6; ++i)
      dUdF(i) = vgx[i];  // Copy
  };

  virtual void ComputeDPK1DF(const ParameterSet& mat,
                             const MatrixXd& mF,
                             MatrixXd& dPdF) const override {
    VectorXd Fv;
    MathUtils::Vectorize<Real, 3, 3>(mF, Fv);
    const Real& lame1 = mat[ParameterSet::Param_Lame1];
    const Real& lame2 = mat[ParameterSet::Param_Lame2];

    Real mHx[6][6];

#include "../include/PhySim/Utils/Auto/FEMMem_InNH_Hessian.mcg"

    dPdF.resize(6, 6);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        dPdF(i, j) = mHx[i][j];  // Copy
  };
};
}  // namespace PhySim
