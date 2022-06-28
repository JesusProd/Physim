//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/EnergyUtils.h>
#include <PhySim/Utils/ParameterSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;
namespace Energies {

Matrix2d computeStressForE_2DStVK(const Matrix2d& mE,
                                  const ParameterSet& material) {
  Real lame1 = material[ParameterSet::Param_Lame1];
  Real lame2 = material[ParameterSet::Param_Lame2];
  return lame1 * mE.trace() * Matrix2d::Identity() + 2 * lame2 * mE;
}

Matrix2d computeStressForE_2DCoNH(const Matrix2d& mE,
                                  const ParameterSet& material) {
  // TODO
  return Matrix2d();
}

Real computeEnergyDensity_3Din3D_StVK(const VectorXd& Fv,
                                      const ParameterSet& material) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

#include "../Maple/FEMVol_StVK_Energy.mcg"

  return t58;
}

void computeCauchyStress_3Din3D_StVK(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real vgx[9];

#include "../Maple/FEMVol_StVK_Gradient.mcg"

  dUdF.resize(9);
  for (int i = 0; i < 9; ++i)
    dUdF(i) = vgx[i];  // Copy
}

void computeDCauchyStressDF_3Din3D_StVK(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real mHx[9][9];

#include "../Maple/FEMVol_StVK_Hessian.mcg"

  dPdF.resize(9, 9);
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j)
      dPdF(i, j) = mHx[i][j];  // Copy
}

Real computeEnergyDensity_3Din3D_CoNH(const VectorXd& Fv,
                                      const ParameterSet& material) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

#include "../Maple/FEMVol_CoNH_Energy.mcg"

  return t31;

  // Matrix3d F;
  // F.col(0) = Fv.segment(0, 3);
  // F.col(1) = Fv.segment(3, 3);
  // F.col(2) = Fv.segment(6, 3);

  // Real Ic = F.squaredNorm();
  // Real alpha = 1.0 + 0.75 * lame1 / lame2;
  // Real JminusAlpha = F.determinant() - alpha;
  // Real PsiTwice = lame1 * (Ic - 3.0 - log(Ic + 1.0)) + lame2 * JminusAlpha *
  // JminusAlpha;

  // return 0.5 * PsiTwice;
}

void computeCauchyStress_3Din3D_CoNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real vgx[9];

#include "../Maple/FEMVol_CoNH_Gradient.mcg"

  dUdF.resize(9);
  for (int i = 0; i < 9; ++i)
    dUdF(i) = vgx[i];  // Copy

  // Matrix3d F;
  // F.col(0) = Fv.segment(0, 3);
  // F.col(1) = Fv.segment(3, 3);
  // F.col(2) = Fv.segment(6, 3);

  // Real Ic = F.squaredNorm();
  // Real alpha = 1.0 + 0.75 * lame1 / lame2;
  // Real JminusAlpha = F.determinant() - alpha;

  // Matrix3d dJdF;

  // dJdF.col(0) = F.col(1).cross(F.col(2));
  // dJdF.col(1) = F.col(2).cross(F.col(0));
  // dJdF.col(2) = F.col(0).cross(F.col(1));

  // Matrix3d P = lame1 * (1.0 - 1.0 / (Ic + 1.0)) * F + lame2 * JminusAlpha *
  // dJdF;

  // dUdF.resize(9);
  // dUdF.segment(0, 3) = P.col(0);
  // dUdF.segment(3, 3) = P.col(1);
  // dUdF.segment(6, 3) = P.col(2);
}

void computeDCauchyStressDF_3Din3D_CoNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real mHx[9][9];

#include "../Maple/FEMVol_CoNH_Hessian.mcg"

  dPdF.resize(9, 9);
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j)
      dPdF(i, j) = mHx[i][j];  // Copy

  // Matrix3d F;
  // F.col(0) = Fv.segment(0, 3);
  // F.col(1) = Fv.segment(3, 3);
  // F.col(2) = Fv.segment(6, 3);

  // Real Ic = F.squaredNorm();
  // Real alpha = 1.0 + 0.75 * lame1 / lame2;
  // Real JminusAlpha = F.determinant() - alpha;

  // VectorXd f(9), g(9);
  // MatrixXd pg_pf(9, 9);

  // f << F.col(0), F.col(1), F.col(2);

  // g << F.col(1).cross(F.col(2)),
  //	F.col(2).cross(F.col(0)),
  //	F.col(0).cross(F.col(1));

  // Matrix3d m0, m1, m2;
  // crossMatrix(F.col(0), m0);
  // crossMatrix(F.col(1), m1);
  // crossMatrix(F.col(2), m2);

  // pg_pf << Matrix3d::Zero(), -m2, m1,
  //	m2, Matrix3d::Zero(), -m0,
  //	-m1, m0, Matrix3d::Zero();

  // dPdF = MatrixXd::Identity(9, 9) * lame1 * (1.0 - 1.0 / (Ic + 1.0)) +
  //	f * f.transpose() * 2.0 * lame1 / ((Ic + 1.0)*(Ic + 1.0)) +
  //	g * g.transpose() * lame2 +
  //	pg_pf * lame2 * JminusAlpha;
}

Real computeEnergyDensity_3Din3D_CoMR(const VectorXd& Fv,
                                      const ParameterSet& material) {
  const Real& bulk = material[ParameterSet::Param_Bulk];
  const Real& mr01 = material[ParameterSet::Param_Mooney01];
  const Real& mr10 = material[ParameterSet::Param_Mooney10];

#include "../Maple/FEMVol_CoMR_Energy.mcg"

  return t61;
}

void computeCauchyStress_3Din3D_CoMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  const Real& bulk = material[ParameterSet::Param_Bulk];
  const Real& mr01 = material[ParameterSet::Param_Mooney01];
  const Real& mr10 = material[ParameterSet::Param_Mooney10];

  Real vgx[9];

#include "../Maple/FEMVol_CoMR_Gradient.mcg"

  dUdF.resize(9);
  for (int i = 0; i < 9; ++i)
    dUdF(i) = vgx[i];  // Copy
}

void computeDCauchyStressDF_3Din3D_CoMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  const Real& bulk = material[ParameterSet::Param_Bulk];
  const Real& mr01 = material[ParameterSet::Param_Mooney01];
  const Real& mr10 = material[ParameterSet::Param_Mooney10];

  Real mHx[9][9];

#include "../Maple/FEMVol_CoMR_Hessian.mcg"

  dPdF.resize(9, 9);
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j)
      dPdF(i, j) = mHx[i][j];  // Copy
}

Real computeEnergyDensity_3Din3D_InNH(const VectorXd& Fv,
                                      const ParameterSet& material) {
  throw exception("Not implemented");

  return 0;
}

Real computeEnergyDensity_3Din3D_InMR(const VectorXd& Fv,
                                      const ParameterSet& material) {
  throw exception("Not implemented");

  return 0;
}

void computeCauchyStress_3Din3D_InNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  throw exception("Not implemented");
}

void computeCauchyStress_3Din3D_InMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  throw exception("Not implemented");
}

void computeDCauchyStressDF_3Din3D_InNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  throw exception("Not implemented");
}

void computeDCauchyStressDF_3Din3D_InMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  throw exception("Not implemented");
}

Real computeEnergyDensity_2Din3D_StVK(const VectorXd& Fv,
                                      const ParameterSet& material) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  // const Real& Y = material[ParameterSet::Param_Young];
  // const Real& v = material[ParameterSet::Param_Poisson];

#include "../Maple/FEMMem_StVK_Energy.mcg"

  return t32;
}

void computeCauchyStress_2Din3D_StVK(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  // const Real& Y = material[ParameterSet::Param_Young];
  // const Real& v = material[ParameterSet::Param_Poisson];

  Real vgx[6];

#include "../Maple/FEMMem_StVK_Gradient.mcg"

  dUdF.resize(6);
  for (int i = 0; i < 6; ++i)
    dUdF(i) = vgx[i];  // Copy
}

void computeDCauchyStressDF_2Din3D_StVK(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  // const Real& Y = material[ParameterSet::Param_Young];
  // const Real& v = material[ParameterSet::Param_Poisson];

  Real mHx[6][6];

#include "../Maple/FEMMem_StVK_Hessian.mcg"

  dPdF.resize(6, 6);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      dPdF(i, j) = mHx[i][j];  // Copy
}

Real computeEnergyDensity_2Din3D_CoNH(const VectorXd& Fv,
                                      const ParameterSet& material) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

#include "../Maple/FEMMem_CoNH_Energy.mcg"

  return t32;
}

void computeCauchyStress_2Din3D_CoNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real vgx[6];

#include "../Maple/FEMMem_CoNH_Gradient.mcg"

  dUdF.resize(6);
  for (int i = 0; i < 6; ++i)
    dUdF(i) = vgx[i];  // Copy
}

void computeDCauchyStressDF_2Din3D_CoNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real mHx[6][6];

#include "../Maple/FEMMem_CoNH_Hessian.mcg"

  dPdF.resize(6, 6);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      dPdF(i, j) = mHx[i][j];  // Copy
}

Real computeEnergyDensity_2Din3D_InNH(const VectorXd& Fv,
                                      const ParameterSet& material) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

#include "../Maple/FEMMem_InNH_Energy.mcg"

  return t26;
}

void computeCauchyStress_2Din3D_InNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real vgx[6];

#include "../Maple/FEMMem_InNH_Gradient.mcg"

  dUdF.resize(6);
  for (int i = 0; i < 6; ++i)
    dUdF(i) = vgx[i];  // Copy
}

void computeDCauchyStressDF_2Din3D_InNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  const Real& lame1 = material[ParameterSet::Param_Lame1];
  const Real& lame2 = material[ParameterSet::Param_Lame2];

  Real mHx[6][6];

#include "../Maple/FEMMem_InNH_Hessian.mcg"

  dPdF.resize(6, 6);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      dPdF(i, j) = mHx[i][j];  // Copy
}

Real computeEnergyDensity_2Din3D_CoMR(const VectorXd& Fv,
                                      const ParameterSet& material) {
  throw exception("Not implemented");

  return 0;
}

void computeCauchyStress_2Din3D_CoMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  throw exception("Not implemented");
}

void computeDCauchyStressDF_2Din3D_CoMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  throw exception("Not implemented");
}

Real computeEnergyDensity_2Din3D_InMR(const VectorXd& Fv,
                                      const ParameterSet& material) {
  throw exception("Not implemented");

  return 0;
}

void computeCauchyStress_2Din3D_InMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF) {
  throw exception("Not implemented");
}

void computeDCauchyStressDF_2Din3D_InMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF) {
  throw exception("Not implemented");
}

}  // namespace Energies
}  // namespace PhySim
