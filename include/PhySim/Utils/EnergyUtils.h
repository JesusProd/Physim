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

#include <PhySim/BasicTypes.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Simulable;
class ParameterSet;

namespace Energies {

Matrix2d computeStressForE_2DStVK(const Matrix2d& mE,
                                  const ParameterSet& material);

Matrix2d computeStressForE_2DCoNH(const Matrix2d& mE,
                                  const ParameterSet& material);

Real computeEnergyDensity_3Din3D_StVK(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_3Din3D_CoNH(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_3Din3D_InNH(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_3Din3D_CoMR(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_3Din3D_InMR(const VectorXd& Fv,
                                      const ParameterSet& material);

void computeCauchyStress_3Din3D_StVK(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_3Din3D_CoNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_3Din3D_InNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_3Din3D_CoMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_3Din3D_InMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);

void computeDCauchyStressDF_3Din3D_StVK(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_3Din3D_CoNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_3Din3D_InNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_3Din3D_CoMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_3Din3D_InMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);

Real computeEnergyDensity_2Din3D_StVK(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_2Din3D_CoNH(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_2Din3D_InNH(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_2Din3D_CoMR(const VectorXd& Fv,
                                      const ParameterSet& material);
Real computeEnergyDensity_2Din3D_InMR(const VectorXd& Fv,
                                      const ParameterSet& material);

void computeCauchyStress_2Din3D_StVK(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_2Din3D_CoNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_2Din3D_InNH(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_2Din3D_CoMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);
void computeCauchyStress_2Din3D_InMR(const VectorXd& Fv,
                                     const ParameterSet& material,
                                     VectorXd& dUdF);

void computeDCauchyStressDF_2Din3D_StVK(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_2Din3D_CoNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_2Din3D_InNH(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_2Din3D_CoMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
void computeDCauchyStressDF_2Din3D_InMR(const VectorXd& Fv,
                                        const ParameterSet& material,
                                        MatrixXd& dPdF);
}  // namespace Energies
}  // namespace PhySim
