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

class IEnergyElement {
 public:
  IEnergyElement(){};
  virtual ~IEnergyElement(){};

  virtual string GetName() const = 0;

  virtual void Init() = 0;

  virtual int GetSupportSize() const = 0;
  // virtual const vector<IDoFSet*>& GetSupportDoF() const = 0;
  // virtual void SetSupportDoF(const vector<IDoFSet*> vns) = 0;

  virtual void UpdateKinematics() = 0;
  virtual void UpdateMechanics() = 0;
  virtual void ComputeAndStore_Energy() = 0;
  virtual void ComputeAndStore_Gradient() = 0;
  virtual void ComputeAndStore_Hessian() = 0;

  virtual void PreprocessAssembly(int layer) = 0;
  virtual void AssemblePreprocessedGradient(AVectorXd& vglobalGradient) = 0;
  virtual void AssemblePreprocessedHessian(AMatrixSd& mglobalHessian) = 0;

  virtual void Propagate_Gradient(AVectorXd& vglobalGradient) = 0;
  virtual void Propagate_Hessian(AMatrixSd& mglobalHessian) = 0;

  virtual void Assemble_Gradient(AVectorXd& vglobalGradient) = 0;
  virtual void Assemble_Hessian(AMatrixSd& mglobalHessian) = 0;

  virtual void PropagateAndAssemble_Gradient(AVectorXd& vglobalGradient) = 0;
  virtual void PropagateAndAssemble_Hessian(AMatrixSd& mglobalHessian) = 0;
  virtual int GetHessianSize() const = 0;

  virtual const Real& GetIntegrationVolume() const = 0;
  virtual void SetIntegrationVolume(const Real& iv) = 0;

  virtual Real GetLocalEnergy() const = 0;
  virtual const VectorXd& GetLocalGradient() const = 0;
  virtual const MatrixXd& GetLocalHessian() const = 0;

  virtual bool TestLocalGradient() = 0;
  virtual bool TestLocalHessian() = 0;

  virtual bool GetUseFDGradient() const = 0;
  virtual void SetUseFDGradient(bool f) = 0;

  virtual bool GetUseFDHessian() const = 0;
  virtual void SetUseFDHessian(bool f) = 0;
};

}  // namespace PhySim
