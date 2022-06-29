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

#include <PhySim/Kinematics/KinematicsEle.h>
#include <PhySim/Kinematics/KinematicsMap.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class KinematicsEle;
class KinematicsMap;

class DerivativeAssembler {
 public:
  struct DoFMapping {
    KinematicsEle* mappedElement;
    vector<KinematicsEle*> mappingElements;
    vector<vector<PtrS<KMappingBlock>>> mappingChains;

    vector<MatrixXd> mappingMatrices;
  };

 public:
  DerivativeAssembler(bool isDynamic = false) { this->m_isDynamic = isDynamic; }

  ~DerivativeAssembler() {
    // Nothing to do here
  }

  inline bool IsStatic() const { return !m_isDynamic; }
  inline bool IsDynamic() const { return m_isDynamic; }

  void InitializeMappings(const vector<IDoFSet*>& vDoF, int layer);
  void PreprocessMappings(const vector<IDoFSet*>& vDoF, int layer);
  void AssemblePreprocessedGradient(const VectorXd& vgradient,
                                    AVectorXd& vglobalGradient);
  void AssemblePreprocessedHessian(const MatrixXd& mHessian,
                                   AMatrixSd& vglobalHessian);

  void PropagateGradient(const VectorXd& vgradient,
                         const vector<IDoFSet*>& vDoF,
                         int layer);
  void AssembleGradient(AVectorXd& vglobalGradient);

  void PropagateHessian(const MatrixXd& mHessian,
                        const vector<IDoFSet*>& vDoF,
                        int layer);
  void AssembleHessian(AMatrixSd& vglobalHessian);

  void PropagateAndAssembleGradient(const VectorXd& vgradient,
                                    const vector<IDoFSet*>& vDoF,
                                    AVectorXd& vglobalGradient);
  void PropagateAndAssembleHessian(const MatrixXd& mHessian,
                                   const vector<IDoFSet*>& vDoF,
                                   AMatrixSd& mglobalHessian);

  void AssembleJacobian(const MatrixXd& mJacobian,
                        const vector<IDoFSet*>& vDoF,
                        AMatrixSd& mglobalJacobian);

 protected:
  bool m_isDynamic;

  vector<DoFMapping> m_vpMapping;
  int m_initializedMapping;

  // Deprecated?
  vector<KinematicsEle*> m_vMappingKines;
  vector<MatrixXd> m_vMappingBlocks;
  vector<MatrixXd> m_vBlocksGrad;
  vector<MatrixXd> m_vBlocksHess;
  vector<KinematicsEle*> m_vKinesFront;
  vector<KinematicsEle*> m_vKinesBack;
  vector<MatrixXd> m_vBlocksTemp;
  vector<KinematicsEle*> m_vKinesTemp;
  vector<PtrS<AMatrixSd::Block>> m_vhessBlocksRef;
};
}  // namespace PhySim