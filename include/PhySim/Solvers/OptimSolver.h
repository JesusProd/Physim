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

#include <PhySim/Solvers/LinearSolver.h>
#include <PhySim/Solvers/OptimProblem.h>

#include <PhySim/Utils/CustomTimer.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

struct OptimSolverOptions {
  Real tolMaxError;  // Tolerance: maximum opt error
  Real tolMinStep;   // Tolerance: minimum sol step size
  Real tolMinImpr;   // Tolerance: minimum obj step size
  int minStepMaxCount;
  int minImprMaxCount;
  int numMaxIters;
  Real maxStepSize;
  bool trySteepest;
  QPSolverType qpSolverType;
  LSearchType lSearchType;

  int lsearch_numTry;
  Real lSearch_alpha;
  Real lSearch_wolfe1;
  Real lSearch_wolfe2;

  LSSolverType lsSolverType;
  int lsNumMaxIters;
  Real lsTolMaxError;
  int lsNumRegTrials;

  bool profileTime;

  OptimSolverOptions() {
    qpSolverType = QPSolverType::QP_Newton;
    tolMaxError = 1e-6;
    tolMinStep = 1e-5;
    minStepMaxCount = 3;
    tolMinImpr = 1e-9;
    minImprMaxCount = 3;
    numMaxIters = 100;
    trySteepest = true;
    maxStepSize = 1000;

    lsSolverType = LSSolverType::LS_EigenLDLT;
    lsNumRegTrials = 10;
    lsNumMaxIters = 1000;
    lsTolMaxError = 1e-9;

    lSearchType = LSearchType::LSearch_Simple;
    lSearch_alpha = 0.5;
    lSearch_wolfe1 = 1e-4;
    lSearch_wolfe2 = 0.9;
    lsearch_numTry = 32;

    profileTime = true;
  }
};

class IOptimSolver {
 public:
  IOptimSolver(){};
  virtual ~IOptimSolver(){};

  virtual const OptimState& SolveStep() = 0;
  virtual const OptimState& SolveFull() = 0;
  virtual void RegisterCallback_Step(void (*amazingCallback)(IOptimSolver*,
                                                             void*),
                                     void* pReceiver) = 0;
  virtual void RegisterCallback_Full(void (*amazingCallback)(IOptimSolver*,
                                                             void*),
                                     void* pReceiver) = 0;
};

class OptimSolver : public IOptimSolver {
 protected:
  OptimSolverOptions m_options;

  IOptimProblem* m_pProblem;
  ILinearSolver* m_pLSolver;

  CustomTimer m_timerLinearSolve;
  CustomTimer m_timerComputeStep;
  CustomTimer m_timerLineSearch;

  OptimState m_state;

  bool m_isInitialized;

  void (*m_stepCallback)(IOptimSolver* pSender, void* pReceiver);
  void (*m_fullCallback)(IOptimSolver* pSender, void* pReceiver);
  void* m_stepCallbackReceiver;
  void* m_fullCallbackReceiver;

  // For BFGS step

  struct BFGSState {
    // COMMON
    bool m_isInit;
    VectorXd m_vx_P;  // Prev. solution
    VectorXd m_vg_P;  // Prev. gradient
    int m_N;          // Problem size

    // BFGS
    VectorXd m_vs;
    VectorXd m_vy;
    MatrixSd m_mBFGS;
    MatrixXd m_mBFGSMod_A;
    MatrixXd m_mBFGSMod_B;

    // LBFGS
    MatrixXd m_mS;   // History of the s vectors
    MatrixXd m_mY;   // History of the y vectors
    VectorXd m_vys;  // History of the s'y values
    VectorXd m_va;   // History of the step lengths
    int m_K;         // Last position
    int m_M;         // Buffer size
  };
  BFGSState m_bfgs;

 public:
  OptimSolver(IOptimProblem* pProblem, const OptimSolverOptions& options);

  virtual ~OptimSolver();

  IOptimProblem* getProblem() { return m_pProblem; }

  virtual void Init(IOptimProblem* pProblem, const OptimSolverOptions& options);

  virtual OptimSolverOptions& Options() { return m_options; }

  virtual const OptimState& SolveStep() override;
  virtual const OptimState& SolveFull() override;

  virtual const CustomTimer& GetTimer_LinearSolve() const {
    return this->m_timerLinearSolve;
  }
  virtual const CustomTimer& GetTimer_ComputeStep() const {
    return this->m_timerComputeStep;
  }
  virtual const CustomTimer& GetTimer_LineSearch() const {
    return this->m_timerLineSearch;
  }

  virtual const OptimState& SolveStepInternal() = 0;

  virtual void RegisterCallback_Step(void (*amazingCallback)(IOptimSolver*,
                                                             void*),
                                     void* pReceiver) override {
    this->m_stepCallback = amazingCallback;
    this->m_stepCallbackReceiver = pReceiver;
  }

  virtual void RegisterCallback_Full(void (*amazingCallback)(IOptimSolver*,
                                                             void*),
                                     void* pReceiver) override {
    this->m_fullCallback = amazingCallback;
    this->m_fullCallbackReceiver = pReceiver;
  }

  virtual void OnEvent_AfterStep() {
    if (m_stepCallback != NULL)
      m_stepCallback(this, m_stepCallbackReceiver);
  }

  virtual void OnEvent_AfterFull() {
    if (m_fullCallback != NULL)
      m_fullCallback(this, m_stepCallbackReceiver);
  }

 protected:
  virtual bool ComputeStep(VectorXd& dx, int trial = 0);
  virtual bool ComputeStep_Newton(VectorXd& dx, int trial = 0);
  virtual bool ComputeStep_BFGSDirect(VectorXd& dx, int trial = 0);
  virtual bool ComputeStep_BFGSInverse(VectorXd& dx, int trial = 0);
  virtual bool ComputeStep_LBFGS(VectorXd& dx, int trial = 0);

  virtual bool LineSearch(const VectorXd& dxIn,
                          VectorXd& dxOut,
                          Real& objNew,
                          AVectorXd& vgNew,
                          int& numBis);
  virtual bool LineSearch_Simple(const VectorXd& dxIn,
                                 VectorXd& dxOut,
                                 Real& objNew,
                                 AVectorXd& vgNew,
                                 int& numBis);
  virtual bool LineSearch_Armijo(const VectorXd& dxIn,
                                 VectorXd& dxOut,
                                 Real& objNew,
                                 AVectorXd& vgNew,
                                 int& numBis);
  virtual bool LineSearch_WolfeWeak(const VectorXd& dxIn,
                                    VectorXd& dxOut,
                                    Real& objNew,
                                    AVectorXd& vgNew,
                                    int& numBis);
  virtual bool LineSearch_WolfeStrong(const VectorXd& dxIn,
                                      VectorXd& dxOut,
                                      Real& objNew,
                                      AVectorXd& vgNew,
                                      int& numBis);
  virtual bool LSearch_None(const VectorXd& dxIn,
                            VectorXd& dxOut,
                            Real& objNew,
                            AVectorXd& vgNew,
                            int& numBis);

  virtual void UpdateStepResult(const VectorXd& dx,
                                const Real& objNew,
                                const AVectorXd& vgNew);

  virtual bool UpdateBFGS_Inverse(const VectorXd& vxNew, const VectorXd& vgNew);
  virtual bool UpdateBFGS_Direct(const VectorXd& vxNew, const VectorXd& vgNew);
  virtual bool InitialBFGS(const VectorXd& vxNew, const VectorXd& vgNew);
};
}  // namespace PhySim