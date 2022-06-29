//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#ifdef USE_KNITRO

#include <PhySim/CommonIncludes.h>

#include <PhySim/Solvers/OptimProblem_KnitroBridge.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class OptimSolver_Knitro : public IOptimSolver {
 protected:
  IOptimProblem* m_pProblem;
  bool m_isInitialized;

  void (*m_stepCallback)(IOptimSolver* pSender, void* pReceiver);
  void (*m_fullCallback)(IOptimSolver* pSender, void* pReceiver);
  void* m_stepCallbackReceiver;
  void* m_fullCallbackReceiver;

 public:
  OptimSolver_Knitro(IOptimProblem* pProblem);
  virtual void Init(IOptimProblem* pProblem);
  virtual ~OptimSolver_Knitro();

  virtual SolveResult SolveFull();
  virtual SolveResult SolveStep();

  static int FunctionCallback(const int evalRequestCode,
                              const int n,
                              const int m,
                              const int nnzJ,
                              const int nnzH,
                              const double* const x,
                              const double* const l,
                              double* const obj,
                              double* const vc,
                              double* const vg,
                              double* const mJ,
                              double* const mH,
                              double* const vH,
                              void* userParams);

  // static int NewPointCallback(
  //	KTR_context_ptr kc,
  //	const int n,
  //	const int m,
  //	const int nnzJ,
  //	const double* const x,
  //	const double* const l,
  //	const double obj,
  //	const double* const vc,
  //	const double* const vg,
  //	const double* const mJ,
  //	void* userParams);

  virtual IOptimProblem* Problem() { return this->m_pProblem; }

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

  virtual void OnStepFinished() {
    this->m_stepCallback(this, m_stepCallbackReceiver);
  }

  virtual void OnFullFinished() {
    this->m_fullCallback(this, m_fullCallbackReceiver);
  }
};
}  // namespace PhySim

#endif