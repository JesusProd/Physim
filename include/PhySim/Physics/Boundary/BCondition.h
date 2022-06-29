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

#include <PhySim/Utils/GeometryFilters.h>
#include <PhySim/Utils/VectorAnimators.h>

#include <PhySim/Physics/Boundary/IBCondition.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class BCondition : public IBCondition {
 public:
  /**
   * An animator to change the values of the boundary condition over steps.
   */
  PtrS<IVectorAnimator> m_pAnimator;

  /**
   * Current incremental loading stage.
   */
  int m_curStage;

  /**
   * Number of incremental loading stages.
   */
  int m_numStage;

  /**
   * Current number of substeps of the current loading stage
   */
  int m_curSubsteps;

  /**
   * Number of substeps to perform before each loading stage
   */
  int m_numSubsteps;

  /**
   * Maximum error allowed before performing a loading stage
   */
  Real m_maxErrorBeforeStage;

  /**
   * The values of the boundary conditions.
   */
  vector<VectorXd> m_vini;

  /**
   * The simulable object affected by the boundary condition.
   */
  Simulable* m_pModel;

  /**
   * The energy elements used to enforce the boundary condition.
   */
  vector<PtrS<IEnergyElement>> m_vpEne;

  /**
   * The constraint elements used to enforce the boundary condition.
   */
  vector<PtrS<IConstraintSet>> m_vpCon;

  /**
   * Defines whether the boundary condition is loaded continuously (e.g. for
   * collisions).
   */
  bool m_continuouslyLoaded;

  // BCSetup							m_setup;

 public:
  BCondition(Simulable* pModel) {
    assert(pModel != NULL);
    this->m_pModel = pModel;

    this->m_curStage = 0;
    this->m_numStage = 1;
    this->m_curSubsteps = 0;
    this->m_numSubsteps = 1;
    this->m_maxErrorBeforeStage = 1e9;
    this->m_vpCon.clear();
    this->m_vpEne.clear();
    this->m_pAnimator.reset();
    this->m_continuouslyLoaded = false;
  }

  virtual ~BCondition(void) {
    // Nothing to do here...
  }

  inline const vector<PtrS<IEnergyElement>>& Energies() const override {
    return this->m_vpEne;
  }
  inline const vector<PtrS<IConstraintSet>>& Constraints() const override {
    return this->m_vpCon;
  }

  inline virtual string Name() const = 0;

  inline virtual int& CurStage() override { return m_curStage; }
  inline virtual int& NumStage() override { return m_numStage; }
  inline virtual int& CurStepsBeforeStage() override { return m_curSubsteps; }
  inline virtual int& NumStepsBeforeStage() override { return m_numSubsteps; }
  inline virtual Real& MaxErrorBeforeStage() override {
    return m_maxErrorBeforeStage;
  }
  inline virtual bool& IsContinuouslyLoaded() override {
    return m_continuouslyLoaded;
  }

  inline virtual vector<VectorXd>& Values() { return this->m_vini; }

  virtual bool HasAnimator() const { return this->m_pAnimator != NULL; }

  virtual bool IsLoaded() const override {
    return m_continuouslyLoaded || this->m_curStage == this->m_numStage;
  }

  virtual bool StepLoading() override;
  virtual bool ResetLoading() override;
  virtual bool FullLoading() override;

  virtual void GetIniValues(vector<VectorXd>& vval) {
    this->GetValues(0, vval);
  }

  virtual void GetCurValues(vector<VectorXd>& vval) {
    Real a = ((Real)CurStage() / (Real)NumStage());

    this->GetValues(a, vval);
  }

  virtual void GetEndValues(vector<VectorXd>& vval) {
    this->GetValues(1, vval);
  }

  virtual void GetValues(Real alpha, vector<VectorXd>& vval) {
    if (!this->HasAnimator()) {
      vval = this->m_vini;
    } else
      this->m_pAnimator->Evaluate(alpha, vval);
  }

  virtual void Init() override = 0;
  virtual void Update() override = 0;
};

}  // namespace PhySim
