//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_FixDoF.h>

#include <PhySim/Physics/DoFSet.h>
#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

bool BCondition::ResetLoading() {
  if (this->m_continuouslyLoaded) {
    IOUtils::logTrace(PhySim::Verbosity::V1_Default,
                      "\n[TRACE] BC %s loading (Continuous)",
                      this->Name().c_str());
    return true;
  } else {
    if (this->m_curStage == 0)
      return false;

    IOUtils::logTrace(
        PhySim::Verbosity::V1_Default,
        "\n[TRACE] BC %s loading (Full): (%d/%d)", this->Name().c_str(),
        this->m_numSubsteps * this->CurStage() + this->m_curSubsteps,
        this->m_numSubsteps * this->NumStage());

    this->m_curStage = 0;
    return true;
  }
}

bool BCondition::StepLoading() {
  if (this->m_continuouslyLoaded) {
    IOUtils::logTrace(PhySim::Verbosity::V1_Default,
                      "\n[TRACE] BC %s loading (Continuous)",
                      this->Name().c_str());
    return true;
  } else {
    if (this->CurStage() == this->NumStage())
      return false;  // Already at max step

    IOUtils::logTrace(
        PhySim::Verbosity::V1_Default,
        "\n[TRACE] BC %s loading (Step): (%d/%d)", this->Name().c_str(),
        this->m_numSubsteps * this->CurStage() + this->m_curSubsteps + 1,
        this->m_numSubsteps * this->NumStage());

    this->m_curSubsteps++;

    if (this->m_curSubsteps >= this->m_numSubsteps) {
      this->m_curStage++;
      this->m_curSubsteps = 0;
    }

    return true;
  }
}

bool BCondition::FullLoading() {
  if (this->m_continuouslyLoaded) {
    IOUtils::logTrace(PhySim::Verbosity::V1_Default,
                      "\n[TRACE] BC %s loading (Continuous)",
                      this->Name().c_str());
    return true;  // TODO: Unsure if this is the correct choice.
  } else {
    if (this->m_curStage == this->m_numStage)
      return false;

    this->m_curStage = this->m_numStage;

    IOUtils::logTrace(
        PhySim::Verbosity::V1_Default,
        "\n[TRACE] BC %s loading (Full): (%d/%d)", this->Name().c_str(),
        this->m_numSubsteps * this->CurStage() + this->m_curSubsteps,
        this->m_numSubsteps * this->NumStage());

    return true;
  }
}

}  // namespace PhySim
