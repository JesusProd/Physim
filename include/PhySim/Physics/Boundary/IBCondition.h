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


#include <PhySim/Physics/Elements/IEnergyElement.h>
#include <PhySim/Physics/Elements/IConstraintSet.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;


	class IBCondition
	{
	public:
		IBCondition() { };
		virtual ~IBCondition() { };

		// Simulable modifiers
		virtual const vector<PtrS<IEnergyElement>>& Energies() const = 0;
		virtual const vector<PtrS<IConstraintSet>>& Constraints() const = 0;

		// Incremental loading
		virtual bool StepLoading() = 0;
		virtual bool ResetLoading() = 0;
		virtual bool FullLoading() = 0;
		virtual bool IsLoaded() const = 0;

		inline virtual int& CurStage() = 0;
		inline virtual int& NumStage() = 0;
		inline virtual int& CurStepsBeforeStage() = 0;
		inline virtual int& NumStepsBeforeStage() = 0;
		inline virtual Real& MaxErrorBeforeStage() = 0;
		inline virtual bool& IsContinuouslyLoaded() = 0;

		// Initialization
		virtual void Init() = 0;
		virtual void Update() = 0;
	};

}
