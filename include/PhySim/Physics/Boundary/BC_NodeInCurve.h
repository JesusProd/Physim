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


#include <PhySim/Physics/Boundary/BCondition.h>

#include <PhySim/Geometry/Meshes/Curve.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class BC_NodeInCurve : public BCondition
	{
	protected:
		vector<IDoFSet*>			m_vpDoF;
		bool						m_isSoft;
		Real						m_kAini;
		Real						m_kAend;
		PtrS<Curve>					m_pCurve;


	public:
		BC_NodeInCurve(Simulable* pModel, const vector<IDoFSet*>& vpDoF, PtrS<Curve> pCurve, bool isSoft = false, Real kSini = 1e9, Real kSend = 1e9);
		virtual ~BC_NodeInCurve(void);

		inline virtual string Name() const { return "PositionInCurve"; };

		virtual const bool& IsSoft() { return this->m_isSoft; }
		virtual Real& KStart() { return this->m_kAini; }
		virtual Real& KFinal() { return this->m_kAend; }
		virtual vector<IDoFSet*>& DoFs() { return this->m_vpDoF; }

		virtual void Init() override;
		virtual void Update() override;

	};
}
