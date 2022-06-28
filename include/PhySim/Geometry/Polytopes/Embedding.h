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


#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Embedding
	{
	protected:
		bool				m_isValid;
		Poly*				m_pMaster;
		VectorXd			m_vpar;

	public:
		Embedding();
		Embedding(Poly* pMaster, const VectorXd& vpar);
		virtual ~Embedding();

		inline const bool& Valid() const { return this->m_isValid; }
		inline const VectorXd& Parameter() const { return this->m_vpar; }
		inline Poly* Master() { return this->m_pMaster; }

		virtual VectorXd InterpolateValue(Tag s) const;
		virtual VectorXd InterpolateDeformation(Tag s0, Tag sx) const;

	};
}
