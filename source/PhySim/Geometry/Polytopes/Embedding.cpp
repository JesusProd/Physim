//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Embedding.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Poly.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Embedding::Embedding()
	{
		m_isValid = false;
		this->m_pMaster = NULL;
		this->m_vpar.setZero();
	}

	Embedding::Embedding(Poly* pMaster, const VectorXd& vpar)
	{
		this->m_pMaster = pMaster;
		this->m_vpar = vpar;
		this->m_isValid = this->m_pMaster->IsValidParametric(m_vpar);
	}

	Embedding::~Embedding(void)
	{
		// Nothing to do here...
	}

	VectorXd Embedding::InterpolateValue(Tag s) const
	{
		return this->m_pMaster->InterpolateValue(this->m_vpar, s);
	}

	VectorXd Embedding::InterpolateDeformation(Tag s0, Tag sx) const
	{
		return this->m_pMaster->InterpolateDeformation(this->m_vpar, s0, sx);
	}

}