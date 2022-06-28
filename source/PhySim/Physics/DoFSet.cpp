#include <PhySim/Physics/DoFSet.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Geometry/Geometry.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	DoFSet::DoFSet()
	{
		this->m_numDim = -1;

		m_offset = -1;
		m_index = -1;
		m_layer = -1;
		m_active = true;

		this->m_pModel = NULL;
		this->m_pGeometry = NULL;
	}

	DoFSet::DoFSet(Simulable* pModel)
	{
		this->m_numDim = -1;

		m_offset = -1;
		m_index = -1;
		m_layer = -1;
		m_active = true;

		this->m_pModel = pModel;
		this->m_pGeometry = NULL;
		this->m_layer = pModel->ID();
	}

	DoFSet::DoFSet(Simulable* pModel, Geometry* pGeometry, int numDim)
	{
		this->m_pModel = pModel;
		this->m_numDim = numDim;
		this->m_layer = pModel->ID();

		m_offset = -1;
		m_index = -1;
		m_active = true;
		m_vfixed.resize(numDim, false);
		this->m_pGeometry = pGeometry;
	}

	void DoFSet::GetTrait(Tag t, VectorXd& vt) const
	{
		vt = this->Traits().VectorXd(t);
	}

	void DoFSet::SetTrait(Tag t, const VectorXd& vt)
	{
		this->Traits().VectorXd(t) = vt;
	}

	const TraitSet& DoFSet::Traits() const
	{
		if (this->m_pGeometry == NULL)
			return this->m_traitSet;
		else return this->m_pGeometry->Traits();
	}

	TraitSet& DoFSet::Traits()
	{
		if (this->m_pGeometry == NULL)
			return this->m_traitSet;
		else return this->m_pGeometry->Traits();
	}

}

