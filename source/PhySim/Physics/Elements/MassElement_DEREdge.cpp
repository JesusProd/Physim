//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/MassElement_DEREdge.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable_DER.h>
#include <PhySim/Physics/DoFSet.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;


	MassElement_DEREdge::MassElement_DEREdge(Simulable_DER* pModel, Edge* pEdge, PtrS<ParameterSet> pParam) : MassElement(pModel, pParam)
	{
		this->m_pEdge = pEdge;
		m_pModelDER = pModel;

		this->m_vDoF.resize(1);
		this->m_vDoF.push_back(this->m_pEdge->Traits().Kinematics(Tag::Tag_DOF_0));

		m_vDMDtv = VectorXd::Zero(0);
		m_mMass = MatrixXd::Zero(1, 1);
	}

	MassElement_DEREdge::~MassElement_DEREdge(void)
	{

	}

	void MassElement_DEREdge::Init()
	{
		// Nothing to do here...
	}

	void MassElement_DEREdge::ComputeAndStore_Mass()
	{
		Real rho = (*m_pModelDER->SetupOptions().m_pMatParams->GetValueAtDomainPoint(this->m_pEdge->ID()))[ParameterSet::Param_Density];
		Real rw = this->m_pEdge->Traits().Double(Tag_Size_0);
		Real rh = this->m_pEdge->Traits().Double(Tag_Size_1);
		Real V = this->m_pEdge->VolumeBasis(Tag_Position_0);
		Real ra = 0.5*(rw + rh);

		this->m_mMass(0, 0) = (rho*V*ra*ra)/2;
	}

}