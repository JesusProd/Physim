//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_FEM.h>

#include <PhySim/Geometry/Polytopes/Poly.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Physics/Simulables/Simulable_FEM.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_FEM::EnergyElement_FEM(Simulable_FEM* pSimul, Poly* pPoly) : EnergyElement(pSimul)
	{
		this->m_pSimulFEM = pSimul;
		this->m_pElemPoly = pPoly;

		int N = (int)pPoly->Nodes().size();

		this->m_vDoF.resize(N);
		for (int i = 0; i < N; ++i)
		{
			this->m_vDoF[i] = pPoly->Nodes()[i]->Traits().Kinematics(Tag::Tag_DOF_0);
		}

		this->m_vgradient.resize(3 * N);
		this->m_mHessian.resize(3 * N, 3 * N);

		this->m_mstrain0.setIdentity(pPoly->DimBasis(), pPoly->DimBasis());
	}

	EnergyElement_FEM::~EnergyElement_FEM()
	{
		// Nothing to do...
	}

	void EnergyElement_FEM::UpdateKinematics()
	{
		this->m_pSimulFEM->SetupOptions().m_pShapeFunctions->GetValueAtDomainPoint(this->m_pElemPoly->ID())->UpdateKinematicsAtSamples(m_pSampleData);
	}

	void EnergyElement_FEM::UpdateMechanics()
	{
		this->m_pSimulFEM->SetupOptions().m_pShapeFunctions->GetValueAtDomainPoint(this->m_pElemPoly->ID())->UpdateDeformationAtSamples(m_pSampleData);
	}

	void EnergyElement_FEM::Init()
	{
		int dimSpace = m_pElemPoly->DimSpace();
		int dimBasis = m_pElemPoly->DimBasis();
		int numNodes = m_pElemPoly->NumNodes();

		PtrS<IQuadrature> pQuad = this->m_pSimulFEM->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		PtrS<IShapeFunction> pShape = this->m_pSimulFEM->SetupOptions().m_pShapeFunctions->GetValueAtDomainPoint(this->m_pElemPoly->ID());

		const dVector& vw = pQuad->Compute_Values();
		const vector<VectorXd>& vp = pQuad->Points();

		m_pSampleData = pShape->CreateDeformationData();
		m_pSampleData->SetPoly(m_pElemPoly);
		m_pSampleData->SetTag0(Tag_Position_0);
		m_pSampleData->SetTagX(Tag_Position_X);
		m_pSampleData->SetPoints(vp);

		pShape->InitDeformationAtSamples(m_pSampleData);
		pShape->UpdateKinematicsAtSamples(m_pSampleData);

		// UpdateMechanics quadrature weights

		this->m_vintWei.resize(pQuad->NumPoints());

		for (int i = 0; i < pQuad->NumPoints(); ++i)
		{
			// UpdateMechanics integration weight: quadrature weight · |DXDp|_i

			m_vintWei[i] = vw[i] * m_pSampleData->Get_J()[i].determinant();

			// If it's 2D element embedded in 3D space, consider thickness

			if (dimSpace != 2 && dimBasis == 2)
			{
				this->m_vintWei[i] *= m_pElemPoly->Traits().Double(Tag::Tag_Mat_Thickness);
			}

			// If it's 1D element embedded in 3D space, consider thickness

			if (dimSpace != 1 && dimBasis == 1)
			{
				this->m_vintWei[i] *= M_PI *
					m_pElemPoly->Traits().Double(Tag::Tag_Mat_Radius0)*
					m_pElemPoly->Traits().Double(Tag::Tag_Mat_Radius1);
			}
		}

		// Set the integration volume

		this->m_intVolume = this->m_pElemPoly->VolumeBasis(Tag_Position_0);

		// If it's 2D elemente embedded in 3D space, consider also thickness

		if (dimSpace != 2 && dimBasis == 2)
		{
			this->m_intVolume *= m_pElemPoly->Traits().Double(Tag::Tag_Mat_Thickness);
		}

		// If it's 1D elemente embedded in 3D space, consider also thickness

		if (dimSpace != 1 && dimBasis == 1)
		{
			this->m_intVolume *= M_PI *
				m_pElemPoly->Traits().Double(Tag::Tag_Mat_Radius0)*
				m_pElemPoly->Traits().Double(Tag::Tag_Mat_Radius1);
		}
	}

	void EnergyElement_FEM::ComputeAndStore_Energy_Internal()
	{
		if (this->m_pElemPoly->VolumeBasis(Tag_Position_X) < 0)
		{
            IOUtils::logTrace(PhySim::Verbosity::V1_Default,
                              "\n[WARNING] Inverted element detected!");
			this->m_energy = HUGE_VAL;
			return; // Inverted element
		}

		PtrS<IQuadrature> pQuad = this->m_pSimulFEM->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		PtrS<IShapeFunction> pShape = this->m_pSimulFEM->SetupOptions().m_pShapeFunctions->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		const vector<VectorXd>& vp = pQuad->Points();

		this->m_energy = 0;

		for (int i = 0; i < pQuad->NumPoints(); ++i)
		{
			// Obtain material at quadrature point

			PtrS<IMaterialModel> pMatModel = this->m_pSimulFEM->SetupOptions().m_pMatModels->GetValueAtDomainPoint(this->m_pElemPoly->ID(), vp[i]);
			PtrS<ParameterSet> pMatParam = this->m_pSimulFEM->SetupOptions().m_pMatParams->GetValueAtDomainPoint(this->m_pElemPoly->ID(), vp[i]);

			// Vectorize DxD0 colmajor

			Real energyi = 0;
			pMatModel->ComputeEnergyDensity(*pMatParam, m_pSampleData->Get_F()[i], energyi);
			this->m_energy += energyi * m_vintWei[i];
		}
	}

	void EnergyElement_FEM::ComputeAndStore_Gradient_Internal()
	{
		int dimSpace = m_pElemPoly->DimSpace();
		int numNodes = m_pElemPoly->NumNodes();

		PtrS<IQuadrature> pQuad = this->m_pSimulFEM->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		PtrS<IShapeFunction> pShape = this->m_pSimulFEM->SetupOptions().m_pShapeFunctions->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		const vector<VectorXd>& vp = pQuad->Points();

		this->m_vgradient.setZero(dimSpace * numNodes);

		for (int i = 0; i < pQuad->NumPoints(); ++i)
		{
			// Obtain material at quadrature point

			PtrS<IMaterialModel> pMatModel = this->m_pSimulFEM->SetupOptions().m_pMatModels->GetValueAtDomainPoint(this->m_pElemPoly->ID(), vp[i]);
			PtrS<ParameterSet> pMatParam = this->m_pSimulFEM->SetupOptions().m_pMatParams->GetValueAtDomainPoint(this->m_pElemPoly->ID(), vp[i]);

			VectorXd vPK1;
			pMatModel->ComputePK1(*pMatParam, m_pSampleData->Get_F()[i], vPK1);
			const MatrixXd& mDFDxi = this->m_pSampleData->Get_DFDx()[i];
			this->m_vgradient += (mDFDxi.transpose()*vPK1)*m_vintWei[i];
		}
	}

	void EnergyElement_FEM::ComputeAndStore_Hessian_Internal()
	{
		int dimSpace = m_pElemPoly->DimSpace();
		int numNodes = m_pElemPoly->NumNodes();

		PtrS<IQuadrature> pQuad = this->m_pSimulFEM->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		PtrS<IShapeFunction> pShape = this->m_pSimulFEM->SetupOptions().m_pShapeFunctions->GetValueAtDomainPoint(this->m_pElemPoly->ID());
		const vector<VectorXd>& vp = pQuad->Points();

		m_mHessian.setZero(dimSpace * numNodes, dimSpace * numNodes);

		for (int i = 0; i < pQuad->NumPoints(); ++i)
		{
			// Obtain material at quadrature point

			PtrS<IMaterialModel> pMatModel = this->m_pSimulFEM->SetupOptions().m_pMatModels->GetValueAtDomainPoint(this->m_pElemPoly->ID(), vp[i]);
			PtrS<ParameterSet> pMatParam = this->m_pSimulFEM->SetupOptions().m_pMatParams->GetValueAtDomainPoint(this->m_pElemPoly->ID(), vp[i]);

			MatrixXd mDPK1DF;
			pMatModel->ComputeDPK1DF(*pMatParam, m_pSampleData->Get_F()[i], mDPK1DF);
			const MatrixXd& mDFDxi = this->m_pSampleData->Get_DFDx()[i];
			m_mHessian += (mDFDxi.transpose()*mDPK1DF*mDFDxi)*m_vintWei[i];
		}
	}

	MatrixXd EnergyElement_FEM::ComputeRestStrain()
	{
		return MatrixXd::Identity(this->m_pElemPoly->DimBasis(), this->m_pElemPoly->DimBasis());
	}

}