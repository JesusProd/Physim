//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable_ThinShell.h>

#include <PhySim/Geometry/Meshes/Mesh_Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face.h>

#include <PhySim/Physics/Elements/EnergyElement_FEM.h>
#include <PhySim/Physics/Elements/EnergyElement_DiscreteShells.h>

#include <igl/principal_curvature.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Simulable_ThinShell::Simulable_ThinShell() : Simulable_FEM_Surface()
	{
		// Nothing to do here...
	}

	Simulable_ThinShell::~Simulable_ThinShell()
	{
		this->FreeInternal();

#ifndef NDEBUG
		logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Simulable_ThinShell");
#endif
	}

	void Simulable_ThinShell::FreeInternal()
	{
		Simulable_FEM::FreeInternal();

		this->m_venergyEle_bending.clear();
	}


	Simulable_ThinShell::Options& Simulable_ThinShell::SetupOptions()
	{
		if (this->m_pOptions == NULL) // Create if needed
			this->m_pOptions = new Simulable_ThinShell::Options();
		return *((Simulable_ThinShell::Options*) this->m_pOptions);
	}

	void Simulable_ThinShell::CreateEnergyElements(vector<IEnergyElement*>& vEnergies)
	{
		Simulable_FEM::CreateEnergyElements(vEnergies);

		// Check bending material

		assert(this->SetupOptions().m_material.HasParameter(ParameterSet::Param_Young));
		assert(this->SetupOptions().m_material.HasParameter(ParameterSet::Param_Poisson));
		assert(this->SetupOptions().m_material.HasParameter(ParameterSet::Param_Thickness));

		// Add the bending elements

		Options* pOptions = static_cast<Options*>(this->m_pOptions);

		Mesh_Face* pMesh = this->MeshFace();

		int numEdges = pMesh->NumEdges();
		this->m_venergyEle_bending.reserve(numEdges);
		this->m_venerEle.reserve(this->m_venerEle.size() + numEdges);

		for (int i = 0; i < numEdges; ++i)
		{
			Edge* pEdge = pMesh->Edges()[i];

			if (pMesh->IsBoundary(pMesh->GetHEEdge(i)->m_pHandle))
				continue; // Boundary edge, no bending through this

			// Add material

			//pEdge->Traits().AddTrait<ParameterSet>(Semantics::Tag_Mat_0, ParameterSet());

			Real Y = pOptions->m_material[ParameterSet::Param_Young];
			Real v = pOptions->m_material[ParameterSet::Param_Poisson];
			Real t = pOptions->m_material[ParameterSet::Param_Thickness];
			/*			pEdge->Traits().ParameterSet(Semantics::Tag_Mat_0).AddParameter(ParameterSet::Param_Young, Y);
			pEdge->Traits().ParameterSet(Semantics::Tag_Mat_0).AddParameter(ParameterSet::Param_Poisson, v);
			pEdge->Traits().ParameterSet(Semantics::Tag_Mat_0).AddParameter(ParameterSet::Param_Thickness, t)*/;
			pEdge->Traits().AddTrait(Tag::Tag_Mat_Young, Y);
			pEdge->Traits().AddTrait(Tag::Tag_Mat_Poisson, v);
			pEdge->Traits().AddTrait(Tag::Tag_Mat_Thickness, t);
			if (pOptions->m_material.HasParameter(ParameterSet::Param_BendingK))
			{
				Real kB = pOptions->m_material[ParameterSet::Param_BendingK];
				//pEdge->Traits().ParameterSet(Tag_Mat_0).AddParameter(ParameterSet::Param_BendingK, kB);

				pEdge->Traits().AddTrait(Tag::Tag_Mat_BendingK, pOptions->m_material[ParameterSet::Param_BendingK]);
			}
			else
			{
				pEdge->Traits().AddTrait(Tag::Tag_Mat_BendingK, 0);
			}

			// Add element

			EnergyElement_DiscreteShells* pEle = new EnergyElement_DiscreteShells(this, pEdge);

			vEnergies.push_back(pEle);
			this->m_venergyEle_bending.push_back(pEle);
		}
	}

	void Simulable_ThinShell::ComputePlanarStrain(vector<vector<Matrix2d>>& vmEp)
	{
		Mesh_Face* pMesh = MeshFace();

		vmEp.resize(pMesh->NumElems());

		for (int i = 0; i < pMesh->NumElems(); ++i)
		{
			int numQ = this->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(i)->NumPoints();
			const vector<VectorXd>& vp = this->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(i)->Points();

			vmEp[i].resize(numQ);
			for (int j = 0; j < numQ; ++j)
				vmEp[i][j] = pMesh->ComputePlanarStrain(pMesh->Faces()[i], Tag_Position_0, Tag_Position_X, vp[j]);
		}
	}

	void Simulable_ThinShell::ComputeBendingStrain(vector<vector<Matrix2d>>& vmEb)
	{
		Mesh_Face* pMesh = MeshFace();

		vmEb.resize(pMesh->NumElems());

		for (int i = 0; i < pMesh->NumElems(); ++i)
		{
			MatrixXd mEb = pMesh->ComputeBendingStrain(pMesh->Faces()[i], Tag_Position_0, Tag_Position_X);

			int numQ = this->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(i)->NumPoints();
			
			vmEb[i].resize(numQ);
			for (int j = 0; j < numQ; ++j)
				vmEb[i][j] = mEb;
		}
	}

	void Simulable_ThinShell::ComputeNodalBendingStrain(vector<Matrix2d>& vmEb)
	{
		Mesh_Face* pMesh = MeshFace();

		MatrixXd mV;
		MatrixXi mF;
		MatrixXd PD1, PD2;
		VectorXd PV1, PV2;
		pMesh->GetElemMatrix(mF);
		pMesh->GetNodesTrait(mV, Tag_Position_X);
		igl::principal_curvature(mV, mF, PD1, PD2, PV1, PV2);

		vmEb.resize(pMesh->NumNodes());
		for (int i = 0; i < pMesh->NumNodes(); ++i)
		{
			Vector3d v13d = PD1.row(i).transpose();
			Vector3d v23d = PD2.row(i).transpose();

			Matrix3d mT;
			mT.col(0) = v13d;
			mT.col(1) = v23d;
			mT.col(2) = v13d.cross(v23d);

			assert((mT*mT.transpose() - Matrix3d::Identity()).norm() < 1e-6);

			MatrixXd mA = mT.inverse().block(0, 0, 2, 3);
			Vector2d v1 = mA * PD1.row(i).transpose();
			Vector2d v2 = mA * PD2.row(i).transpose();

			// Check principal curvatures are orthonormal
			assert(abs(v1.norm() - 1) <= 1e-9);
			assert(abs(v2.norm() - 1) <= 1e-9);
			assert(abs(v1.dot(v2)) <= 1e-9);

			// Reconstruct shape operator 

			Matrix2d D = Matrix2d::Zero();
			D(0, 0) = abs(PV1(i));
			D(1, 1) = abs(PV2(i));
			Matrix2d U;
			U.col(0) = v1;
			U.col(1) = v2;
			vmEb[i] = U*D*U.transpose();
		}
	}

	void Simulable_ThinShell::ComputeNodalBendingStress(vector<Matrix2d>& vmSb)
	{
		vector<Matrix2d> vmEb;

		this->ComputeNodalBendingStrain(vmEb);

		Real thickness = this->SetupOptions().m_material[ParameterSet::Param_Thickness];

		vmSb.resize((int) vmEb.size());
		for (int i = 0; i < (int)vmEb.size(); ++i)
			vmSb[i] = ComputeStressForE((thickness/2)*vmEb[i]);
	}

	void Simulable_ThinShell::ComputeTotalStrain(vector<vector<Matrix2d>>& vmEt)
	{
		Mesh_Face* pMesh = MeshFace();

		vector<vector<Matrix2d>> vmEb;
		this->ComputeBendingStrain(vmEb);
		this->ComputePlanarStrain(vmEt);

		Real thickness = this->SetupOptions().m_material[ParameterSet::Param_Thickness];

		// Add bending strain

		for (int i = 0; i < pMesh->NumElems(); ++i)
		{
			int numQ = this->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(i)->NumPoints();
			for (int j = 0; j < numQ; ++j)
			{
				const Matrix2d& mB = vmEb[i][j];
				JacobiSVD<Matrix2d> svd = mB.jacobiSvd(ComputeFullU | ComputeFullV);
				Matrix2d S = Matrix2d::Zero();
				S(0, 0) = abs(svd.singularValues()[0]);
				S(1, 1) = abs(svd.singularValues()[1]);
				vmEt[i][j] += (thickness / 2)*svd.matrixU()*S*svd.matrixV().transpose();
			}
		}
	}

	void Simulable_ThinShell::ComputeStressIntegral(const vector<vector<Matrix2d>>& vmE, vector<Matrix2d>& vmS)
	{
		Mesh_Face* pMesh = MeshFace();

		vmS.resize(pMesh->NumElems());

		for (int i = 0; i < pMesh->NumElems(); ++i)
		{
			int numQ = this->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(i)->NumPoints();

			PtrS<EnergyElement_FEM> pEle = dynamic_pointer_cast<EnergyElement_FEM>(m_venerEle[i]);

			// Stress integration

			vmS[i].setZero();
			for (int j = 0; j < numQ; ++j)
				vmS[i] += this->ComputeStressForE(vmE[i][j])*pEle->GetIntegrationWeights()[j];
		}
	}

	void Simulable_ThinShell::ComputeStressDensity(const vector<vector<Matrix2d>>& vmE, vector<Matrix2d>& vmS)
	{
		Mesh_Face* pMesh = MeshFace();

		vmS.resize(pMesh->NumElems());

		for (int i = 0; i < pMesh->NumElems(); ++i)
		{
			int numQ = this->SetupOptions().m_pQuadratures->GetValueAtDomainPoint(i)->NumPoints();

			PtrS<EnergyElement_FEM> pEle = dynamic_pointer_cast<EnergyElement_FEM>(m_venerEle[i]);

			// Stress density average
			
			double weiSum = 0;

			vmS[i].setZero();
			for (int j = 0; j < numQ; ++j)
			{
				Real wei = pEle->GetIntegrationWeights()[j];
				vmS[i] += ComputeStressForE(vmE[i][j])*wei;
				weiSum += wei;
			}

			vmS[i] /= weiSum;
		}
	}

	Matrix2d Simulable_ThinShell::ComputeStressForE(const Matrix2d& mS)
	{
		Options* pOptions = static_cast<Options*>(m_pOptions);

		//switch (pOptions->m_material.PhysicalModel())
		//{
		//case ParameterSet::Model_StVK:
		//	return Utils::computeStressForE_2DStVK(mS, pOptions->m_material);
		//	break;
		//case ParameterSet::Model_CoNH:
		//	return Utils::computeStressForE_2DCoNH(mS, pOptions->m_material);
		//	break;
		//}

		throw new exception("[ERROR] Invalid material model.");
	}

}

