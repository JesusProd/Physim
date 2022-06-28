//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable_DER.h>

#include <PhySim/Geometry/Meshes/Mesh_DER.h>

#include <PhySim/Kinematics/KEleDER.h>

#include <PhySim/Physics/Elements/MassElement_DEREdge.h>
#include <PhySim/Physics/Elements/EnergyElement_DER_Stretch.h>
#include <PhySim/Physics/Elements/EnergyElement_DER_BendTwist.h>
#include <PhySim/Physics/Elements/EnergyElement_DER_Connection.h>

#include <PhySim/Utils/DomainDistribution.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Simulable_DER::Simulable_DER() : Simulable_Mesh()
	{
		// Nothing to do here...
	}

	Simulable_DER::~Simulable_DER()
	{
		// Nothing to do...
	}

	void Simulable_DER::FreeInternal()
	{
		Simulable_Mesh::FreeInternal();

		this->m_venerEle_stretch.clear();
		this->m_venerEle_bendTwist.clear();
		this->m_venerEle_connection.clear();
	}

	Simulable_DER::Options& Simulable_DER::SetupOptions()
	{
		if (this->m_pOptions == NULL) // Create if needed
			this->m_pOptions = new Simulable_DER::Options();
		return *((Simulable_DER::Options*) this->m_pOptions);
	}

	void Simulable_DER::PreInit()
	{
		Simulable_Mesh::PreInit();

		if (this->SetupOptions().m_pMatParams == NULL)
		{
			PtrS<ParameterSet> pMatParam(new ParameterSet());
			pMatParam->InitLinearFromYoungPoisson(1e9, 0.3, 1000);
			this->SetupOptions().m_pMatParams.reset(new DomainDistribution_Constant<PtrS<ParameterSet>>(pMatParam));
		}
	}

	void Simulable_DER::CreateDiscretization(PtrS<Mesh>& pMesh)
	{
		Simulable_Mesh::CreateDiscretization(pMesh);

		m_pMeshDER = static_cast<Mesh_DER*>(pMesh.get());
	}

	void Simulable_DER::CreateKinematicsEle(vector<PtrS<KinematicsEle>>& vpEle)
	{
		Simulable_Mesh::CreateKinematicsEle(vpEle);

		vpEle.reserve(vpEle.size() + m_pMeshDER->NumNodes() + m_pMeshDER->NumEdges() + 5*m_pMeshDER->NumConns());

		// Add material twist angles at edges

		for (int i = 0; i < m_pMeshDER->NumElems(); ++i)
		{
			vpEle.push_back(PtrS<KinematicsEle>(new KEleDERMatTwist(this, m_pMeshDER->Elems()[i])));
			m_pMeshDER->Elems()[i]->Traits().AddTrait<IDoFSet*>(Tag_DOF_0, vpEle.back().get());
		}

		// Add reference twist angles at nodes

		for (int i = 0; i < m_pMeshDER->NumNodes(); ++i)
		{
			vpEle.push_back(PtrS<KinematicsEle>(new KEleDERRefTwist(this, m_pMeshDER->Nodes()[i])));
			m_pMeshDER->Nodes()[i]->Traits().AddTrait<IDoFSet*>(Tag_DOF_2, vpEle.back().get());
		}

		// Add rotation angles at conns

		for (int i = 0; i < m_pMeshDER->NumConns(); ++i)
		{
			Mesh_DER::Connection* pConn = m_pMeshDER->Connections()[i];

			vpEle.push_back(PtrS<KinematicsEle>(new KEleDERConnRot(this, pConn->m_center)));
			pConn->m_center->Traits().AddTrait<IDoFSet*>(Tag_DOF_1, vpEle.back().get());

			for (int j = 0; j < (int)pConn->m_vedges.size(); ++j)
			{
				vpEle.push_back(PtrS<KinematicsEle>(new KEleDERRefTwist(this, pConn->m_vedges[j])));
				pConn->m_vedges[j]->Traits().AddTrait<IDoFSet*>(Tag_DOF_1, vpEle.back().get());
			}
		}
	}

	void Simulable_DER::CreateEnergyElements(vector<IEnergyElement*>& vEle)
	{
		Simulable_Mesh::CreateEnergyElements(vEle);

		Options* pOptions = static_cast<Options*>(this->m_pOptions);

		int numEdges = this->m_pMeshDER->NumElems();
		int numConns = this->m_pMeshDER->NumConns();
		int numNodes = this->m_pMeshDER->NumNodes();

		// Estimate element number

		vEle.reserve(vEle.size() + numEdges + numNodes);

		m_venerEle_stretch.reserve(numEdges);
		m_venerEle_bendTwist.reserve(numNodes);
		m_venerEle_connection.reserve(4*numConns);

		// Initialize stretch elements

		for (int i = 0; i < numEdges; ++i)
		{
			this->m_venerEle_stretch.push_back(new EnergyElement_DER_Stretch(this, this->m_pMeshDER->GetEdge(i)));

			vEle.push_back(this->m_venerEle_stretch.back());
		}

		// Initialize bending/twist elements

		for (int i = 0; i < numNodes; ++i)
		{
			Node* pNode = this->m_pMesh->Nodes()[i];
			vector<Edge*> vnext = this->m_pMeshDER->NextEdges(pNode);
			vector<Edge*> vprev = this->m_pMeshDER->PrevEdges(pNode);

			if (m_pMeshDER->IsConnectionNode(pNode->ID()) || 
				vprev.size() != 1 || vnext.size() != 1)
				continue; // Connection or extemal node

			vector<Edge*> vedges(2);
			vedges[0] = vprev[0];
			vedges[1] = vnext[0];
			this->m_venerEle_bendTwist.push_back(new EnergyElement_DER_BendTwist(this, vedges));

			vEle.push_back(this->m_venerEle_bendTwist.back());
		}

		// Initialize connection elements

		for (int i = 0; i < numConns; ++i)
		{
			Mesh_DER::Connection* pConn = this->m_pMeshDER->Connections()[i];
			int numConnEdges = (int) pConn->m_vedges.size();
			for (int j = 0; j < numConnEdges; ++j)
			{
				this->m_venerEle_connection.push_back(new EnergyElement_DER_Connection(this, pConn->m_center, pConn->m_vedges[j]));

				vEle.push_back(this->m_venerEle_connection.back());
			}
		}
	}

	void Simulable_DER::CreateMassElements(vector<IMassElement*>& vMasses)
	{
		Simulable_Mesh::CreateMassElements(vMasses);

		// Estimate element number

		int numEdges = this->m_pMeshDER->NumElems();
		vMasses.reserve(vMasses.size() + numEdges);

		// Initialize stretch elements

		for (int i = 0; i < numEdges; ++i)
		{
			PtrS<ParameterSet> pPar = this->m_pOptions->m_pMatParams->GetValueAtDomainPoint(i);
			vMasses.push_back(new MassElement_DEREdge(this, this->m_pMeshDER->GetEdge(i), pPar));
		}
	}

}

