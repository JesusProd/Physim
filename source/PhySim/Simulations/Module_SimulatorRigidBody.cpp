//==========================================================
//
//	PhyEye library. Generic library for physics visualization.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC Madrid
//
//==========================================================

#pragma once

#include <PhyEye/Modules/Module_SimulatorRigidBody.h>

#include <PhyEye/Modules/RenderGeometry.h>
#include <PhyEye/Modules/RenderMaterial.h>

#include <PhySim/Physics/DoFSet.h>

#include <PhySim/Physics/Boundary/BC_FixDoF.h>
#include <PhySim/Physics/Boundary/BC_Force.h>
#include <PhySim/Physics/Boundary/BC_Gravity.h>

#include <PhySim/Physics/Simulables/Simulable_RB.h>

#include <PhySim/Utils/Serializer_Mesh.h>

#include <PhySim/Geometry/Meshes/Mesh_Tri.h>


namespace PhyEye
{
	using namespace std;
	using namespace Eigen;

	Module_SimulatorRigidBody::Module_SimulatorRigidBody()
	{
		// Nothing to do here...
	}

	Module_SimulatorRigidBody::~Module_SimulatorRigidBody()
	{
		// Nothing to do here...
	}

	void Module_SimulatorRigidBody::Setup(IAppEye* pApp)
	{
		this->m_pControlRB = new Control_SimulatorRigidBody(this);
		this->m_mcontrols.insert(pair<string, IControlRef>(Control_SimulatorRigidBody::NAME(), IControlRef(static_cast<IControl*>(m_pControlRB))));
		this->m_operation = 0;
	}

	bool Module_SimulatorRigidBody::StartSimulation()
	{
		bool result;

		switch (this->m_pControlRB->m_selectedSimulation)
		{
		case 0: result = this->StartSimulation_0(); break;
		}

		return result;
	}

	bool Module_SimulatorRigidBody::StartSimulation_0()
	{
		this->m_pSimulableRB = new Simulable_RB();

		MatrixXd mV0, mV1;
		MatrixXi mT0, mT1;
		Utils::create3DGridNodes(1, 1, 1, Vector3d(0.1, 0.1, 0.1), mV0);
		Utils::create3DGridIndex_Tri3(1, 1, 1, mT0);
		Utils::create3DGridIndex_Tri3(1, 1, 1, mT1);
		mV1 = mV0;
		mV1.rowwise() += Vector3d(0.25, 0, 0).transpose();

		vector<PhySim::Tag> vnTraits(3);
		vnTraits[0] = Tag_Position_0;
		vnTraits[1] = Tag_Position_X;
		vnTraits[2] = Tag_Velocity;
		PtrS<Mesh_Tri> pRB0(new Mesh_Tri(mV0, mT0, Discretization_Tri3, vnTraits));
		PtrS<Mesh_Tri> pRB1(new Mesh_Tri(mV1, mT1, Discretization_Tri3, vnTraits));

		this->m_pSimulableRB->SetupOptions().m_vpGeometries.push_back(dynamic_pointer_cast<IGeometry>(pRB0));
		this->m_pSimulableRB->SetupOptions().m_vpGeometries.push_back(dynamic_pointer_cast<IGeometry>(pRB1));
	
		PtrS<ParameterSet> pParSet(new ParameterSet());
		pParSet->InitFromYoungPoisson(1e9, 0.5, 1000);

		this->m_pSimulableRB->SetupOptions().m_vpMaterials.push_back(pParSet);
		this->m_pSimulableRB->SetupOptions().m_vpMaterials.push_back(pParSet);

		this->m_pSimulableRB->Setup();

		//////////////////////////////////////////////////////////////
		//////////////////// BOUNDARY CONDITIONS /////////////////////
		//////////////////////////////////////////////////////////////

		vector<PtrS<IBCondition>> vBC;

		// Gravity

		PtrS<BC_Gravity> pBCGravity(new BC_Gravity(m_pSimulableRB, Vector3d(0.0, -9.8, 0.0)));
		m_pSimulableRB->AddBoundaryCondition(pBCGravity);
		vBC.push_back(pBCGravity);

		// Fixed

		vector<IDoFSet*> vbbDOF(1);
		vector<VectorXd> vbbVal(1);
		IDoFSet* pDoF = pRB0->Traits().DoFSet(PhySim::Tag_DOF_0);
		vbbDOF.push_back(pDoF);
		vbbVal.push_back(pDoF->GetValue());

		PtrS<BC_FixDoF> pBCFixed(new BC_FixDoF(m_pSimulableRB, vbbDOF, vbbVal));
		m_pSimulableRB->AddBoundaryCondition(pBCFixed);
		vBC.push_back(pBCFixed);

		//////////////////////////////////////////////////
		//////////////////// PROBLEM /////////////////////
		//////////////////////////////////////////////////

		PtrS<OptimProblem_BasicDynamic> pDynamicProblem(new OptimProblem_BasicDynamic(m_pSimulableRB, 0.01));

		/////////////////////////////////////////////////
		//////////////////// SOLVER /////////////////////
		/////////////////////////////////////////////////

		OptimSolverOptions options;
		options.maxIters = 100; // Maximum iterations
		options.maxError = 1e-3; // The solver tolerance
		options.lsSolverType = LSSolverType::LS_EigenCG; // CGSolver solver
		options.lsMaxIters = 500;
		options.lSearch_type = LSearchType::LSearch_Simple; // Simple line-search

		PtrS<OptimSolver_USQP_LS> pUSQPSolver(new OptimSolver_USQP_LS(pDynamicProblem.get(), options));

		//////////////////////////////////////////////////////////////
		///////////////////////// SIMULATOR //////////////////////////
		//////////////////////////////////////////////////////////////

		this->m_pSimulator->Init(PtrS<Simulable>(m_pSimulableRB),
			dynamic_pointer_cast<OptimProblem>(pDynamicProblem),
			dynamic_pointer_cast<OptimSolver>(pUSQPSolver),
			vBC);

		this->m_isInit = true;

		return true;
	}

	bool Module_SimulatorRigidBody::UpdateTopology()
	{
		this->m_mobjects.clear();

		for (int i = 0; i < (int)this->m_pSimulableRB->Geometries().size(); ++i)
		{
			IMesh* pMesh = (IMesh*)this->m_pSimulableRB->Geometries()[i].get();

			MatrixXd mVX, mV0;
			pMesh->GetNodesTrait(mVX, Tag_Position_X);
			pMesh->GetNodesTrait(mV0, Tag_Position_0);

			MatrixXi mT;
			pMesh->GetElemMatrix(mT);

			IRenderMaterialRef matX = RenderMaterial_PhongPerObject::Create(ci::Color(1, 0, 0), ci::Color(1, 0, 0), 0.0);
			IRenderMaterialRef mat0 = RenderMaterial_PhongPerObject::Create(ci::Color(0, 0, 1), ci::Color(0, 0, 1), 0.0);
			IRenderGeometryRef geoX = RenderGeometry::CreateEdgeMesh(mVX, mT);
			IRenderGeometryRef geo0 = RenderGeometry::CreateEdgeMesh(mV0, mT);

			IObjectRef pObjWorld = Object::Create("World" + i, geoX, matX);
			pObjWorld->Render().m_edgeWidth = 5;
			pObjWorld->Render().m_solid = false;
			pObjWorld->Render().m_edge = true;
			pObjWorld->Render().m_show = true;

			IObjectRef pObjRest = Object::Create("Rest" + i, geoX, matX);
			pObjRest->Render().m_edgeWidth = 5;
			pObjRest->Render().m_solid = false;
			pObjRest->Render().m_edge = true;
			pObjRest->Render().m_show = true;

			this->Locker().lock();
			this->m_mobjects["World" + i] = pObjWorld;
			this->m_mobjects["Rest" + i] = pObjRest;
			this->Locker().unlock();
		}

		return true;
	}

	bool Module_SimulatorRigidBody::UpdateGeometry()
	{
		this->m_mobjects.clear();

		for (int i = 0; i < (int)this->m_pSimulableRB->Geometries().size(); ++i)
		{
			IMesh* pMesh = (IMesh*)this->m_pSimulableRB->Geometries()[i].get();

			MatrixXd mVX, mV0;
			pMesh->GetNodesTrait(mVX, Tag_Position_X);
			pMesh->GetNodesTrait(mV0, Tag_Position_0);

			this->Locker().lock();
			this->m_mobjects["World" + i]->Geometry()->Vertices() = mVX;
			this->m_mobjects["Rest" + i]->Geometry()->Vertices() = mV0;
			this->Locker().unlock();
		}

		return true;
	}

}
