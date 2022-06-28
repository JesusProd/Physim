////==========================================================
////
////	PhySim library. Generic library for physical simulation.
////
////	Authors:
////			Jesus Perez Rodriguez, jesusprod @ GitHub
////
////==========================================================
//
//#include <PhySim/Physics/Elements/EnergyElement_Gravity.h>
//
//#include <PhySim/Geometry/Polytopes/Node.h>
//#include <PhySim/Physics/Simulables/Simulable_Mesh.h>
//
//#include <PhySim/Kinematics/KinematicsEle.h>
//
//namespace PhySim
//{
//	using namespace std;
//	using namespace Eigen;
//
//	EnergyElement_Gravity::EnergyElement_Gravity(Simulable_Mesh* pModel) : EnergyElement(pModel)
//	{
//		const vector<PtrS<KinematicsEle>>& vKine = pModel->GetDoFSets();
//
//		this->m_vDoF.reserve((int) vKine.size());
//		for (int i = 0; i < (int) vKine.size(); ++i)
//		{
//			if (!vKine[i]->Active()) 
//				continue; // Not DoF
//
//			this->m_vDoF.push_back(vKine[i].get());
//		}
//	
//		this->m_vgravity = Vector3d(0, -9.8, 0);
//	}
//
//	EnergyElement_Gravity::~EnergyElement_Gravity()
//	{
//		// Nothing to do here...
//	}
//
//	void EnergyElement_Gravity::Init()
//	{
//		this->m_pModel->GetInertia(this->m_mM);
//	}
//
//	void EnergyElement_Gravity::ComputeAndStore_Energy()
//	{
//		this->ComputeAndStore_Gradient();
//
//		VectorXd vx;
//		this->m_pModel->GetDOFVector(vx, Tag_Position_X);
//		this->m_energy = vx.dot(m_vgradient);
//	}
//
//	void EnergyElement_Gravity::ComputeAndStore_Gradient()
//	{
//		Simulable_Mesh* pModel = static_cast<Simulable_Mesh*>(m_pModel);
//
//		this->m_vgradient = VectorXd::Zero(m_pModel->GetNumFullDOF());
//
//		for (size_t i = 0; i < pModel->GetMesh().NumNodes(); ++i)
//		{
//			IDoFSet* pDoF = pModel->GetMesh().Nodes()[i]->Traits().Kinematics(Tag_DOF_0);
//			this->m_vgradient.block(pDoF->Offset(), 0, 3, 1) = -this->m_vgravity;
//		}
//
//		m_vgradient = m_mM*m_vgradient;
//	}
//}