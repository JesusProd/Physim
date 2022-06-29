////==========================================================
////
////	PhySim library. Generic library for physical simulation.
////
////	Authors:
////			Jesus Perez Rodriguez, jesusprod @ GitHub
////
////==========================================================
//
//#pragma once
//
//#include <PhySim/CommonIncludes.h>
//
//
//#include <PhySim/Physics/Elements/EnergyElement.h>
//
// namespace PhySim
//{
//	using namespace std;
//	using namespace Eigen;
//
//	class EnergyElement_Gravity : public EnergyElement
//	{
//
//	protected:
//		AMatrixSd m_mM;
//		Vector3d m_vgravity;
//
//	public:
//		EnergyElement_Gravity(Simulable_Mesh* pModel);
//		virtual ~EnergyElement_Gravity(void);
//
//		virtual void Init();
//
//		virtual Vector3d& Gravity() { return this->m_vgravity; }
//
//		virtual void ComputeAndStore_Energy();
//		virtual void ComputeAndStore_Gradient();
//	};
//
//}
//
//
