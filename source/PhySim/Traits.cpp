//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

//#include <PhySim/CommonIncludes.h>
//
//
//#include <PhySim/Traits.h>
//
//namespace PhySim
//{
//	using namespace std;
//	using namespace Eigen;
//
//	PhySim::IDoFSet* TraitSet::DoFSet(Tag s) 
//	{
//		return *static_cast<PhySim::IDoFSet**>(m_mData.at(s)->pValue);
//	}
//
//	PhySim::KinematicsEle* TraitSet::Kinematics(Tag s)
//	{
//		return *static_cast<PhySim::KinematicsEle**>(m_mData.at(s)->pValue);
//	}
//
//	const PhySim::IDoFSet* TraitSet::DoFSet(Tag s) const
//	{
//		return *static_cast<PhySim::IDoFSet**>(m_mData.at(s)->pValue);
//	}
//
//	const PhySim::KinematicsEle* TraitSet::Kinematics(Tag s) const
//	{
//		return *static_cast<PhySim::KinematicsEle**>(m_mData.at(s)->pValue);
//	}
//
//}