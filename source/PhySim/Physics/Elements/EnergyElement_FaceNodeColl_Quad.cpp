//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_FaceNodeColl_Quad.h>

#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

EnergyElement_FaceNodeColl_Quad::EnergyElement_FaceNodeColl_Quad(
    Simulable* pModel,
    Face* pFace,
    Node* pNode)
    : EnergyElement(pModel) {
  this->m_vDoF.resize(4);
  this->m_vDoF[0] = pFace->Nodes()[0]->Traits().Kinematics(Tag::Tag_DOF_0);
  this->m_vDoF[1] = pFace->Nodes()[1]->Traits().Kinematics(Tag::Tag_DOF_0);
  this->m_vDoF[2] = pFace->Nodes()[2]->Traits().Kinematics(Tag::Tag_DOF_0);
  this->m_vDoF[3] = pNode->Traits().Kinematics(Tag::Tag_DOF_0);

  this->m_vgradient.resize(12);
  this->m_mHessian.resize(12, 12);
}

EnergyElement_FaceNodeColl_Quad::~EnergyElement_FaceNodeColl_Quad(void) {
  // Nothing to do here...
}

void EnergyElement_FaceNodeColl_Quad::ComputeAndStore_Energy_Internal() {
  //		const Vector3d& x0 =
  //this->m_vDoF[0]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x1 =
  //this->m_vDoF[1]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x2 =
  //this->m_vDoF[2]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x3 =
  //this->m_vDoF[3]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		Vector3d e0
  //= (x1 - x0).normalized(); 		Vector3d e1 = (x2 - x0).normalized(); 		Vector3d e3
  //= x3 - x0; 		Real D = e0.cross(e1).dot(e3); 		Real T =
  //(*this->m_pMaterial)[ParameterSet::Param_CollT]; 		Real K =
  //(*this->m_pMaterial)[ParameterSet::Param_CollK]; 		if (D > T)
  //		{
  //			this->m_energy = 0;
  //			return; // Oneside
  //		}
  //
  //#include "../include/PhySim/Utils/Auto/FaceNodeCollQuad_Energy.mcg"
  //
  //		this->m_energy = t47;
}

void EnergyElement_FaceNodeColl_Quad::ComputeAndStore_Gradient_Internal() {
  //		const Vector3d& x0 =
  //this->m_vDoF[0]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x1 =
  //this->m_vDoF[1]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x2 =
  //this->m_vDoF[2]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x3 =
  //this->m_vDoF[3]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		Vector3d e0
  //= (x1 - x0).normalized(); 		Vector3d e1 = (x2 - x0).normalized(); 		Vector3d e3
  //= x3 - x0; 		Real D = e0.cross(e1).dot(e3); 		Real T =
  //(*this->m_pMaterial)[ParameterSet::Param_CollT]; 		Real K =
  //(*this->m_pMaterial)[ParameterSet::Param_CollK]; 		if (D > T)
  //		{
  //			this->m_vgradient.setZero(12);
  //			return; // Oneside (return)
  //		}
  //
  //		Real vgx[12];
  //
  //#include "../include/PhySim/Utils/Auto/FaceNodeCollQuad_Gradient.mcg"
  //
  //		for (int i = 0; i < 12; ++i)
  //			m_vgradient(i) = vgx[i];
}

void EnergyElement_FaceNodeColl_Quad::ComputeAndStore_Hessian_Internal() {
  //		const Vector3d& x0 =
  //this->m_vDoF[0]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x1 =
  //this->m_vDoF[1]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x2 =
  //this->m_vDoF[2]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		const
  //Vector3d& x3 =
  //this->m_vDoF[3]->Geometry()->Traits().Vector3d(Tag::Tag_Position_X); 		Vector3d e0
  //= (x1 - x0).normalized(); 		Vector3d e1 = (x2 - x0).normalized(); 		Vector3d e3
  //= x3 - x0; 		Real D = e0.cross(e1).dot(e3); 		Real T =
  //(*this->m_pMaterial)[ParameterSet::Param_CollT]; 		Real K =
  //(*this->m_pMaterial)[ParameterSet::Param_CollK]; 		if (D > T)
  //		{
  //			this->m_mHessian.setZero(12,12);
  //			return; // Oneside (return)
  //		}
  //
  //		Real mHx[12][12];
  //
  //#include "../include/PhySim/Utils/Auto/FaceNodeCollQuad_Hessian.mcg"
  //
  //		for (int i = 0; i < 12; ++i)
  //			for (int j = 0; j < 12; ++j)
  //				m_mHessian(i,j) = mHx[i][j];
}

}  // namespace PhySim