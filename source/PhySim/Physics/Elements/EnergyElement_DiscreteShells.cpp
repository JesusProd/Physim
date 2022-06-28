//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_DiscreteShells.h>

#include <PhySim/Geometry/Meshes/Mesh_Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_DiscreteShells::EnergyElement_DiscreteShells(Simulable* pModel, Edge* pEdge) : EnergyElement(pModel)
	{
		this->m_pEdge = pEdge;

		this->m_pMesh = static_cast<Mesh_Face*>(pEdge->GetMesh());

		this->m_vnodes.resize(4);
		Mesh_Face::HE_Edge* pheEdge = m_pMesh->GetHEEdge(pEdge->ID());
		this->m_vnodes[0] = pheEdge->m_pheHalf0->m_pheNode->m_pHandle;
		this->m_vnodes[1] = pheEdge->m_pheHalf1->m_pheNode->m_pHandle;
		this->m_vnodes[2] = pheEdge->m_pheHalf0->m_pheNext->m_pheNode->m_pHandle;
		this->m_vnodes[3] = pheEdge->m_pheHalf1->m_pheNext->m_pheNode->m_pHandle;

		this->m_vDoF.resize(4);
		this->m_vDoF[0] = m_vnodes[0]->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[1] = m_vnodes[1]->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[2] = m_vnodes[2]->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[3] = m_vnodes[3]->Traits().Kinematics(Tag::Tag_DOF_0);

		this->m_vgradient.resize(12);
		this->m_mHessian.resize(12,12);
	}

	EnergyElement_DiscreteShells::~EnergyElement_DiscreteShells(void)
	{
		// Nothing to do here...
	}

	void EnergyElement_DiscreteShells::Init()
	{
		this->m_shape0 = this->ComputeShape(Tag_Position_0);
		this->m_theta0 = this->ComputeDihedral(Tag_Position_0);
		this->m_intVolume = this->m_shape0;
	}

	void EnergyElement_DiscreteShells::ComputeAndStore_Energy_Internal()
	{
		Vector3d x0 = this->m_vnodes[0]->Traits().Vector3d(Tag_Position_X);
		Vector3d x1 = this->m_vnodes[1]->Traits().Vector3d(Tag_Position_X);
		Vector3d x2 = this->m_vnodes[2]->Traits().Vector3d(Tag_Position_X);
		Vector3d x3 = this->m_vnodes[3]->Traits().Vector3d(Tag_Position_X);

		Real kB = 0;

		if (this->m_pEdge->Traits().HasTrait(Tag_Mat_BendingK))
		{
			kB = this->m_pEdge->Traits().Double(Tag_Mat_BendingK);
		}
		else 
		{
			Real Y = this->m_pEdge->Traits().Double(Tag_Mat_Young);
			Real v = this->m_pEdge->Traits().Double(Tag_Mat_Poisson);
			Real t = this->m_pEdge->Traits().Double(Tag_Mat_Thickness);
			kB = (Y*t*t*t) / (12 * (1 - v*v));
		}

		Real theta0 = this->m_theta0;
		Real shape0 = this->m_shape0;

		{
#include "../Maple/DiscreteShellsBending_Energy.mcg"

			this->m_energy = t105;
		}
	}

	void EnergyElement_DiscreteShells::ComputeAndStore_Gradient_Internal()
	{
		Vector3d x0 = this->m_vnodes[0]->Traits().Vector3d(Tag_Position_X);
		Vector3d x1 = this->m_vnodes[1]->Traits().Vector3d(Tag_Position_X);
		Vector3d x2 = this->m_vnodes[2]->Traits().Vector3d(Tag_Position_X);
		Vector3d x3 = this->m_vnodes[3]->Traits().Vector3d(Tag_Position_X);

		Real kB = 0;

		if (this->m_pEdge->Traits().HasTrait(Tag_Mat_BendingK))
		{
			kB = this->m_pEdge->Traits().Double(Tag_Mat_BendingK);
		}
		else
		{
			Real Y = this->m_pEdge->Traits().Double(Tag_Mat_Young);
			Real v = this->m_pEdge->Traits().Double(Tag_Mat_Poisson);
			Real t = this->m_pEdge->Traits().Double(Tag_Mat_Thickness);
			kB = (Y*t*t*t) / (12 * (1 - v*v));
		}

		Real theta0 = this->m_theta0;
		Real shape0 = this->m_shape0;

		Real vg[12];

		{
#include "../Maple/DiscreteShellsBending_Gradient.mcg"

			for (int i = 0; i < 12; ++i)
				m_vgradient[i] = vg[i];
		}

		//Real theta = this->ComputeDihedral(Tag_Position_X);

		//Mesh_Face::HE_Edge* pheEdge = this->m_pMesh->GetHEEdge(this->m_pEdge->ID());
		//Vector3d vn0 = pheEdge->m_pheHalf0->m_pheFace->m_pHandle->Normal(Tag_Position_X);
		//Vector3d vn1 = pheEdge->m_pheHalf1->m_pheFace->m_pHandle->Normal(Tag_Position_X);
		//Real h0 = this->helperDistance(x2, x0, x1);
		//Real h1 = this->helperDistance(x3, x0, x1);
		//Vector2d w0 = this->helperBarycentric(x2, x0, x1);
		//Vector2d w1 = this->helperBarycentric(x3, x0, x1);
		//VectorXd dTheta(12);
		//dTheta.segment(0,3) = -(w0[0] * vn0 / h0 + w1[0] * vn1 / h1);
		//dTheta.segment(3,3) = -(w0[1] * vn0 / h0 + w1[1] * vn1 / h1);
		//dTheta.segment(6,3) = vn0 / h0;
		//dTheta.segment(9,3) = vn1 / h1;

		//this->m_vgradient = kB*this->m_shape0*(theta - m_theta0)*dTheta / 2;
	}

	void EnergyElement_DiscreteShells::ComputeAndStore_Hessian_Internal()
	{
		Vector3d x0 = this->m_vnodes[0]->Traits().Vector3d(Tag_Position_X);
		Vector3d x1 = this->m_vnodes[1]->Traits().Vector3d(Tag_Position_X);
		Vector3d x2 = this->m_vnodes[2]->Traits().Vector3d(Tag_Position_X);
		Vector3d x3 = this->m_vnodes[3]->Traits().Vector3d(Tag_Position_X);

		Real kB = 0;

		if (this->m_pEdge->Traits().HasTrait(Tag_Mat_BendingK))
		{
			kB = this->m_pEdge->Traits().Double(Tag_Mat_BendingK);
		}
		else
		{
			Real Y = this->m_pEdge->Traits().Double(Tag_Mat_Young);
			Real v = this->m_pEdge->Traits().Double(Tag_Mat_Poisson);
			Real t = this->m_pEdge->Traits().Double(Tag_Mat_Thickness);
			kB = (Y*t*t*t) / (12 * (1 - v*v));
		}

		Real theta0 = this->m_theta0;
		Real shape0 = this->m_shape0;

		{
			Real mHspd[12][12];

#include "../Maple/DiscreteShellsBending_HessianSPD.mcg"

			for (int i = 0; i < 12; ++i)
				for (int j = 0; j < 12; ++j)
					m_mHessian(i, j) = mHspd[i][j];
		}

		//Real theta = this->ComputeDihedral(Tag_Position_X);

		//Mesh_Face* pMeshFace = static_cast<Mesh_Face*>(this->m_pMesh);
		//Mesh_Face::HE_Edge* pheEdge = pMeshFace->GetHEEdge(this->m_pEdge->ID());
		//Vector3d vn0 = pheEdge->m_pheHalf0->m_pheFace->m_pHandle->Normal(Tag_Position_X);
		//Vector3d vn1 = pheEdge->m_pheHalf1->m_pheFace->m_pHandle->Normal(Tag_Position_X);
		//Real h0 = this->helperDistance(x2, x0, x1);
		//Real h1 = this->helperDistance(x3, x0, x1);
		//Vector2d w0 = this->helperBarycentric(x2, x0, x1);
		//Vector2d w1 = this->helperBarycentric(x3, x0, x1);
		//VectorXd dTheta(12);
		//dTheta.segment(0, 3) = -(w0[0] * vn0 / h0 + w1[0] * vn1 / h1);
		//dTheta.segment(3, 3) = -(w0[1] * vn0 / h0 + w1[1] * vn1 / h1);
		//dTheta.segment(6, 3) = vn0 / h0;
		//dTheta.segment(9, 3) = vn1 / h1;

		//this->m_mHessian = kB*this->m_shape0*dTheta*dTheta.transpose()/2;
	}

	Real EnergyElement_DiscreteShells::ComputeShape(Tag s)
	{
		Vector3d x0 = this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d x1 = this->m_vnodes[1]->Traits().Vector3d(s);
		Vector3d x2 = this->m_vnodes[2]->Traits().Vector3d(s);
		Vector3d x3 = this->m_vnodes[3]->Traits().Vector3d(s);

		{
#include "../Maple/DiscreteShellsBending_Shape.mcg";

			return t46;
		}

		//Mesh_Face::HE_Edge* pheEdge = this->m_pMesh->GetHEEdge(this->m_pEdge->ID());
		//Real A =
		//	pheEdge->m_pheHalf0->m_pheFace->m_pHandle->Area(s) +
		//	pheEdge->m_pheHalf1->m_pheFace->m_pHandle->Area(s);
		//Real L = this->m_pEdge->Length(s);
		//return (L*L) / (2*A);
	}

	Real EnergyElement_DiscreteShells::ComputeDihedral(Tag s)
	{
		Vector3d x0 = this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d x1 = this->m_vnodes[1]->Traits().Vector3d(s);
		Vector3d x2 = this->m_vnodes[2]->Traits().Vector3d(s);
		Vector3d x3 = this->m_vnodes[3]->Traits().Vector3d(s);

		{
#include "../Maple/DiscreteShellsBending_Dihedral.mcg";

			return t101;
		}

		//Real angle = this->m_pMesh->ComputeDihedralAngle(this->m_pEdge, s);

		//return angle;
	}

	Real EnergyElement_DiscreteShells::helperDistance(const Vector3d& vx, const Vector3d& va, const Vector3d& vb)
	{
		Vector3d ve = vb - va;
		Vector3d xp = ve*ve.dot(vx - va) / ve.dot(ve);
		Real D = ((vx - va) - xp).norm();
		return max(D, 1.0e-3*ve.norm());
	}

	Vector2d EnergyElement_DiscreteShells::helperBarycentric(const Vector3d& vx, const Vector3d& va, const Vector3d& vb)
	{
		Vector3d ve = vb - va;
		double t = ve.dot(vx - va) / ve.dot(ve);
		return Vector2d(1 - t, t);
	}

}