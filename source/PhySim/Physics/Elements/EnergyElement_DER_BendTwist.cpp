//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, IST Austria
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_DER_BendTwist.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>

#include <PhySim/Physics/Simulables/Simulable_DER.h>

#include <PhySim/Utils/Auto/getBTEnergyOPT.h>
#include <PhySim/Utils/Auto/getBTGradientOPT.h>
#include <PhySim/Utils/Auto/getBTHessianOPT.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_DER_BendTwist::EnergyElement_DER_BendTwist(Simulable* pModel, const vector<Edge*>& vedges) : EnergyElement(pModel)
	{
		this->m_pModelDER = static_cast<Simulable_DER*>(pModel);

		this->m_vedges = vedges;

		this->m_vDoF.resize(5);
		this->m_vDoF[0] = vedges[0]->GetTail()->Traits().Kinematics(Tag_DOF_0);
		this->m_vDoF[1] = vedges[0]->GetHead()->Traits().Kinematics(Tag_DOF_0);
		this->m_vDoF[2] = vedges[1]->GetHead()->Traits().Kinematics(Tag_DOF_0);
		this->m_vDoF[3] = vedges[0]->Traits().Kinematics(Tag_DOF_0);
		this->m_vDoF[4] = vedges[1]->Traits().Kinematics(Tag_DOF_0);

		this->m_vgradient.resize(11);
		this->m_mHessian.resize(11, 11);

		this->m_mDeDx.resize(8, 11);
		this->m_mDeDx.setZero();
		this->m_mDeDx(6, 9) = 1.0;
		this->m_mDeDx(7, 10) = 1.0;
		this->m_mDeDx.block(0, 0, 3, 3) = -Matrix3d::Identity();
		this->m_mDeDx.block(3, 3, 3, 3) = -Matrix3d::Identity();
		this->m_mDeDx.block(0, 3, 3, 3) = Matrix3d::Identity();
		this->m_mDeDx.block(3, 6, 3, 3) = Matrix3d::Identity();

		this->Init();
	}

	EnergyElement_DER_BendTwist::~EnergyElement_DER_BendTwist(void)
	{
		// Nothing to do here...
	}

	void EnergyElement_DER_BendTwist::Init()
	{
		this->m_intVolume = 0.5*this->m_vedges[0]->VolumeBasis(Tag_Position_0) + 
							0.5*this->m_vedges[1]->VolumeBasis(Tag_Position_0);

		this->m_bend0 = this->ComputeBendStrain(Tag_Position_0);
		this->m_twist0 = this->ComputeTwistStrain(Tag_Position_0);
	}

	void EnergyElement_DER_BendTwist::ComputeAndStore_Energy_Internal()
	{
		// Get data

		Vector3d e1 = this->m_vedges[0]->Vector(Tag_Position_X);
		Vector3d e2 = this->m_vedges[1]->Vector(Tag_Position_X);

		const Frame3d& Fr_e = this->m_vedges[0]->Traits().Frame3d(Tag_Frame_X);
		const Frame3d& Fr_f = this->m_vedges[1]->Traits().Frame3d(Tag_Frame_X);
		const Real& t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_X);
		const Real& t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_X);
		const Real& rt = this->m_vedges[0]->GetHead()->Traits().Double(Tag_Twist_X);

		// Get material

		ParameterSet& parameters0 = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_vedges[0]->ID());
		ParameterSet& parameters1 = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_vedges[1]->ID());

		Real vshear[2] = {
			parameters0[ParameterSet::Param_Lame2],
			parameters1[ParameterSet::Param_Lame2]
		};
		Real vyoung[2] = {
			parameters0[ParameterSet::Param_Young],
			parameters1[ParameterSet::Param_Young]
		};

		// Get radius

		Real vradh[2] = {
			this->m_vedges[0]->Traits().Double(Tag_Size_0),
			this->m_vedges[1]->Traits().Double(Tag_Size_0)
		};
		Real vradw[2] = {
			this->m_vedges[0]->Traits().Double(Tag_Size_1),
			this->m_vedges[1]->Traits().Double(Tag_Size_1)
		};

		// Compute

		this->m_energy = getBTEnergyOPT(
			vshear, vyoung, vradw, vradh,
			this->m_vedges[0]->VolumeBasis(Tag_Position_0),
			this->m_vedges[1]->VolumeBasis(Tag_Position_0),
			this->m_bend0.data(), this->m_twist0,
			Fr_e.nor.data(), Fr_e.tan.data(),
			Fr_f.nor.data(), Fr_f.tan.data(),
			rt, e1.data(), e2.data(), t_e, t_f);
	}

	void EnergyElement_DER_BendTwist::ComputeAndStore_Gradient_Internal()
	{
		// Get data

		Vector3d e1 = this->m_vedges[0]->Vector(Tag_Position_X);
		Vector3d e2 = this->m_vedges[1]->Vector(Tag_Position_X);

		const Frame3d& Fr_e = this->m_vedges[0]->Traits().Frame3d(Tag_Frame_X);
		const Frame3d& Fr_f = this->m_vedges[1]->Traits().Frame3d(Tag_Frame_X);
		const Real& t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_X);
		const Real& t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_X);
		const Real& rt = this->m_vedges[0]->GetHead()->Traits().Double(Tag_Twist_X);

		// Get material

		ParameterSet& parameters0 = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_vedges[0]->ID());
		ParameterSet& parameters1 = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_vedges[1]->ID());

		Real vshear[2] = {
			parameters0[ParameterSet::Param_Lame2],
			parameters1[ParameterSet::Param_Lame2]
		};
		Real vyoung[2] = {
			parameters0[ParameterSet::Param_Young],
			parameters1[ParameterSet::Param_Young]
		};

		// Get radius

		Real vradh[2] = {
			this->m_vedges[0]->Traits().Double(Tag_Size_0),
			this->m_vedges[1]->Traits().Double(Tag_Size_0)
		};
		Real vradw[2] = {
			this->m_vedges[0]->Traits().Double(Tag_Size_1),
			this->m_vedges[1]->Traits().Double(Tag_Size_1)
		};

		// Compute

		VectorXd vg(8);

		getBTGradientOPT(
			vshear, vyoung, vradw, vradh,
			this->m_vedges[0]->VolumeBasis(Tag_Position_0),
			this->m_vedges[1]->VolumeBasis(Tag_Position_0),
			this->m_bend0.data(), this->m_twist0,
			Fr_e.nor.data(), Fr_e.tan.data(),
			Fr_f.nor.data(), Fr_f.tan.data(),
			rt, e1.data(), e2.data(), t_e, t_f,
			vg.data());

		this->m_vgradient = this->m_mDeDx.transpose()*vg;

		//{
		//	Real eps = 1e-6;
		//	Real gt_e, gt_f;

		//	gt_e = (getBTEnergyOPT(
		//		vshear, vyoung, vradw, vradh,
		//		this->m_vedges[0]->VolumeBasis(Tag_Position_0),
		//		this->m_vedges[1]->VolumeBasis(Tag_Position_0),
		//		this->m_bend0.data(), this->m_twist0,
		//		Fr_e.nor.data(), Fr_e.tan.data(),
		//		Fr_f.nor.data(), Fr_f.tan.data(),
		//		rt, e1.data(), e2.data(), t_e + eps, t_f)
		//		-
		//		getBTEnergyOPT(
		//			vshear, vyoung, vradw, vradh,
		//			this->m_vedges[0]->VolumeBasis(Tag_Position_0),
		//			this->m_vedges[1]->VolumeBasis(Tag_Position_0),
		//			this->m_bend0.data(), this->m_twist0,
		//			Fr_e.nor.data(), Fr_e.tan.data(),
		//			Fr_f.nor.data(), Fr_f.tan.data(),
		//			rt, e1.data(), e2.data(), t_e - eps, t_f)
		//		)/(2*eps);

		//	gt_f = (getBTEnergyOPT(
		//		vshear, vyoung, vradw, vradh,
		//		this->m_vedges[0]->VolumeBasis(Tag_Position_0),
		//		this->m_vedges[1]->VolumeBasis(Tag_Position_0),
		//		this->m_bend0.data(), this->m_twist0,
		//		Fr_e.nor.data(), Fr_e.tan.data(),
		//		Fr_f.nor.data(), Fr_f.tan.data(),
		//		rt, e1.data(), e2.data(), t_e, t_f + eps)
		//		-
		//		getBTEnergyOPT(
		//			vshear, vyoung, vradw, vradh,
		//			this->m_vedges[0]->VolumeBasis(Tag_Position_0),
		//			this->m_vedges[1]->VolumeBasis(Tag_Position_0),
		//			this->m_bend0.data(), this->m_twist0,
		//			Fr_e.nor.data(), Fr_e.tan.data(),
		//			Fr_f.nor.data(), Fr_f.tan.data(),
		//			rt, e1.data(), e2.data(), t_e, t_f - eps)
		//		) / (2 * eps);

		//	logTrace(Verbosity::V1_Default, "\n[DEBUG] Angle gradient E error: %.6e vs %.6e", gt_e, m_vgradient[9]);
		//	logTrace(Verbosity::V1_Default, "\n[DEBUG] Angle gradient F error: %.6e vs %.6e", gt_f, m_vgradient[10]);
		//}
	}

	void EnergyElement_DER_BendTwist::ComputeAndStore_Hessian_Internal()
	{
		// Get data

		Vector3d e1 = this->m_vedges[0]->Vector(Tag_Position_X);
		Vector3d e2 = this->m_vedges[1]->Vector(Tag_Position_X);

		const Frame3d& Fr_e = this->m_vedges[0]->Traits().Frame3d(Tag_Frame_X);
		const Frame3d& Fr_f = this->m_vedges[1]->Traits().Frame3d(Tag_Frame_X);
		const Real& t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_X);
		const Real& t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_X);
		const Real& rt = this->m_vedges[0]->GetHead()->Traits().Double(Tag_Twist_X);

		// Get material

		ParameterSet& parameters0 = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_vedges[0]->ID());
		ParameterSet& parameters1 = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_vedges[1]->ID());

		Real vshear[2] = {
			parameters0[ParameterSet::Param_Lame2],
			parameters1[ParameterSet::Param_Lame2]
		};
		Real vyoung[2] = {
			parameters0[ParameterSet::Param_Young],
			parameters1[ParameterSet::Param_Young]
		};

		// Get radius

		Real vradh[2] = {
			this->m_vedges[0]->Traits().Double(Tag_Size_0),
			this->m_vedges[1]->Traits().Double(Tag_Size_0)
		};
		Real vradw[2] = {
			this->m_vedges[0]->Traits().Double(Tag_Size_1),
			this->m_vedges[1]->Traits().Double(Tag_Size_1)
		};

		// Compute

		MatrixXd mH(8,8);

		getBTHessianOPT(
			vshear, vyoung, vradw, vradh,
			this->m_vedges[0]->VolumeBasis(Tag_Position_0),
			this->m_vedges[1]->VolumeBasis(Tag_Position_0),
			this->m_bend0.data(), this->m_twist0,
			Fr_e.nor.data(), Fr_e.tan.data(),
			Fr_f.nor.data(), Fr_f.tan.data(),
			rt, e1.data(), e2.data(), t_e, t_f,
			mH.transpose().data());

		this->m_mHessian = this->m_mDeDx.transpose()*mH*this->m_mDeDx;
	}

	Vector2d EnergyElement_DER_BendTwist::ComputeBendStrain(Tag s)
	{
		if (s != Tag_Position_0 && s != Tag_Position_X)
			throw PhySim::exception("Invalid Tag: [Position_0 | Position_X]");

		Vector3d e;
		Vector3d f;
		Frame3d Fr_e;
		Frame3d Fr_f;
		Real t_e;
		Real t_f;
		Real rt;

		if (s == Tag_Position_0)
		{
			e = this->m_vedges[0]->Vector(Tag_Position_0);
			f = this->m_vedges[1]->Vector(Tag_Position_0);

			Fr_e = this->m_vedges[0]->Traits().Frame3d(Tag_Frame_0);
			Fr_f = this->m_vedges[1]->Traits().Frame3d(Tag_Frame_0);
			t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_0);
			t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_0);

			rt = 0;
		}

		if (s == Tag_Position_X)
		{
			e = this->m_vedges[0]->Vector(Tag_Position_X);
			f = this->m_vedges[1]->Vector(Tag_Position_X);

			Fr_e = this->m_vedges[0]->Traits().Frame3d(Tag_Frame_X);
			Fr_f = this->m_vedges[1]->Traits().Frame3d(Tag_Frame_X);
			t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_X);
			t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_X);

			rt = this->m_vedges[0]->GetHead()->Traits().Double(Tag_Twist_X);
		}

		// Compute curvature binormal

		Vector3d t0 = e.normalized();
		Vector3d t1 = f.normalized();
		Vector3d KB = (t0.cross(t1)) * 2 * (1 / (1 + t0.dot(t1)));

		// Compute bending strain
		 
		Vector3d m1e = Fr_e.bin;
		Vector3d m1f = Fr_f.bin;
		Vector3d m2e = Fr_e.nor;
		Vector3d m2f = Fr_f.nor;
		Vector2d kappa;
		kappa[0] = (m1e + m1f).dot(KB) * 0.5;
		kappa[1] = (m2e + m2f).dot(KB) * -0.5;

		return kappa;
	}

	Real EnergyElement_DER_BendTwist::ComputeTwistStrain(Tag s)
	{
		if (s != Tag_Position_0 && s != Tag_Position_X)
			throw PhySim::exception("Invalid Tag: [Position_0 | Position_X]");

		Real t_e;
		Real t_f;
		Real rt;

		if (s == Tag_Position_0)
		{
			t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_0); // Material twist
			t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_0); // Material twist
			rt = 0;
		}

		if (s == Tag_Position_X)
		{
			t_e = this->m_vedges[0]->Traits().Double(Tag_Angle_X); // Material twist
			t_f = this->m_vedges[1]->Traits().Double(Tag_Angle_X); // Material twist
			rt = this->m_vedges[0]->GetHead()->Traits().Double(Tag_Twist_X); // Reference twist
		}

		return t_f - t_e + rt;
	}

}