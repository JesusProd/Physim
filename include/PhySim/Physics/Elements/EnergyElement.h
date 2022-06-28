//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>


#include <PhySim/Physics/Elements/IEnergyElement.h>


namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class DerivativeAssembler;
	class ParameterEnergy;
	class Simulable;
	class KinematicsEle;

	class EnergyElement : public IEnergyElement
	{
		friend class ParameterEnergy;

	protected:
		Simulable* m_pModel;

		vector<KinematicsEle*> m_vDoF;

		Real m_energy;
		VectorXd m_vgradient;		// Local element gradient
		MatrixXd m_mHessian;		// Local element Hessian

		PtrS<DerivativeAssembler> m_pAssembler;

		Real m_intVolume;

		bool m_useFDGradient = false;
		bool m_useFDHessian = false;

	public:

		EnergyElement(Simulable* pModel, bool isDynamic = false);
		virtual ~EnergyElement(void);

		virtual string GetName() const override { return "[BASE]"; };

		virtual int GetSupportSize() const override;

		inline virtual const vector<KinematicsEle*> GetSupportDoF() const { return this->m_vDoF; }

		virtual void Init();

		virtual void UpdateKinematics() override {}
		virtual void UpdateMechanics() override {}

		virtual void ComputeAndStore_Energy() override;
		virtual void ComputeAndStore_Gradient() override;
		virtual void ComputeAndStore_Hessian() override;

		inline Real GetLocalEnergy() const override { return this->m_energy; }
		inline const VectorXd& GetLocalGradient() const override { return this->m_vgradient; }
		inline const MatrixXd& GetLocalHessian() const override { return this->m_mHessian; }

		virtual void PreprocessAssembly(int layer) override;
		virtual void AssemblePreprocessedGradient(AVectorXd& vglobalGradient) override;
		virtual void AssemblePreprocessedHessian(AMatrixSd& mglobalHessian) override;

		virtual void Propagate_Gradient(AVectorXd& vglobalGradient) override;
		virtual void Propagate_Hessian(AMatrixSd& mglobalHessian) override;
		virtual void Assemble_Gradient(AVectorXd& vglobalGradient) override;
		virtual void Assemble_Hessian(AMatrixSd& mglobalHessian) override;
		virtual void PropagateAndAssemble_Gradient(AVectorXd& vglobalGradient) override;
		virtual void PropagateAndAssemble_Hessian(AMatrixSd& mglobalMatrix) override;

		virtual int GetHessianSize() const override { return (int) pow(this->GetSupportSize(),2); };

		inline virtual const Real& GetIntegrationVolume() const override { return this->m_intVolume; }
		inline virtual void SetIntegrationVolume(const Real& iv) override { this->m_intVolume = iv; }

		virtual void ComputeAndStore_Gradient_FD();
		virtual void ComputeAndStore_Hessian_FD();

		virtual bool TestLocalGradient() override;
		virtual bool TestLocalHessian() override;

		virtual bool GetUseFDGradient() const { return this->m_useFDGradient; }
		virtual void SetUseFDGradient(bool f) { this->m_useFDGradient = f; }

		virtual bool GetUseFDHessian() const { return this->m_useFDHessian; }
		virtual void SetUseFDHessian(bool f) { this->m_useFDHessian = f; }

	protected:
		virtual void ComputeAndStore_Energy_Internal() {}
		virtual void ComputeAndStore_Gradient_Internal() {}
		virtual void ComputeAndStore_Hessian_Internal() {}
	};
}
