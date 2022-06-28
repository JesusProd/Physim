//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement.h>

#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Kinematics/KinematicsEle.h>

#include <PhySim/Utils/DerivativeAssembler.h>
#include <PhySim/Utils/IOUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement::EnergyElement(Simulable* pModel, bool isDynamic)
	{
		this->m_pModel = pModel;

		m_energy = 0;
		m_vgradient = VectorXd::Zero(0);
		m_mHessian = MatrixXd::Zero(0, 0);

		this->m_vDoF.clear();

		m_intVolume = 0;

		this->m_pAssembler = make_shared<DerivativeAssembler>(isDynamic);
	}

	EnergyElement::~EnergyElement()
	{
		// Nothing to do...
	}

	void EnergyElement::Init()
	{
		// Nothing to do...
	}

	int EnergyElement::GetSupportSize() const
	{ 
		int N = 0; 
		for (int i = 0; i < this->m_vDoF.size(); ++i) 
			N += this->m_vDoF[i]->NumDim(); 
		return N; 
	}

	void EnergyElement::PreprocessAssembly(int layer)
	{
		if (this->m_vgradient.size() == 0 &&
			this->m_mHessian.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PreprocessMappings(vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), layer);
	}

	void EnergyElement::AssemblePreprocessedGradient(AVectorXd& vglobalGradient)
	{
		if (this->m_vgradient.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->AssemblePreprocessedGradient(this->m_vgradient, vglobalGradient);
	}

	void EnergyElement::AssemblePreprocessedHessian(AMatrixSd& mglobalHessian)
	{
		if (this->m_mHessian.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->AssemblePreprocessedHessian(this->m_mHessian, mglobalHessian);
	}

	void EnergyElement::Propagate_Gradient(AVectorXd& vglobalGradient)
	{
		if (this->m_vgradient.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PropagateGradient(this->m_vgradient, vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), vglobalGradient.Layer());
	}

	void EnergyElement::Assemble_Gradient(AVectorXd& vglobalGradient)
	{
		if (this->m_vgradient.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->AssembleGradient(vglobalGradient);
	}

	void EnergyElement::Propagate_Hessian(AMatrixSd& mglobalHessian)
	{
		if (this->m_mHessian.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PropagateHessian(this->m_mHessian, vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), mglobalHessian.Layer());
	}

	void EnergyElement::Assemble_Hessian(AMatrixSd& mglobalHessian)
	{
		if (this->m_mHessian.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->AssembleHessian(mglobalHessian);
	}

	void EnergyElement::PropagateAndAssemble_Gradient(AVectorXd& vglobalGradient)
	{
		if (this->m_vgradient.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PropagateAndAssembleGradient(this->m_vgradient, vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), vglobalGradient);
	}

	void EnergyElement::PropagateAndAssemble_Hessian(AMatrixSd& mglobalHessian)
	{
		if (this->m_mHessian.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PropagateAndAssembleHessian(this->m_mHessian, vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), mglobalHessian);
	}

	void EnergyElement::ComputeAndStore_Energy()
	{
		this->ComputeAndStore_Energy_Internal();
	}

	void EnergyElement::ComputeAndStore_Gradient()
	{
		if (this->m_useFDGradient)
			this->ComputeAndStore_Gradient_FD();
		else this->ComputeAndStore_Gradient_Internal();
	}

	void EnergyElement::ComputeAndStore_Hessian()
	{
		if (this->m_useFDHessian)
			this->ComputeAndStore_Hessian_FD();
		else this->ComputeAndStore_Hessian_Internal();
	}

	void EnergyElement::ComputeAndStore_Gradient_FD()
	{
		Real eps = 1e-6;

		int numDoF = (int)this->m_vDoF.size();
		m_vgradient.resize(GetSupportSize());

		int count = 0;

		for (int i = 0; i < numDoF; ++i)
		{
			KinematicsEle* pKine = static_cast<KinematicsEle*>(this->m_vDoF[i]);
			VectorXd vs0 = pKine->GetFullState();
			VectorXd vxF = pKine->GetPositionX();
			for (int j = 0; j < pKine->NumDim(); ++j)
			{
				// +
				vxF[j] += eps;
				pKine->SetPositionX(vxF);
				//pKine->UpdateKinematics();
				this->UpdateMechanics();
				this->ComputeAndStore_Energy();
				Real ep = this->m_energy;

				// -
				vxF[j] -= 2 * eps;
				pKine->SetPositionX(vxF);
				//pKine->UpdateKinematics();
				this->UpdateMechanics();
				this->ComputeAndStore_Energy();
				Real em = this->m_energy;

				// Estimate g
				m_vgradient[count] = (ep - em) / (2 * eps);

				// Restore state
				vxF[j] += eps;

				pKine->SetFullState(vs0);

				// Next!
				count++;
			}
		}

		this->UpdateMechanics();
		this->ComputeAndStore_Energy();
	}

	void EnergyElement::ComputeAndStore_Hessian_FD()
	{
		Real eps = 1e-6;

		int numDoF = (int)this->m_vDoF.size();

		int N = GetSupportSize();
		m_mHessian.resize(N, N);

		int count = 0;

		for (int i = 0; i < numDoF; ++i)
		{
			KinematicsEle* pKine = static_cast<KinematicsEle*>(this->m_vDoF[i]);
			VectorXd vs0 = pKine->GetFullState();
			VectorXd vxF = pKine->GetPositionX();
			for (int j = 0; j < pKine->NumDim(); ++j)
			{
				// +
				vxF[j] += eps;
				pKine->SetPositionX(vxF);
				//pKine->UpdateKinematics();
				this->UpdateMechanics();
				this->ComputeAndStore_Gradient();
				VectorXd vgp = this->m_vgradient;

				// -
				vxF[j] -= 2 * eps;
				pKine->SetPositionX(vxF);
				//pKine->UpdateKinematics();
				this->UpdateMechanics();
				this->ComputeAndStore_Gradient();
				VectorXd vgm = this->m_vgradient;

				// Estimate g
				this->m_mHessian.col(count) = (vgp - vgm) / (2 * eps);

				// Restore state
				vxF[j] += eps;

				pKine->SetFullState(vs0);

				// Next!
				count++;
			}
		}

		this->UpdateMechanics();
		this->ComputeAndStore_Gradient();
	}

	bool EnergyElement::TestLocalGradient()
	{
		this->ComputeAndStore_Gradient_FD();
		VectorXd vgF = this->m_vgradient;
		this->ComputeAndStore_Gradient_Internal();
		VectorXd vgA = this->m_vgradient;

		// Object
		VectorXd vgD = vgA - vgF;
		Real testNorm = vgA.norm();
		Real realNorm = vgF.norm();
		Real diffNorm = vgD.norm();
		if (realNorm < 1e-6)
		{
                  IOUtils::logTrace(
                      Verbosity::V1_Default,
                      "\n[INVALID] Finite difference gradient near zero: %f",
                      realNorm);

			return true;
		}
		else
		{
			if (diffNorm / realNorm > 1e-6)
			{
                    IOUtils::logTrace(Verbosity::V1_Default,
                                      "\n[FAILURE] %s gradient test error: %f",
                                      this->GetName().c_str(),
                                      diffNorm / realNorm);

				//MatrixXd mResult(vgA.size(), 3);
				//mResult.col(0) = vgA;
				//mResult.col(1) = vgF;
				//mResult.col(2) = vgD;

				//ostringstream str;
				//str << endl << "Analitic|Estimated|Difference:" << endl << mResult << endl;
				//logTrace(Verbosity::V1_Default, "%s", str.str().c_str());

				//logFile("csvGradientTest_F.csv", vectorToString_CSV(vgF));
				//logFile("csvGradientTest_A.csv", vectorToString_CSV(vgA));

				return false;
			}
			else
			{
                          IOUtils::logTrace(
                              Verbosity::V1_Default,
                              "\n[SUCCESS] %s gradient test error: %f",
                              this->GetName().c_str(), diffNorm / realNorm);

				return true;
			}
		}
	}

	bool EnergyElement::TestLocalHessian()
	{
		this->ComputeAndStore_Hessian_FD();
		MatrixXd mHF = this->m_mHessian;
		this->ComputeAndStore_Hessian_Internal();
		MatrixXd mHA = this->m_mHessian;

		// Object
		MatrixXd mHD = mHA - mHF;
		Real realNorm = mHF.norm();
		Real diffNorm = mHD.norm();
		if (realNorm < 1e-6)
		{
                  IOUtils::logTrace(
                      Verbosity::V1_Default,
                      "\n[INVALID] Finite difference Hessian near zero: %f",
                      realNorm);

			return true;
		}
		else
		{
			if (diffNorm / realNorm > 1e-6)
			{
                    IOUtils::logTrace(Verbosity::V1_Default,
                                      "\n[FAILURE] %s Hessian test error: %f",
                                      this->GetName().c_str(),
                                      diffNorm / realNorm);
				//ostringstream str;
				//str << endl << "Analitic Hessian:" << endl << mHA;
				//str << endl << "Estimated Hessian" << endl << mHF;
				//str << endl << "Difference" << endl << mHA - mHF;
				//logTrace(Verbosity::V1_Default, "%s", str.str().c_str());

				//logFile("csvHessianTest_F.csv", matrixToString_CSV(mHF));
				//logFile("csvHessianTest_A.csv", matrixToString_CSV(mHA));

				return false;
			}
			else
			{
                          IOUtils::logTrace(
                              Verbosity::V1_Default,
                              "\n[SUCCESS] %s Hessian test error: %f",
                              this->GetName().c_str(), diffNorm / realNorm);

				return true;
			}
		}
	}

}