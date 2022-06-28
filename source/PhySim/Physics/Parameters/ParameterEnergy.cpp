////==========================================================
////
////	PhySim library. Generic library for physical simulation.
////
////	Authors:
////			Jesus Perez Rodriguez, jesusprod @ GitHub
////
////==========================================================
//
//#include <PhySim/Physics/Parameters/ParameterEnergy.h>
//
//#include <PhySim/Utils/DerivativeAssembler.h>
//
//namespace PhySim
//{
//	using namespace std;
//	using namespace Eigen;
//
//	ParameterEnergy::ParameterEnergy(EnergyElement* pEle)
//	{
//		this->m_pElement = pEle;
//
//		m_vDEDp = VectorXd::Zero(0);
//		m_mDE2Dxp = MatrixXd::Zero(0, 0);
//		m_mDE2Dpp = MatrixXd::Zero(0, 0);
//
//		this->m_vDoF.clear();
//	}
//
//	int ParameterEnergy::GetSupportSize() const
//	{
//		int N = 0;
//		for (int i = 0; i < this->m_vDoF.size(); ++i)
//			N += this->m_vDoF[i]->NumDim();
//		return N;
//	}
//
//	void ParameterEnergy::AssembleGlobal_DEDp(AVectorXd& vglobalDEDp)
//	{
//		if (this->m_vDEDp.size() == 0)
//			return; // Ignore assembly
//		
//		this->m_pElement->m_pAssembler->PropagateAndAssembleGradient(this->m_vDEDp, this->m_vDoF, vglobalDEDp);
//	}
//
//	void ParameterEnergy::AssembleGlobal_DE2Dxp(AMatrixSd& mglobalDE2Dxp)
//	{
//		if (this->m_mDE2Dxp.size() == 0)
//			return; // Ignore assembly
//
//		this->m_pElement->m_pAssembler->AssembleJacobian(this->m_mDE2Dxp, this->m_vDoF, mglobalDE2Dxp);
//	}
//
//	void ParameterEnergy::AssembleGlobal_DE2Dpp(AMatrixSd& mglobalDE2Dxp)
//	{
//		if (this->m_mDE2Dxp.size() == 0)
//			return; // Ignore assembly
//
//		//this->m_pElement->m_pAssembler->AssembleHessian(this->m_mDE2Dpp, this->m_vDoF, mglobalDE2Dxp);
//	}
//
//	bool ParameterEnergy::TestLocalDEDp()
//	{
//		Real eps = 1e-6;
//
//		int numDoF = (int)this->m_vDoF.size();
//
//		VectorXd vgF(this->GetSupportSize());
//
//		int count = 0;
//
//		for (int i = 0; i < numDoF; ++i)
//		{
//			VectorXd vpF = m_vDoF[i]->GetValue();
//
//			for (int j = 0; j < m_vDoF[i]->NumDim(); ++j)
//			{
//				// +
//				vpF[j] += eps;
//				m_vDoF[i]->SetValue(vpF);
//				this->m_pElement->UpdateMechanics();
//				this->m_pElement->ComputeAndStore_Energy_Internal();
//				Real ep = this->m_pElement->m_energy;
//
//				// -
//				vpF[j] -= 2*eps;
//				m_vDoF[i]->SetValue(vpF);
//				this->m_pElement->UpdateMechanics();
//				this->m_pElement->ComputeAndStore_Energy_Internal();
//				Real em = this->m_pElement->m_energy;
//
//				// Estimate g
//				vgF[count] = (ep - em) / (2 * eps);
//
//				// Restore state
//				vpF[j] += eps;
//
//				// Next!
//				count++;
//			}
//
//			m_vDoF[i]->SetValue(vpF);
//		}
//
//		this->m_pElement->ComputeAndStore_Energy_Internal();
//		this->ComputeAndStore_DEDp();
//		VectorXd vgA = this->m_vDEDp;
//
//		// Object
//		VectorXd vgD = vgA - vgF;
//		Real testNorm = vgA.norm();
//		Real realNorm = vgF.norm();
//		Real diffNorm = vgD.norm();
//		if (realNorm < 1e-6)
//		{
//			logTrace(Verbosity::V1_Default, "\n[INVALID] Finite difference DEDp near zero: %f", realNorm);
//
//			return true;
//		}
//		else
//		{
//			if (diffNorm / realNorm > 1e-6)
//			{
//				logTrace(Verbosity::V1_Default, "\n[FAILURE] %s DEDp test error: %f", this->GetName().c_str(), diffNorm / realNorm);
//
//				MatrixXd mResult(vgA.size(), 3);
//				mResult.col(0) = vgA;
//				mResult.col(1) = vgF;
//				mResult.col(2) = vgD;
//
//				ostringstream str;
//				str << endl << "Analitic|Estimated|Difference:" << endl << mResult << endl;
//				logTrace(Verbosity::V1_Default, "%s", str.str().c_str());
//
//				//logFile("csvDEDpTest_F.csv", vectorToString_CSV(vgF));
//				//logFile("csvDEDpTest_A.csv", vectorToString_CSV(vgA));
//
//				return false;
//			}
//			else
//			{
//				logTrace(Verbosity::V1_Default, "\n[SUCCESS] %s gradient test error: %f", this->GetName().c_str(), diffNorm / realNorm);
//
//				return true;
//			}
//		}
//	}
//
//	bool ParameterEnergy::TestLocalDE2Dxp()
//	{
//		return false;
//	}
//
//	bool ParameterEnergy::TestLocalDE2Dpp()
//	{
//		return false;
//	}
//}
