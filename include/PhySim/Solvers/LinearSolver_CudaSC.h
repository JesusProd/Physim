#pragma once

//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/CommonIncludes.h>


#include <PhySim/Solvers/LinearSolver.h>

#ifdef USE_CUDA

#include <cuda_runtime.h>

#include <cuda.h>
#include <cusparse.h>
#include <cusolverSp.h>
#include <cusolverRf.h>
#include <cusolver_common.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class LinearSolver_CudaSC : public LinearSolver
	{
	protected:

		LinearSolverOptions m_options;

		CustomTimer timerSolve;

		// Solver status/handle

		cusolverStatus_t cusolver_status;
		cusolverSpHandle_t cusolver_handle;
		cusolverRfHandle_t curefact_handle;

		// Sparsity status/handle

		cusparseHandle_t cusparse_handle;
		cusparseStatus_t cusparse_status;

		// Matrix descriptor

		cusparseMatDescr_t mA_descriptor;

		// Matrix in HOST

		iVector host_rows_coo;
		iVector host_cols_csr;
		fVector host_vals_csr;

		// Matrix in DEVICE

		int* devi_rows_coo;
		int* devi_cols_csr;
		int* devi_rows_csr;
		float* devi_vals_csr;

		// Vectors in HOST

		fVector host_x;
		fVector host_t;
		fVector host_b;

		// Vectors in DEVICE

		float* devi_b;
		float* devi_x;
		float* devi_t;

		// Diagonal values

		iVector vDIdx;

	public:

		LinearSolver_CudaSC();
		LinearSolver_CudaSC(const MatrixSd& mA, const LinearSolverOptions& options);
		virtual void Init(const MatrixSd& mA, const LinearSolverOptions& options);
		virtual ~LinearSolver_CudaSC();

	protected:

		virtual SolveResult SolveInternal(MatrixSd& mA, const VectorXd& vb, VectorXd& vx);
		virtual SolveResult SolveInternal(MatrixSd& mA, const MatrixXd& mB, MatrixXd& mX);
		virtual SolveResult SolveInternal(MatrixSd& mA, const MatrixSd& mB, MatrixSd& mX);

		virtual void Free();

	};

}

#endif
