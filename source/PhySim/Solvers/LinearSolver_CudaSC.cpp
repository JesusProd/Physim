//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/LinearSolver_CudaSC.h>

#ifdef USE_CUDA

namespace PhySim {
using namespace std;
using namespace Eigen;

LinearSolver_CudaSC::LinearSolver_CudaSC() : LinearSolver() {
  devi_rows_coo = NULL;
  devi_cols_csr = NULL;
  devi_rows_csr = NULL;
  devi_vals_csr = NULL;
  devi_b = NULL;
  devi_x = NULL;
  cusparse_handle = 0;
  cusolver_handle = 0;
}

LinearSolver_CudaSC::LinearSolver_CudaSC(const MatrixSd& mA,
                                         const LinearSolverOptions& options)
    : LinearSolver(mA, options) {
  devi_rows_coo = NULL;
  devi_cols_csr = NULL;
  devi_rows_csr = NULL;
  devi_vals_csr = NULL;
  devi_b = NULL;
  devi_x = NULL;
  devi_t = NULL;
  cusparse_handle = 0;
  cusolver_handle = 0;

  this->Init(mA, options);
}

LinearSolver_CudaSC::~LinearSolver_CudaSC() {
  this->Free();
}

void LinearSolver_CudaSC::Init(const MatrixSd& mA,
                               const LinearSolverOptions& options) {
  LinearSolver::Init(mA, options);

  this->Free();

  // Initialize memory

  VectorTd vAFull;
  MatrixSd mAFull = mA.selfadjointView<Lower>();
  eigenSparseMatrixToTriplets(mAFull, vAFull);
  sort(vAFull.begin(), vAFull.end(), LT_Triplet());

  int numRows = (int)mAFull.rows();
  int numCols = (int)mAFull.cols();
  int numNZ = (int)vAFull.size();

  cusolver_status = cusolverSpCreate(&cusolver_handle);
  cusparse_status = cusparseCreate(&cusparse_handle);

  logTrace(Verbosity::V1_Default,
           "\n[TRACE] CUDA Solver: created solver handle %d", cusolver_status);
  logTrace(Verbosity::V1_Default,
           "\n[TRACE] CUDA Solver: created sparse handle %d", cusparse_status);

  // Initialize host vectors/matrix

  VectorXd vb = VectorXd::Zero(numRows);
  VectorXd vx = VectorXd::Zero(numRows);
  VectorXd vt = VectorXd::Zero(numRows);
  transformEigenTripletsToCUDA(vAFull, host_rows_coo, host_cols_csr,
                               host_vals_csr);
  transformEigenVectorToCUDA(vb, host_b);
  transformEigenVectorToCUDA(vx, host_x);
  transformEigenVectorToCUDA(vt, host_t);
  getDiagonalIndices(vAFull, numRows, numCols, vDIdx);

  // Reserve memory in the device

  cudaMalloc((void**)&devi_b, numRows * sizeof(float));
  cudaMalloc((void**)&devi_x, numRows * sizeof(float));
  cudaMalloc((void**)&devi_t, numRows * sizeof(float));

  logTrace(Verbosity::V1_Default,
           "\n[TRACE] CUDA Solver: reserved vector DEVICE memory");

  cudaMalloc((void**)&devi_vals_csr, numNZ * sizeof(float));
  cudaMalloc((void**)&devi_rows_coo, numNZ * sizeof(int));
  cudaMalloc((void**)&devi_cols_csr, numNZ * sizeof(int));
  cudaMalloc((void**)&devi_rows_csr, (numRows + 1) * sizeof(int));

  logTrace(Verbosity::V1_Default,
           "\n[TRACE] CUDA Solver: created matrix DEVICE memory");

  // Copy Matrix indices host to device

  cudaMemcpy(devi_rows_coo, host_rows_coo.data(),
             host_rows_coo.size() * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(devi_cols_csr, host_cols_csr.data(),
             host_cols_csr.size() * sizeof(int), cudaMemcpyHostToDevice);
  cusparse_status =
      cusparseXcoo2csr(cusparse_handle, devi_rows_coo, numNZ, numRows,
                       devi_rows_csr, CUSPARSE_INDEX_BASE_ZERO);
}

void LinearSolver_CudaSC::Free() {
  if (devi_b != NULL)
    cudaFree(devi_b);
  if (devi_x != NULL)
    cudaFree(devi_x);
  if (devi_t != NULL)
    cudaFree(devi_t);
  if (devi_rows_coo != NULL)
    cudaFree(devi_rows_coo);
  if (devi_rows_csr != NULL)
    cudaFree(devi_rows_csr);
  if (devi_cols_csr != NULL)
    cudaFree(devi_cols_csr);
  if (devi_vals_csr != NULL)
    cudaFree(devi_vals_csr);
  if (cusparse_handle != 0)
    cusparseDestroy(cusparse_handle);
}

SolveResult LinearSolver_CudaSC::SolveInternal(MatrixSd& mA,
                                               const VectorXd& vb,
                                               VectorXd& vx) {
  MatrixSd mAFull = mA.selfadjointView<Lower>();

  Real normA = mAFull.norm();
  Real regThres = min(1e-3, max(1e-6, 1e-6 * normA));
  Real sinThres = min(1e-6, max(1e-9, 1e-9 * normA));
  Real regValue = m_options.regSign * regThres;

  VectorTd vAFull;
  eigenSparseMatrixToTriplets(mAFull, vAFull);
  sort(vAFull.begin(), vAFull.end(), LT_Triplet());
  int numRows = mAFull.rows();
  int numCols = mAFull.cols();
  int numNZ = (int)vAFull.size();

  VectorXd vt;
  vt = VectorXd::Zero(numRows);
  vx = VectorXd::Zero(numRows);
  transformEigenVectorToCUDA(vb, host_b);
  transformEigenVectorToCUDA(vx, host_x);
  transformEigenVectorToCUDA(vt, host_t);
  transformTripletValuesToCUDA(vAFull, host_vals_csr);

  // Copy vector values host to device

  cudaMemcpy(devi_b, host_b.data(), host_b.size() * sizeof(float),
             cudaMemcpyHostToDevice);

  // Solve the specified definite positive linear system.
  // Regularize the matrix if needed to create a DP system.

  int i = 0;
  for (i = 0; i < this->m_options.regIters; ++i) {
    if (i != 0) {
      logTrace(Verbosity::V1_Default,
               "\n[INFO] Regularizing system, iteration %u", i);

      for (int j = 0; j < numRows; ++j)
        host_vals_csr[vDIdx[j]] += (float)regValue * pow(10, i - 1);
    }

    // Copy Matrix values host to device

    cudaMemcpy(devi_vals_csr, host_vals_csr.data(),
               host_vals_csr.size() * sizeof(float), cudaMemcpyHostToDevice);

    // Solving

    int singular = 0;

    cudaDeviceSynchronize();  // logTrace(Verbosity::V1_Default, "\n[TRACE] CUDA
                              // Solver: synchronization %d",
                              // cudaDeviceSynchronize());

    cusparse_status = cusparseCreateMatDescr(&mA_descriptor);

    cusolver_status = cusolverSpScsrlsvchol(
        cusolver_handle, numRows, numNZ, mA_descriptor, devi_vals_csr,
        devi_rows_csr, devi_cols_csr, devi_b, 0, 1, devi_x, &singular);

    // logTrace(Verbosity::V1_Default, "\n[TRACE] CUDA Solver: system solved
    // %d", cusolver_status);

    cudaDeviceSynchronize();  // logTrace(Verbosity::V1_Default, "\n[TRACE] CUDA
                              // Solver: synchronization: %d",
                              // cudaDeviceSynchronize());

    // Check singularity

    if (singular != -1) {
      logTrace(Verbosity::V1_Default,
               "\n[FAILURE] CUDA Solver: indefinite or singular");
      continue;
    }

    // Compute solution test t = A*x

    float alpha = 1;
    float beta = 0;
    cusparseScsrmv(cusparse_handle, CUSPARSE_OPERATION_NON_TRANSPOSE, numRows,
                   numCols, numNZ, &alpha, mA_descriptor, devi_vals_csr,
                   devi_rows_csr, devi_cols_csr, devi_x, &beta, devi_t);

    cudaDeviceSynchronize();  // logTrace(Verbosity::V1_Default, "\n[TRACE] CUDA
                              // Solver: synchronization: %d",
                              // cudaDeviceSynchronize());

    // Copy solution values device to host

    cudaMemcpy(host_x.data(), devi_x, numRows * sizeof(float),
               cudaMemcpyDeviceToHost);
    cudaMemcpy(host_t.data(), devi_t, numRows * sizeof(float),
               cudaMemcpyDeviceToHost);
    for (int j = 0; j < numRows; ++j) {
      vx(j) = host_x[j];
      vt(j) = host_t[j];
    }

    // Check indefiniteness

    double dot = vb.dot(vx);

    if (this->m_options.regSign > 0 && dot < 0.0) {
      logTrace(Verbosity::V1_Default,
               "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
      continue;
    }
    if (this->m_options.regSign < 0 && dot > 0.0) {
      logTrace(Verbosity::V1_Default,
               "[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
      continue;
    }

    //// Check exact solution

    // double absError = (vb - vt).norm();
    // double relError = absError / vb.norm();
    // if (absError > this->options.maxError && relError >
    // this->options.maxError)
    //{
    //	logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: inexact
    // solution, error: %.6e", relError); 	continue; // Iterate again
    //}

    logTrace(Verbosity::V1_Default,
             "\n[SUCCESS] CUDA Solver: solved using Cholesky after %i "
             "regularization steps",
             i);

    return SolveResult::Success;
  }

  return SolveResult::Failure;
}

SolveResult LinearSolver_CudaSC::SolveInternal(MatrixSd& mA,
                                               const MatrixXd& mB,
                                               MatrixXd& mX) {
  throw new exception("Not implemented");

  return SolveResult::Failure;
}

SolveResult LinearSolver_CudaSC::SolveInternal(MatrixSd& mA,
                                               const MatrixSd& mB,
                                               MatrixSd& mX) {
  throw new exception("Not implemented");

  return SolveResult::Failure;
}

}  // namespace PhySim

#endif