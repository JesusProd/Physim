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

#include <PhySim/BasicTypes.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

void eigenTripletsToSparseMatrix(const VectorTd& triplets, MatrixSd& M);
void eigenSparseMatrixToPointers(const MatrixSd& M, VectorTp& triplets);
void eigenSparseMatrixToTriplets(const MatrixSd& M, VectorTd& triplets);

void buildBlockExpandedMatrix(const MatrixSd& mInput,
                              int expRate,
                              MatrixSd& mOut);
void buildSelectionMatrix(const iVector& vselected,
                          int numIndex,
                          MatrixSd& mOut);

void buildBlockExpandedMatrix(const MatrixXd& mInput,
                              int expRate,
                              MatrixXd& mOut);
void buildSelectionMatrix(const VectorXi& vselected,
                          int numIndex,
                          MatrixXd& mOut);

void buildDimensionRemovalMatrix(int numVert,
                                 int numDim,
                                 int remDim,
                                 MatrixXd& mOut);

void assembleSparseMatrix(VectorTd& triplets,
                          unsigned int row,
                          unsigned int col,
                          const MatrixXd& M);

void getDiagonalIndices(const VectorTd& vT, int rows, int cols, iVector& vDIdx);

}  // namespace PhySim
