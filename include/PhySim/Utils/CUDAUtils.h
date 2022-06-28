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

void transformTripletValuesToCUDA(const VectorTd& triplets,
                                  vector<double>& cuda);
void transformCUDAToTripletValues(const vector<double>& cuda,
                                  VectorTd& triplets);
void transformTripletValuesToCUDA(const VectorTd& triplets,
                                  vector<float>& cuda);
void transformCUDAToTripletValues(const vector<float>& cuda,
                                  VectorTd& triplets);
void transformEigenVectorToCUDA(const VectorXd& veigen, vector<double>& cuda);
void transformEigenVectorToCUDA(const VectorXd& veigen, vector<float>& cuda);
void transformCUDAToEigenVector(const vector<float>& cuda, VectorXd& veigen);
void transformCUDAToEigenVector(const vector<double>& cuda, VectorXd& veigen);
void transformEigenTripletsToCUDA(const VectorTd& triplets,
                                  vector<int>& rows,
                                  vector<int>& cols,
                                  vector<double>& vals);
void transformEigenTripletsToCUDA(const VectorTd& triplets,
                                  vector<int>& rows,
                                  vector<int>& cols,
                                  vector<float>& vals);
void transformCUDAToEigenTriplets(const vector<int>& rows,
                                  const vector<int>& cols,
                                  const vector<float>& vals,
                                  VectorTd& triplets);
void transformCUDAToEigenTriplets(const vector<int>& rows,
                                  const vector<int>& cols,
                                  const vector<double>& vals,
                                  VectorTd& triplets);
}  // namespace PhySim
