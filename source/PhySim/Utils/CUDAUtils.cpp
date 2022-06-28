//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/CUDAUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

void transformEigenVectorToCUDA(const VectorXd& veigen, vector<float>& vcuda) {
  int numVal = (int)veigen.size();

  vcuda.clear();
  vcuda.reserve(numVal);
  for (int i = 0; i < numVal; ++i)
    vcuda.push_back((float)veigen(i));
}

void transformEigenVectorToCUDA(const VectorXd& veigen, vector<double>& vcuda) {
  int numVal = (int)veigen.size();

  vcuda.clear();
  vcuda.reserve(numVal);
  for (int i = 0; i < numVal; ++i)
    vcuda.push_back((float)veigen(i));
}

void transformCUDAToEigenVector(const vector<double>& vcuda, VectorXd& veigen) {
  int numVal = (int)vcuda.size();

  veigen.resize(numVal);
  for (int i = 0; i < numVal; ++i)
    veigen(i) = vcuda[i];
}

void transformCUDAToEigenVector(const vector<float>& vcuda, VectorXd& veigen) {
  int numVal = (int)vcuda.size();

  veigen.resize(numVal);
  for (int i = 0; i < numVal; ++i)
    veigen(i) = vcuda[i];
}

void transformTripletValuesToCUDA(const VectorTd& vT, vector<float>& vals) {
  int numNZ = (int)vT.size();
  assert((int)vals.size() == numNZ);
  for (int i = 0; i < numNZ; ++i)
    vals[i] = (float)vT[i].value();
}

void transformTripletValuesToCUDA(const VectorTd& vT, vector<double>& vals) {
  int numNZ = (int)vT.size();
  assert((int)vals.size() == numNZ);
  for (int i = 0; i < numNZ; ++i)
    vals[i] = (float)vT[i].value();
}

void transformCUDAToTripletValues(const vector<float>& vals, VectorTd& vT) {
  int numNZ = (int)vals.size();
  assert((int)vT.size() == numNZ);
  for (int i = 0; i < numNZ; ++i)
    vT[i] = Triplet<Real>(vT[i].row(), vT[i].col(), vals[i]);
}

void transformCUDAToTripletValues(const vector<double>& vals, VectorTd& vT) {
  int numNZ = (int)vals.size();
  assert((int)vT.size() == numNZ);
  for (int i = 0; i < numNZ; ++i)
    vT[i] = Triplet<Real>(vT[i].row(), vT[i].col(), vals[i]);
}

void transformEigenTripletsToCUDA(const VectorTd& triplets,
                                  vector<int>& rows,
                                  vector<int>& cols,
                                  vector<float>& vals) {
  int numNZ = (int)triplets.size();
  rows.reserve(numNZ);
  cols.reserve(numNZ);
  vals.reserve(numNZ);
  for (int i = 0; i < numNZ; ++i) {
    const Triplet<double>& t = triplets[i];
    rows.push_back(t.row());
    cols.push_back(t.col());
    vals.push_back((float)t.value());
  }
}

void transformCUDAToEigenTriplets(const vector<int>& rows,
                                  const vector<int>& cols,
                                  const vector<float>& vals,
                                  VectorTd& triplets) {
  int numNZ = (int)rows.size();
  triplets.reserve(numNZ);
  for (int i = 0; i < numNZ; ++i)
    triplets.push_back(Triplet<double>(rows[i], cols[i], vals[i]));
}

}  // namespace PhySim
