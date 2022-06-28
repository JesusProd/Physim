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

class Simulable;

namespace Utils {
void add3D(int i, const Vector3d& v, dVector& vd);
void set3D(int i, const Vector3d& v, dVector& vd);
Eigen::Vector3d get3D(int i, const dVector& vd);

void getSubvector(int offset, int numEle, const dVector& vin, dVector& vout);
void setSubvector(int offset, int numEle, const dVector& vin, dVector& vout);

void getSubvector(int offset, int numEle, const VectorXd& vin, VectorXd& vout);
void setSubvector(int offset, int numEle, const VectorXd& vin, VectorXd& vout);

dVector toSTL(const Eigen::VectorXd& v);
iVector toSTL(const Eigen::VectorXi& v);
dVector toSTL(const Eigen::Vector3d& v);
Eigen::VectorXd toEigen(const dVector& v);
Eigen::VectorXi toEigen(const iVector& v);

void removeDuplicates(iVector& v);

bool presolveStaticsForFixedDoF(Simulable& simulable,
                                const vector<VectorXd>& vFixedPos);
}  // namespace Utils
}  // namespace PhySim
