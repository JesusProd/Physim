//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

// Standard

#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>

#include <math.h>
#include <stdio.h>

#if WIN32
#include <direct.h>
#else
// TODO
#endif

#define NOMINMAX

// Constants

#define M_E				2.71828182845904523536
#define M_LOG2E			1.44269504088896340736
#define M_LOG10E		0.43429448190325182765
#define M_LN2			0.69314718055994530941
#define M_LN10			2.30258509299404568402
#define M_PI			3.14159265358979323846
#define M_PI_2			1.57079632679489661923
#define M_PI_4			0.78539816339744830961
#define M_1_PI			0.31830988618379067153
#define M_2_PI			0.63661977236758134307
#define M_1_SQRTPI		0.56418958354775628694
#define M_2_SQRTPI		1.12837916709551257390
#define M_SQRT2			1.41421356237309504880
#define M_SQRT_2		0.70710678118654752440

// Eigen

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/CXX11/Tensor>

// Misc

#define GetCurrentDir _getcwd

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	// Smart pointers

	template <typename T> using PtrS = shared_ptr<T>;
	template <typename T> using PtrU = unique_ptr<T>;
    template <typename T, typename... Args> PtrS<T> NewS(Args&&... args) { return make_shared<T>(args...); }
    template <typename T, typename... Args> PtrU<T> NewU(Args&&... args) { return make_unique<T>(args...); }

	// Standard typedefs

	typedef double Real;
	typedef vector<int> iVector;
	typedef vector<float> fVector;
	typedef vector<double> dVector;
	typedef vector<bool> bVector;
	typedef vector<Real> rVector;

	typedef pair<int, int> IntPair;
	typedef map<IntPair, Real*> CoefMap;

	typedef Map<VectorXd> MapMatXd;
	typedef Map<VectorXd> MapVecXd;

	// Eigen typedefs

	using Vector4r = Vector<Real, 4>;
	using Vector3r = Vector<Real, 3>;
	using Vector2r = Vector<Real, 2>;
	using Matrix4r = Matrix<Real, 4, 4>;
	using Matrix3r = Matrix<Real, 3, 3>;
	using Matrix2r = Matrix<Real, 2, 2>;

	using VectorXr = Vector<Real, -1>;
	using MatrixXr = Matrix<Real, -1, -1>;

	template <int N>
	using VectorD = Matrix<Real, N, 1>;

	template <int N, int M>
	using MatrixD = Matrix<Real, N, M>;

	typedef vector<Triplet<Real>> VectorTd;
	typedef vector<Triplet<Real*>> VectorTp;
	typedef SparseMatrix<Real> MatrixSd;

	typedef Matrix<Real*, Dynamic, Dynamic> MatrixXp;

    template <int N>
    using TensorD = Tensor<Real, N>;
    using Tensor3d = Tensor<Real, 3>;
    using Tensor3i = Tensor<int, 3>;
}


// Utils

#include <PhySim/BasicTypes.h>
#include <PhySim/Utils/Utils.h>