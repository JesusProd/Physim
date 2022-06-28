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

#include <PhySim/Geometry/Volumes/ScalarField_Grid.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	namespace Serializer_Field
	{
		bool ReadGrid_SVL(const string& filePath, ScalarField_Grid<char>& field);
		bool ReadGrid_SVL(const string& filePath, ScalarField_Grid<short>& field);
		bool ReadGrid_SVL(const string& filePath, ScalarField_Grid<int>& field);
		bool ReadGrid_SVL(const string& filePath, ScalarField_Grid<float>& field);
		bool ReadGrid_SVL(const string& filePath, ScalarField_Grid<double>& field);

		bool WriteGrid_SVL(const string& filePath, const ScalarField_Grid<char>& field);
		bool WriteGrid_SVL(const string& filePath, const ScalarField_Grid<short>& field);
		bool WriteGrid_SVL(const string& filePath, const ScalarField_Grid<int>& field);
		bool WriteGrid_SVL(const string& filePath, const ScalarField_Grid<float>& field);
		bool WriteGrid_SVL(const string& filePath, const ScalarField_Grid<double>& field);
	};
}