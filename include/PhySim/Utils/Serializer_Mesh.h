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


namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Serializer_Mesh
	{
	public:
		struct FaceGroup
		{
			string gname;
			iVector vidx;
		};

		static bool ReadSurface_Any(const string& filePath, MatrixXd& mV, MatrixXi& mF, MatrixXd& mN);

		static bool ReadSurface_TOBJ(const string& filePath, MatrixXd& mV, MatrixXi& mF, vector<FaceGroup>& vGroups);

		static bool ReadSurface_OBJ(const string& filePath, MatrixXd& mV, MatrixXi& mF);
		static bool ReadSurface_OFF(const string& filePath, MatrixXd& mV, MatrixXi& mF);
		static bool ReadSurface_STL(const string& filePath, MatrixXd& mV, MatrixXi& mF, MatrixXd& mN);

		static bool ReadVolume_TET(const string& filePath, MatrixXd& mV, MatrixXi& mT, MatrixXi& mF);
		static bool ReadVolume_TET_Node(const string& filePath, MatrixXd& mV);
		static bool ReadVolume_TET_Elem(const string& filePath, MatrixXi& mT);
		static bool ReadVolume_TET_Face(const string& filePath, MatrixXi& mF);

		static bool ReadVolume_HEX(const string& filePath, MatrixXd& mV, MatrixXi& mH, VectorXi& vSmap, MatrixXi& mS, MatrixXd& mN);

		static bool WriteSurface_OBJ(const string& filePath, const MatrixXd& mV, const MatrixXi& mF);
		static bool WriteSurface_OFF(const string& filePath, const MatrixXd& mV, const MatrixXi& mF);
		static bool WriteSurface_STL(const string& filePath, const MatrixXd& mV, const MatrixXi& mF, const MatrixXd& mN);

        static bool WriteVolume_TET(const string& filePath, const MatrixXd& mV, const MatrixXi& mT, const MatrixXi& mF);
        static bool WriteVolume_TET_Node(const string& filePath, const MatrixXd& mV);
        static bool WriteVolume_TET_Elem(const string& filePath, const MatrixXi& mT);
        static bool WriteVolume_TET_Face(const string& filePath, const MatrixXi& mF);
		static bool WriteVolume_HEX(const string& filePath, const MatrixXd& mV, const MatrixXi& mH);
	};
}