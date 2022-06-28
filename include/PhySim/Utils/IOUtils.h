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

namespace IOUtils {

Vector3d convertHSLtoRGB(const Vector3d& hsl);
Vector3d computeHeatColorRGB(Real alpha);
Vector3d computeHeatColorHSL(Real alpha);

string getCurrentDirectory();
bool listDirectoryFiles(const string& direPath, vector<string>& vfiles);
string getAbsolutePath(const string& filePath, const string& parent = "");
bool getDirectoryParent(const string& childPath, string& parentPath);
string getFileDirectory(const string& filePath);
string getFileName(const string& filePath);
bool isPathAbsolute(const string& filePath);
bool isPathRelative(const string& filePath);

bool readFileLines(const string& filePath, vector<string>& vlines);
bool readFileString(const string& filePath, string& content);
vector<vector<vector<Matrix3d>>> import_nij(const string& filePath,
                                            int nb_elements,
                                            int coarsening_level);

template <typename T>
void readBinaryValue(istream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
}

template <typename T>
void readBinaryArray(istream& in, T* pBuffer, int count) {
  in.read(reinterpret_cast<char*>(pBuffer), sizeof(T) * count);
}

template <typename TStream, typename TBuffer>
void readBinaryArrayWithCast(istream& in, TBuffer* pBuffer, int count) {
  vector<TStream> vTemporary(count);
  in.read(reinterpret_cast<char*>(vTemporary.data()), sizeof(TStream) * count);
  std::transform(vTemporary.data(), vTemporary.data() + count, pBuffer,
                 [](const TStream& value) { return TBuffer(value); });
}

template <typename T>
void writeBinaryValue(ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
void writeBinaryArray(ostream& out, const T* pBuffer, int count) {
  out.write(reinterpret_cast<const char*>(pBuffer), sizeof(T) * count);
}

template <typename TStream, typename TBuffer>
void writeBinaryArrayWithCast(ostream& out, const TBuffer* pBuffer, int count) {
  vector<TStream> vTemporary(count);
  transform(pBuffer, pBuffer + count, vTemporary.data(),
            [](const TBuffer& value) { return TStream(value); });
  out.write(reinterpret_cast<const char*>(vTemporary.data()),
            sizeof(TStream) * count);
}

string vectorToString_CSV(const VectorXd& v);
string matrixToString_CSV(const MatrixXd& M);
string vectorToString_CSV(const VectorXi& v);
string matrixToString_CSV(const MatrixXi& M);
VectorXd stringToVector_CSV(const string& str);
MatrixXd stringToMatrix_CSV(const string& str);

template <typename T, int N>
void stringToVector_CSV(const string& str, Matrix<T, N, 1>& vOut);

template <typename T, int N, int M>
void stringToMatrix_CSV(const string& str, Matrix<T, N, M>& mOut);

template <typename T, int N, int M>
void matrixToString_CSV(const Matrix<T, N, M>& mIn, string& strOut);

template <typename T, int N, int M>
bool readMatrix_CSV(const string& path, Matrix<T, N, M>& mOut);
template <typename T, int N, int M>
bool writeMatrix_CSV(const string& path, Matrix<T, N, M>& mOut);

bool readMatrix_CSV(const string& path, MatrixXd& mV);
bool writeMatrix_CSV(const string& path, MatrixXd& mV);

vector<string> splitString(const string& input, const string& delim);
void splitString(const string& input, const string& delim, vector<string>& vs);

string vectorToString_CSV(const VectorXd& v);
string matrixToString_CSV(const MatrixXd& M);
void logFile(const string& path, const string& content, bool append = false);
void logTrace(Verbosity verbosity, const char* format, ...);
void logTimes(Verbosity verbosity, const char* format, ...);

}  // namespace IOUtils
}  // namespace PhySim
