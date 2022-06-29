//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/IOUtils.h>

#include <stdarg.h>
#include <iomanip>

#if WIN32
#include <PhySim/Utils/Third/dirent.h>
#else
#include <unistd.h>
#endif

#pragma warning(disable : 4996)  // Disable fopen warning

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace IOUtils {

string vectorToString_CSV(const VectorXi& v) {
  ostringstream str;
  int np = (int)v.size();
  for (int j = 0; j < np - 1; ++j)
    str << v(j) << ",";
  str << v(np - 1);

  return str.str();
}

string vectorToString_CSV(const VectorXd& v) {
  ostringstream str;
  str << std::fixed << std::setprecision(16);
  int np = (int)v.size();
  for (int j = 0; j < np - 1; ++j)
    str << v(j) << ",";
  str << v(np - 1);

  return str.str();
}

string matrixToString_CSV(const MatrixXi& M) {
  ostringstream str;
  int nr = (int)M.rows();
  int nc = (int)M.cols();
  for (int i = 0; i < nr; ++i) {
    for (int j = 0; j < nc - 1; ++j)
      str << M(i, j) << ",";
    str << M(i, nc - 1);

    str << "\n";
  }

  return str.str();
}

string matrixToString_CSV(const MatrixXd& M) {
  ostringstream str;
  int nr = (int)M.rows();
  int nc = (int)M.cols();
  for (int i = 0; i < nr; ++i) {
    for (int j = 0; j < nc - 1; ++j)
      str << M(i, j) << ",";
    str << M(i, nc - 1);

    str << "\n";
  }

  return str.str();
}

VectorXd stringToVector_CSV(const string& str) {
  vector<string> vcoef = splitString(str, ",");

  VectorXd v((int)vcoef.size());
  for (size_t i = 0; i < vcoef.size(); ++i)
    v(i) = atof(vcoef[i].c_str());

  return v;
}

MatrixXd stringToMatrix_CSV(const string& str) {
  vector<string> vstringRows = splitString(str, "\n");

  int numLines = (int)vstringRows.size();

  vector<VectorXd> vparsedRows;
  vparsedRows.reserve(numLines);

  for (size_t i = 0; i < numLines; ++i) {
    if (vstringRows[i].empty() || vstringRows[i] == "\t" ||
        vstringRows[i] == "\r")
      continue;  // Bad

    vparsedRows.push_back(stringToVector_CSV(vstringRows[i]));
  }

  MatrixXd m(vparsedRows.size(), vparsedRows[0].size());
  for (auto i = 0; i < (int)vparsedRows.size(); ++i)
    m.row(i) = vparsedRows[i];

  return m;
}

vector<string> splitString(const string& input, const string& delim) {
  std::string s = input;
  std::string D = delim;

  vector<string> vs;

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(D)) != std::string::npos) {
    vs.push_back(s.substr(0, pos));
    s.erase(0, pos + D.length());
  }
  vs.push_back(s);

  return vs;
}

bool readMatrix_CSV(const string& path, MatrixXd& mV) {
  vector<string> vlines;
  if (!readFileLines(path, vlines))
    return false;  // Invalid file

  int numRows = (int)vlines.size();
  vector<dVector> vlineValue(numRows);
  for (int i = 0; i < numRows; ++i) {
    vector<string> vvalues;
    vvalues = splitString(vlines[i], ",");
    int numValues = (int)vvalues.size();

    if (vvalues[0].empty() || vvalues[0] == "\t" || vvalues[0] == "\r")
      continue;  // Bad

    vlineValue[i].resize(numValues);
    for (int j = 0; j < numValues; ++j)
      vlineValue[i][j] = atof(vvalues[j].c_str());
  }

  int numCols = (int)vlineValue[0].size();
  mV.resize(numRows, numCols);
  for (int i = 0; i < numRows; ++i)
    for (int j = 0; j < numCols; ++j)
      mV(i, j) = vlineValue[i][j];

  return true;
}

bool writeMatrix_CSV(const string& path, MatrixXd& mV) {
  logFile(path, matrixToString_CSV(mV), false);

  return true;
}

template <typename T, int N>
void stringToVector_CSV(const string& str, Matrix<T, N, 1>& vOut) {
  vector<string> vstringRows = Utils::splitString(str, "\n");
  vector<Matrix<T, -1, 1>> vparsedRows(vstringRows.size());

  for (size_t i = 0; i < vstringRows.size(); ++i) {
    vector<string> vcoef = Utils::splitString(str, ",");

    vparsedRows[i].resize((int)vcoef.size());
    for (size_t j = 0; j < vcoef.size(); ++j)
      vparsedRows[i](j) = (T)atof(vcoef[i].c_str());
  }

  vOut.resize(vparsedRows[0].size());

  for (size_t j = 0; j < vparsedRows[0].size(); ++j)
    vOut(j) = vparsedRows[0][j];
}

template <typename T, int N, int M>
void stringToMatrix_CSV(const string& str, Matrix<T, N, M>& mOut) {
  vector<string> vstringRows = Utils::splitString(str, "\n");

  vector<Matrix<T, -1, 1>> vparsedRows;
  vparsedRows.reserve(vstringRows.size());

  for (size_t i = 0; i < vstringRows.size(); ++i) {
    vector<string> vcoef = Utils::splitString(vstringRows[i], ",");

    if (vcoef[0].empty() || vcoef[0] == "\t" || vcoef[0] == "\r")
      continue;  // Bad

    vparsedRows.push_back(Matrix<T, -1, 1>());
    vparsedRows.back().resize(vcoef.size());

    for (size_t j = 0; j < vcoef.size(); ++j)
      vparsedRows.back()[j] = (T)atof(vcoef[j].c_str());
  }

  if (N == -1 && M == -1) {
    mOut.resize(vparsedRows.size(), vparsedRows[0].size());
  } else {
    if (N == -1)
      mOut.resize(vparsedRows.size(), M);
    if (M == -1)
      mOut.resize(N, vparsedRows[0].size());
  }

  for (size_t i = 0; i < vparsedRows.size(); ++i)
    for (size_t j = 0; j < vparsedRows[0].size(); ++j)
      mOut(i, j) = vparsedRows[i][j];
}

template <typename T, int N, int M>
void matrixToString_CSV(const Matrix<T, N, M>& mIn, string& strOut) {
  ostringstream str;

  for (int i = 0; i < mIn.rows(); ++i) {
    for (int j = 0; j < mIn.cols() - 1; ++j)
      str << mIn(i, j) << ",";
    str << mIn(i, mIn.rows() - 1);

    str << "\n";
  }

  strOut = str.str();
}

template <typename T, int N, int M>
bool readMatrix_CSV(const string& path, Matrix<T, N, M>& mOut) {
  string content;

  if (!readFileString(path, content))
    return false;  // Invalid file

  stringToMatrix_CSV(content, mOut);
}

template <typename T, int N, int M>
bool writeMatrix_CSV(const string& path, Matrix<T, N, M>& mOut) {
  string mStr;
  matrixToString_CSV<T, N, M>(mOut, mStr);
  logFile(path, mStr, false);
}

Vector3d convertHSLtoRGB(const Vector3d& hsl) {
  double h = hsl[0];
  double sl = hsl[1];
  double l = hsl[2];

  double v;
  double r, g, b;

  r = l;  // default to gray
  g = l;
  b = l;
  v = (l <= 0.5) ? (l * (1.0 + sl)) : (l + sl - l * sl);
  if (v > 0) {
    double m;
    double sv;
    int sextant;
    double fract, vsf, mid1, mid2;

    m = l + l - v;
    sv = (v - m) / v;
    h *= 6.0;
    sextant = (int)h;
    fract = h - sextant;
    vsf = v * sv * fract;
    mid1 = m + vsf;
    mid2 = v - vsf;
    switch (sextant) {
      case 0:
        r = v;
        g = mid1;
        b = m;
        break;
      case 1:
        r = mid2;
        g = v;
        b = m;
        break;
      case 2:
        r = m;
        g = v;
        b = mid1;
        break;
      case 3:
        r = m;
        g = mid2;
        b = v;
        break;
      case 4:
        r = mid1;
        g = m;
        b = v;
        break;
      case 5:
        r = v;
        g = m;
        b = mid2;
        break;
    }
  }

  Vector3d rgb;
  rgb[0] = r * 255.0f;
  rgb[1] = g * 255.0f;
  rgb[2] = b * 255.0f;
  return rgb;
}

Vector3d computeHeatColorRGB(Real alpha) {
  Vector3d heatHSL = computeHeatColorHSL(alpha);
  return convertHSLtoRGB(heatHSL);  // Convert
}

Vector3d computeHeatColorHSL(Real alpha) {
  double MAX = (240.0 / 360.0);
  double h = (1.0 - alpha) * MAX;
  double s = 0.85;
  double l = 0.5;

  Vector3d hsl;
  hsl[0] = h;
  hsl[1] = s;
  hsl[2] = l;
  return hsl;
}

bool listDirectoryFiles(const string& direPath, vector<string>& vfiles) {
#if WIN32
  DIR* pDire;

  if ((pDire = opendir(direPath.c_str())) != NULL) {
    struct dirent* ent;

    while ((ent = readdir(pDire)) != NULL) {
      if (string(ent->d_name) == ".")
        continue;
      if (string(ent->d_name) == "..")
        continue;

      vfiles.push_back(ent->d_name);
    }

    closedir(pDire);

    return true;
  } else {
    return false;
  }
#else
  // TODO

  return false;
#endif
}

bool getDirectoryParent(const string& childPath, string& parentPath) {
  // TODO
  return false;
}

string getFileDirectory(const string& filePath) {
  size_t found;
  found = filePath.find_last_of("/\\");
  return filePath.substr(0, found);
}

string getFileName(const string& filePath) {
  size_t found;
  found = filePath.find_last_of("/\\");
  return filePath.substr(found + 1);
}

string getCurrentDirectory() {
#if WIN32
  char buffer[1024];
  char* c_cwd = getcwd(buffer, sizeof(buffer));
  if (c_cwd) {
    return string(c_cwd);
  } else {
    return string("");
  }
#else
  char buffer[1024];
  char* c_cwd = getcwd(buffer, sizeof(buffer));
  if (c_cwd) {
    return string(c_cwd);
  } else {
    return string("");
  }
#endif

  return "";
}

bool isPathAbsolute(const string& filePath) {
  FILE* pFile;
  if ((pFile = fopen(filePath.c_str(), "r")) != NULL) {
    return true;
  } else {
    return false;
  }
}

bool isPathRelative(const string& filePath) {
#if WIN32
  string fullPath = getAbsolutePath(getCurrentDirectory(), filePath);

  FILE* pFile;
  if ((pFile = fopen(fullPath.c_str(), "r")) != NULL) {
    return true;
  } else {
    return false;
  }
#else
  string fullPath = getAbsolutePath(getCurrentDirectory(), filePath);

  FILE* pFile;
  if ((pFile = fopen(fullPath.c_str(), "r")) != NULL) {
    return true;
  } else {
    return false;
  }
#endif

  return false;
}

string getAbsolutePath(const string& filePath, const string& parent) {
#if WIN32
  if (isPathAbsolute(filePath)) {
    return filePath;
  }

  if (isPathAbsolute(parent + "/" + filePath)) {
    return parent + "/" + filePath;
  }

  if (isPathAbsolute(getCurrentDirectory() + "/" + filePath)) {
    return getCurrentDirectory() + "/" + filePath;
  }
#else
  if (isPathAbsolute(filePath)) {
    return filePath;
  }

  if (isPathAbsolute(parent + "/" + filePath)) {
    return parent + "/" + filePath;
  }

  if (isPathAbsolute(getCurrentDirectory() + "/" + filePath)) {
    return getCurrentDirectory() + "/" + filePath;
  }
#endif

  return "";
}

bool readFileLines(const string& path, vector<string>& vlines) {
  string line;

  ifstream file(path);
  if (file.is_open()) {
    while (getline(file, line))
      vlines.push_back(line);
    file.close();

    return true;
  } else {
    return false;
  }
}

bool readFileString(const string& filePath, string& content) {
  ifstream fileIn(filePath);
  if (!fileIn.is_open())
    return false;

  fileIn.seekg(0, ios::end);

  content.reserve(fileIn.tellg());

  fileIn.seekg(0, ios::beg);

  content.assign((istreambuf_iterator<char>(fileIn)),
                 istreambuf_iterator<char>());

  return true;
}

vector<vector<vector<Matrix3d>>> import_nij(const string& filePath,
                                            int nb_elements,
                                            int coarsening_level) {
  ifstream fileIn(filePath);
  if (!fileIn.is_open()) {
    cout << "Can't load nij coefficients file" << endl;
    exit(0);
  } else {
    // store the whole nij file in an array
    int dofs = 8 * pow(coarsening_level + 2, 3) * 3 * 3;
    VectorXd nvec(dofs * nb_elements);
    double coef;
    int ii(0);
    while (fileIn >> coef) {
      nvec(ii) = coef;
      ii += 1;
    }

    // order it in its tensor form
    vector<vector<vector<Matrix3d>>> n_full;
    n_full.resize(nb_elements);
    for (int elt = 0; elt < nb_elements; elt++) {
      n_full[elt].resize(8);
      for (int i = 0; i < 8; i++) {
        n_full[elt][i].resize(pow(coarsening_level + 2, 3));
        for (int j = 0; j < pow(coarsening_level + 2, 3); j++) {
          for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
              n_full[elt][i][j](k, l) =
                  nvec(9 * int(pow(coarsening_level + 2, 3)) * i + 9 * j +
                       3 * k + l + 1944 * elt);
        }
      }
    }

    return n_full;
  }
}

void splitString(const string& input, const string& delim, vector<string>& vs) {
  std::string s = input;
  std::string D = delim;

  vs.clear();

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(D)) != std::string::npos) {
    vs.push_back(s.substr(0, pos));
    s.erase(0, pos + D.length());
  }
  vs.push_back(s);
}

void logFile(const string& path, const string& content, bool append) {
  FILE* fp = NULL;
  if (!append)
    fp = fopen(path.c_str(), "wt");
  else
    fp = fopen(path.c_str(), "wa");
  fprintf(fp, "%s", content.c_str());
  fflush(fp);
  fclose(fp);
}

// bool readMatrix_CSV(const string& path, MatrixXd& mV) {
//  vector<string> vlines;
//  if (!readFileLines(path, vlines))
//    return false;  // Invalid file
//
//  int numRows = (int)vlines.size();
//  vector<dVector> vlineValue(numRows);
//  for (int i = 0; i < numRows; ++i) {
//    vector<string> vvalues;
//    splitString(vlines[i], ",", vvalues);
//    int numValues = (int)vvalues.size();
//
//    vlineValue[i].resize(numValues);
//    for (int j = 0; j < numValues; ++j)
//      vlineValue[i][j] = atof(vvalues[j].c_str());
//  }
//
//  int numCols = (int)vlineValue[0].size();
//  mV.resize(numRows, numCols);
//  for (int i = 0; i < numRows; ++i)
//    for (int j = 0; j < numCols; ++j)
//      mV(i, j) = vlineValue[i][j];
//
//  return true;
//}

bool writeMatrix_CSV(const string& path, const MatrixXd& mV) {
  logFile(path, matrixToString_CSV(mV), false);

  return true;
}

void logTrace(Verbosity verbosity, const char* format, ...) {
  if (Logger::Instance().verbosity == Verbosity::V0_Silence ||
      Logger::Instance().verbosity < verbosity)
    return;  // Not logging this particular message, not enough verbosity value
             // or silenced logger

#if WIN32
  static char msg[100000];
  va_list vl;

  va_start(vl, format);
  vsprintf_s(msg, format, vl);
  va_end(vl);

  for (int i = 0; i < (int)Logger::Instance().vLogTraceStreamRef.size(); ++i) {
    (*Logger::Instance().vLogTraceStreamRef[i]) << msg;
    Logger::Instance().vLogTraceStreamRef[i]->flush();
  }
#else
  static char msg[1024];
  va_list vl;

  va_start(vl, format);
  vsnprintf(msg, 1024, format, vl);
  va_end(vl);

  for (int i = 0; i < (int)Logger::Instance().vLogTraceStreamRef.size(); ++i) {
    (*Logger::Instance().vLogTraceStreamRef[i]) << msg;
    Logger::Instance().vLogTraceStreamRef[i]->flush();
  }
#endif
}

void logTimes(Verbosity verbosity, const char* format, ...) {
  if (Logger::Instance().verbosity == Verbosity::V0_Silence ||
      Logger::Instance().verbosity < verbosity)
    return;  // Not logging this particular message, not enough verbosity value
             // or silenced logger

#if WIN32
  static char msg[1024];
  va_list vl;

  va_start(vl, format);
  vsprintf_s(msg, format, vl);
  va_end(vl);

  for (int i = 0; i < (int)Logger::Instance().vLogTimesStreamRef.size(); ++i) {
    (*Logger::Instance().vLogTimesStreamRef[i]) << msg;
    Logger::Instance().vLogTimesStreamRef[i]->flush();
  }
#else
  static char msg[1024];
  va_list vl;

  va_start(vl, format);
  vsnprintf(msg, 1024, format, vl);
  va_end(vl);

  for (int i = 0; i < (int)Logger::Instance().vLogTimesStreamRef.size(); ++i) {
    (*Logger::Instance().vLogTimesStreamRef[i]) << msg;
    Logger::Instance().vLogTimesStreamRef[i]->flush();
  }
#endif
}

}  // namespace IOUtils
}  // namespace PhySim
