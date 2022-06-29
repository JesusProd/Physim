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

namespace PhySim {
using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////////////

enum struct Discretization {
  Nodes,
  Edge2,  // Linear edge
  Edge3,  // Quadratic edge
  Tri3,   // Linear triangle
  Tri6,   // Quadratic triangle
  Quad4,  // Bilinear quadrangle
  Quad8,  // Quadratic quadrangle
  Tet4,   // Linear tetrahedron
  Tet10,  // Quadratic tetrahedron
  Hex8,   // Trilinear hexahedron
  Hex20   // Triquadratic hexahedron
};

///////////////////////////////////////////////////////////////////////////////////////

enum struct LSSolverType {
  EigenLU,
  EigenCG,
  EigenLDLT,
  BiCGSTAB,
  CholmodLDLT,
  SSparseSPQR,
  CUDALU,
  CUDAQR,
  CUDASC
};

enum struct QPSolverType {
  Steepest,
  Newton,
  LBFGS,
  Gauss,
  BFGS_D,
  BFGS_I,
};

enum struct LSearchType { None, Simple, Armijo, WolfeWeak, WolfeStrong };

enum struct StepSelType { LineSearch, TrustRegion };

enum struct LSResult {
  SUCCESS,   // Generic success
  FAILURE,   // Generic failure
  SINGULAR,  // Singular matrix (rank deficient)
  NONSPD,    // Non symmetric positive definite
  MAXITER,   // Maximum iterations reached
};

enum struct OSResult {
  ONGOING,  // Not finished yet
  SUCCESS,  // Generic success
  MINSTEP,  // Minimum step reached
  MINIMPR,  // Minimum improvement
  FAILURE,  // Generic failure
  MAXITER,  // Maximum iterations reached
  NONDESC   // Non-descendent step found
};

///////////////////////////////////////////////////////////////////////////////////////

enum struct DirtyFlags : int {
  None = 0,
  Energy = 1,
  Gradient = 2,
  Hessian = 4,
  Mass = 8,
  Rest = 16,
  Fixed = 32,
  DuDp = 64,
  DgDp = 128,
  DmDp = 256,
  DxDp = 512,
  Constraint = 1024,
  Jacobian = 2048,
  Kinematics = 4096,
  Mechanics = 8192,
  DMvDt = 16384,
  All = 65535
};

inline DirtyFlags operator|(DirtyFlags a, DirtyFlags b) {
  return static_cast<DirtyFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline DirtyFlags operator&(DirtyFlags a, DirtyFlags b) {
  return static_cast<DirtyFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline DirtyFlags operator~(DirtyFlags a) {
  return static_cast<DirtyFlags>(~static_cast<int>(a));
}

///////////////////////////////////////////////////////////////////////////////////////

// Data semantics

enum struct Tag {
  // DoF

  DOF_0,
  DOF_1,
  DOF_2,

  // General DoF traits

  Position_X,
  Position_0,
  Velocity,
  Mass,
  Force,

  // Deformation traits

  DxD0_X,
  DxD0_V,

  // Specific trait kinematics

  Quat_0,
  Quat_X,
  Quat_V,  // Quaternion
  Rotat_0,
  Rotat_X,
  Rotat_V,  // Rotations
  Euler_0,
  Euler_X,
  Euler_V,  // Euler angles
  Angle_0,
  Angle_X,
  Angle_V,  // Angle
  Frame_0,
  Frame_X,
  Frame_V,  // Frame3d
  Twist_0,
  Twist_X,
  Twist_V,  // Twist

  // Even more specific trait kinematics

  Normal_0,
  Normal_X,
  Normal_V,  // Normal
  Tangent_0,
  Tangent_X,
  Tangent_V,  // Tagent
  Binormal_0,
  Binormal_X,
  Binormal_V,  // Binormal

  // Mesh related

  Embedding_0,
  Embedding_1,
  Embedding_2,
  GridMeta,
  FineMeta,
  CoarMeta,
  CoarCoordEle,

  // Dimensions (radii, etc.)

  Size_0,
  Size_1,
  Size_2,

  // Signs (directions, etc.)

  Sign_0,
  Sign_1,
  Sign_2,

  // Material traits

  Mat_Model,
  Mat_Young,
  Mat_Poisson,
  Mat_Lame1,
  Mat_Lame2,
  Mat_Bulk,
  Mat_Mooney01,
  Mat_Mooney10,
  Mat_Radius1,
  Mat_Radius0,
  Mat_Thickness,
  Mat_StretchK,
  Mat_BendingK,
  Mat_ShearK,
  Mat_Density,
  Mat_0,
  Mat_1,
  Mat_2,

  // Mesh types

  SurfaceMesh_0,
  SurfaceMesh_1,
  SurfaceMesh_2,
  VisualMesh_0,
  VisualMesh_1,
  VisualMesh_2,
  ContactMesh_0,
  ContactMesh_1,
  ContactMesh_2,
  CoarseMesh_0,
  CoarseMesh_1,
  CoarseMesh_3,
  FineMesh_0,
  FineMesh_1,
  FineMesh_2,

  // Mesh visualization?

  Color_0,
  Color_1,
  Color_2,
  Label_0,
  Label_1,
  Label_2,
  TexUV_0,
  TexUV_1,
  TexUV_2,
  Alpha_0,
  Alpha_1,
  Alpha_2,

  // Whole models

  Model_0,
  Model_1,
  Model_2,

  // Custom semantics

  Custom_0,
  Custom_1,
  Custom_2,
  Custom_3,
  Custom_4,
  Custom_5,
  Custom_6,
  Custom_7,
  Custom_8,

  // Tag ADMM subsystem and CHen
  Tag_ADMMSubsystem,
  Tag_Weights,
  Tag_SubMesh
};

struct exception {
  string msg;

 public:
  exception(const string& msg) { this->msg = msg; }

  const string& Message() { return msg; }
};

///////////////////////////////////////////////////////////////////////////////////////

enum struct Verbosity {
  V0_Silence,
  V1_Default,
  V2_SoftDebug,
  V3_HardDebug,
  V4_DeepShit
};

class Logger {
 private:
  static PtrS<Logger> INSTANCE;

 public:
  static Logger& Instance() {
    if (INSTANCE == NULL) {
      INSTANCE.reset(new Logger());
    }
    return *INSTANCE;
  }

  vector<PtrS<ostream>> vLogTraceStreamRef;
  vector<PtrS<ostream>> vLogTimesStreamRef;
  Verbosity verbosity;

  void ClearStreams() {
    vLogTraceStreamRef.clear();
    vLogTimesStreamRef.clear();
  }

  void AddTraceStream(PtrS<ostream>& pStream) {
    vLogTraceStreamRef.push_back(pStream);
  }

  void AddTimesStream(PtrS<ostream>& pStream) {
    vLogTimesStreamRef.push_back(pStream);
  }

  void RemoveTraceStream(PtrS<ostream>& pStream) {
    vector<PtrS<ostream>>::iterator it =
        find(this->vLogTraceStreamRef.begin(), this->vLogTraceStreamRef.end(),
             pStream);
    if (it != this->vLogTraceStreamRef.end())
      this->vLogTraceStreamRef.erase(it);
  }

  void RemoveTimesStream(PtrS<ostream>& pStream) {
    vector<PtrS<ostream>>::iterator it =
        find(this->vLogTimesStreamRef.begin(), this->vLogTimesStreamRef.end(),
             pStream);
    if (it != this->vLogTimesStreamRef.end())
      this->vLogTimesStreamRef.erase(it);
  }

 private:
  Logger() {
    this->verbosity = Verbosity::V1_Default;
    vLogTraceStreamRef.push_back(PtrS<ostream>(
        new fstream("logTrace.txt", fstream::out | fstream::trunc)));
    vLogTimesStreamRef.push_back(PtrS<ostream>(
        new fstream("logTimes.txt", fstream::out | fstream::trunc)));
  }

 public:
  virtual ~Logger() { this->ClearStreams(); }
};

///////////////////////////////////////////////////////////////////////////////////////

struct Frame3d {
  Frame3d() {
    this->tan.setZero();
    this->nor.setZero();
    this->bin.setZero();
  }

  Frame3d(const Vector3d& vt) {
    Vector3d vr = Vector3d::Random();

    this->tan = vt.normalized();
    this->bin = tan.cross(vr).normalized();
    this->nor = bin.cross(tan).normalized();
  }

  Frame3d(const Frame3d& toCopy) {
    this->tan = toCopy.tan;
    this->nor = toCopy.nor;
    this->bin = toCopy.bin;
  }

  ~Frame3d() {
    // Nothing here...
  }

  bool operator==(const Frame3d& other) const {
    return (tan - other.tan).squaredNorm() < 1e-9 &&
           (nor - other.nor).squaredNorm() < 1e-9 &&
           (bin - other.bin).squaredNorm() < 1e-9;
  }

  bool operator!=(const Frame3d& other) const { return !(*this == other); }

  Frame3d operator*(const Matrix3d& T) const {
    Frame3d Ft = (*this);
    Ft.tan = T * Ft.tan;
    Ft.bin = T * Ft.bin;
    Ft.nor = T * Ft.nor;
    return Ft;
  }

  Frame3d reverse() const {
    Frame3d out = *this;
    out.tan = -out.tan;
    out.bin = -out.bin;
    return out;
  }

  Vector3d tan;
  Vector3d nor;
  Vector3d bin;
};

///////////////////////////////////////////////////////////////////////////////////////

struct LT_Triplet {
  bool operator()(const Triplet<double>& a, const Triplet<double>& b) {
    return (a.row() < b.row()) || (a.row() == b.row() && a.col() < b.col());
  }
};

class AMatrixSd : public MatrixSd {
 public:
  struct Block {
    int m_row;
    int m_col;
    int m_N;
    int m_M;
    bool m_transpose;
    MatrixXp m_mPointers;
    AMatrixSd* m_pMatrix;

    void Add(const MatrixXd& mB) {
      for (int i = 0; i < this->m_N; ++i)
        for (int j = 0; j < this->m_M; ++j)
          *(this->m_mPointers(i, j)) += mB(i, j);
    }

    void Rem(const MatrixXd& mB) {
      for (int i = 0; i < this->m_N; ++i)
        for (int j = 0; j < this->m_M; ++j)
          *(this->m_mPointers(i, j)) -= mB(i, j);
    }
  };

  virtual inline VectorTd& TripletsDynamic() {
    return this->m_vvalueTripletsStatic;
  }
  virtual inline VectorTd& TripletsStatic() {
    return this->m_vvalueTripletsStatic;
  }
  virtual inline CoefMap& Mapping() { return this->m_coeffMap; }

  AMatrixSd(int numRows = 0,
            int numCols = 0,
            int assembler = -1,
            bool isFast = true) {
    m_coeffMap.clear();
    m_vvalueTripletsStatic.clear();
    m_vvalueTripletsDynamic.clear();

    this->m_isFast = isFast;

    this->resize(numRows, numCols);
    this->m_layer = assembler;
  }

  bool AddStaticBlock(PtrS<AMatrixSd::Block> pBlock, const MatrixXd& mBlock);
  bool AddStaticBlock(int row, int col, const MatrixXd& mBlock);
  bool AddDynamicBlock(int row, int col, const MatrixXd& mBlock);

  PtrS<Block> CreateBlock(int row, int col, int N, int M);

  void StartAssembly();

  void EndAssembly();

  void Zero();

  void Clear();

  auto GetNumNZStatic() const {
    return this->m_vvalueTripletsStatic.capacity();
  }
  auto GetNumNZDynamic() const {
    return this->m_vvalueTripletsDynamic.capacity();
  }
  void SetNumNZStatic(int numNZ) {
    this->m_vvalueTripletsStatic.reserve(numNZ);
  }
  void SetNumNZDynamic(int numNZ) {
    this->m_vvalueTripletsDynamic.reserve(numNZ);
  }

  inline bool IsInit() { return this->rows() != 0 && this->cols() != 0; }

  inline bool IsFast() { return this->m_isFast && !this->m_coeffMap.empty(); }

  inline int& Layer() { return this->m_layer; }

 private:
  CoefMap m_coeffMap;

  VectorTd m_vvalueTripletsStatic;
  VectorTd m_vvalueTripletsDynamic;

  bool m_isFast;

  int m_layer;
};

///////////////////////////////////////////////////////////////////////////////////////

class AVectorXd : public VectorXd {
 public:
  AVectorXd(int numRows = 0, int assembler = -1) : VectorXd(numRows) {
    this->m_layer = assembler;
  }

  void SetBlock(int offset, VectorXd& vBlock) {
    this->segment(offset, vBlock.size()) = vBlock;
  }

  void AddBlock(int offset, VectorXd& vBlock) {
    this->segment(offset, vBlock.size()) += vBlock;
  }

  void SubBlock(int offset, VectorXd& vBlock) {
    this->segment(offset, vBlock.size()) -= vBlock;
  }

  void RemBlock(int offset, int size) {
    this->segment(offset, size) = VectorXd::Zero(size);
  }

  inline int& Layer() { return m_layer; }

 private:
  int m_layer;
};

///////////////////////////////////////////////////////////////////////////////////////

struct XorHasher {
 public:
  template <class T1, class T2>
  size_t operator()(const pair<T1, T2>& p) const {
    auto hash1 = hash<T1>{}(p.first);
    auto hash2 = hash<T2>{}(p.second);
    return hash1 ^ hash2;
  }

  template <typename... T>
  size_t operator()(const tuple<T...>& t) const {
    return TupleIterator<T...>::Element<0>::Get(t);
  }

 private:
  template <typename... T>
  struct TupleIterator {
    template <int N>
    using Type = typename tuple_element<N, tuple<T...>>::type;

    template <int N>
    struct Element {
      static size_t Get(const tuple<T...>& t) {
        const size_t h = hash<Type<N>>{}(get<N>(t));
        return h ^ Element<N + 1>::Get(t);
      }
    };

    template <>
    struct Element<sizeof...(T)> {
      static size_t Get(const tuple<T...>& t) { return 0; }
    };
  };
};

///////////////////////////////////////////////////////////////////////////////////////

template <typename T, int N>
// Class encapsulating an axis aligned bounding box
class AABB {
 public:
  AABB() {
    m_minCor.setZero();
    m_maxCor.setZero();
  }

  AABB(const Vector<T, N>& minCor, const Vector<T, N>& maxCor)
      : m_minCor(minCor), m_maxCor(maxCor) {
    // Nothing to do
  }

  Vector<T, N> m_minCor;
  Vector<T, N> m_maxCor;
};

}  // namespace PhySim