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
  Discretization_Nodes,
  Discretization_Edge2,  // Linear edge
  Discretization_Edge3,  // Quadratic edge
  Discretization_Tri3,   // Linear triangle
  Discretization_Tri6,   // Quadratic triangle
  Discretization_Quad4,  // Bilinear quadrangle
  Discretization_Quad8,  // Quadratic quadrangle
  Discretization_Tet4,   // Linear tetrahedron
  Discretization_Tet10,  // Quadratic tetrahedron
  Discretization_Hex8,   // Trilinear hexahedron
  Discretization_Hex20   // Triquadratic hexahedron
};

///////////////////////////////////////////////////////////////////////////////////////

enum struct LSSolverType {
  LS_EigenLU,
  LS_EigenCG,
  LS_EigenLDLT,
  LS_BiCGSTAB,
  LS_CholmodLDLT,
  LS_SSparseSPQR,
  LS_CUDALU,
  LS_CUDAQR,
  LS_CUDASC
};

enum struct QPSolverType {
  QP_Steepest,
  QP_Newton,
  QP_LBFGS,
  QP_Gauss,
  QP_BFGS_D,
  QP_BFGS_I,
};

enum struct LSearchType {
  LSearch_None,
  LSearch_Simple,
  LSearch_Armijo,
  LSearch_WolfeWeak,
  LSearch_WolfeStrong
};

enum struct StepSelType { SS_LineSearch, SS_TrustRegion };

enum struct LSResult {
  LS_SUCCESS,   // Generic success
  LS_FAILURE,   // Generic failure
  LS_SINGULAR,  // Singular matrix (rank deficient)
  LS_NONSPD,    // Non symmetric positive definite
  LS_MAXITER,   // Maximum iterations reached
};

enum struct OSResult {
  OR_ONGOING,  // Not finished yet
  OR_SUCCESS,  // Generic success
  OR_MINSTEP,  // Minimum step reached
  OR_MINIMPR,  // Minimum improvement
  OR_FAILURE,  // Generic failure
  OR_MAXITER,  // Maximum iterations reached
  OR_NONDESC   // Non-descendent step found
};

///////////////////////////////////////////////////////////////////////////////////////

enum struct DirtyFlags : int {
  Dirty_None = 0,
  Dirty_Energy = 1,
  Dirty_Gradient = 2,
  Dirty_Hessian = 4,
  Dirty_Mass = 8,
  Dirty_Rest = 16,
  Dirty_Fixed = 32,
  Dirty_DuDp = 64,
  Dirty_DgDp = 128,
  Dirty_DmDp = 256,
  Dirty_DxDp = 512,
  Dirty_Constraint = 1024,
  Dirty_Jacobian = 2048,
  Dirty_Kinematics = 4096,
  Dirty_Mechanics = 8192,
  Dirty_DMvDt = 16384,
  Dirty_All = 65535
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

  Tag_DOF_0,
  Tag_DOF_1,
  Tag_DOF_2,

  // General DoF traits

  Tag_Position_X,
  Tag_Position_0,
  Tag_Velocity,
  Tag_Mass,
  Tag_Force,

  // Deformation traits

  Tag_DxD0_X,
  Tag_DxD0_V,

  // Specific trait kinematics

  Tag_Quat_0,
  Tag_Quat_X,
  Tag_Quat_V,  // Quaternion
  Tag_Rotat_0,
  Tag_Rotat_X,
  Tag_Rotat_V,  // Rotations
  Tag_Euler_0,
  Tag_Euler_X,
  Tag_Euler_V,  // Euler angles
  Tag_Angle_0,
  Tag_Angle_X,
  Tag_Angle_V,  // Angle
  Tag_Frame_0,
  Tag_Frame_X,
  Tag_Frame_V,  // Frame3d
  Tag_Twist_0,
  Tag_Twist_X,
  Tag_Twist_V,  // Twist

  // Even more specific trait kinematics

  Tag_Normal_0,
  Tag_Normal_X,
  Tag_Normal_V,  // Normal
  Tag_Tangent_0,
  Tag_Tangent_X,
  Tag_Tangent_V,  // Tagent
  Tag_Binormal_0,
  Tag_Binormal_X,
  Tag_Binormal_V,  // Binormal

  // Mesh related

  Tag_Embedding_0,
  Tag_Embedding_1,
  Tag_Embedding_2,
  Tag_GridMeta,
  Tag_FineMeta,
  Tag_CoarMeta,
  Tag_CoarCoordEle,

  // Dimensions (radii, etc.)

  Tag_Size_0,
  Tag_Size_1,
  Tag_Size_2,

  // Signs (directions, etc.)

  Tag_Sign_0,
  Tag_Sign_1,
  Tag_Sign_2,

  // Material traits

  Tag_Mat_Model,
  Tag_Mat_Young,
  Tag_Mat_Poisson,
  Tag_Mat_Lame1,
  Tag_Mat_Lame2,
  Tag_Mat_Bulk,
  Tag_Mat_Mooney01,
  Tag_Mat_Mooney10,
  Tag_Mat_Radius1,
  Tag_Mat_Radius0,
  Tag_Mat_Thickness,
  Tag_Mat_StretchK,
  Tag_Mat_BendingK,
  Tag_Mat_ShearK,
  Tag_Mat_Density,
  Tag_Mat_0,
  Tag_Mat_1,
  Tag_Mat_2,

  // Mesh types

  Tag_SurfaceMesh_0,
  Tag_SurfaceMesh_1,
  Tag_SurfaceMesh_2,
  Tag_VisualMesh_0,
  Tag_VisualMesh_1,
  Tag_VisualMesh_2,
  Tag_ContactMesh_0,
  Tag_ContactMesh_1,
  Tag_ContactMesh_2,
  Tag_CoarseMesh_0,
  Tag_CoarseMesh_1,
  Tag_CoarseMesh_3,
  Tag_FineMesh_0,
  Tag_FineMesh_1,
  Tag_FineMesh_2,

  // Mesh visualization?

  Tag_Color_0,
  Tag_Color_1,
  Tag_Color_2,
  Tag_Label_0,
  Tag_Label_1,
  Tag_Label_2,
  Tag_TexUV_0,
  Tag_TexUV_1,
  Tag_TexUV_2,
  Tag_Alpha_0,
  Tag_Alpha_1,
  Tag_Alpha_2,

  // Whole models

  Tag_Model_0,
  Tag_Model_1,
  Tag_Model_2,

  // Custom semantics

  Tag_Custom_0,
  Tag_Custom_1,
  Tag_Custom_2,
  Tag_Custom_3,
  Tag_Custom_4,
  Tag_Custom_5,
  Tag_Custom_6,
  Tag_Custom_7,
  Tag_Custom_8,

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