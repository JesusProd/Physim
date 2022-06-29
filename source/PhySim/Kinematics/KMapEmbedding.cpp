//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Embedding.h>
#include <PhySim/Kinematics/KMapEmbedding.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

KMapEmbedding::KMapEmbedding(Simulable* pModel,
                             PtrS<KEleParticle3D>& pParticle,
                             PtrS<Embedding>& pEmbedding) {
  this->Init(pModel, pParticle, pEmbedding);
}

void KMapEmbedding::Init(Simulable* pModel,
                         PtrS<KEleParticle3D>& pParticle,
                         PtrS<Embedding>& pEmbedding) {
  this->m_pEmbedding = pEmbedding;

  int N = pEmbedding->Master()->NumNodes();
  vector<PtrS<KinematicsEle>> vOut(1);
  vector<PtrS<KinematicsEle>> vIn(N);
  for (int i = 0; i < N; ++i)
    vIn[i].reset(
        pEmbedding->Master()->Nodes()[i]->Traits().Kinematics(Tag_DOF_0));
  vOut[0] = pParticle;

  KinematicsMap::Init(pModel, vIn, vOut);

  this->m_isLinear = false;
}

void KMapEmbedding::MapValue(int idxIn,
                             const VectorXd& vpIn,
                             int idxOut,
                             VectorXd& vpOut) {
  throw PhySim::exception("Not implemented");
}

void KMapEmbedding::UpdateMapPartials() {
  throw PhySim::exception("Not implemented");
}

}  // namespace PhySim
