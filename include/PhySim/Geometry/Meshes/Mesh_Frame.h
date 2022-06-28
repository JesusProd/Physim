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


#include <PhySim/Geometry/Meshes/Mesh_Edge.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Mesh_Frame : public Mesh_Edge
	{
	public:
		Mesh_Frame();
		Mesh_Frame(const Mesh_Frame& toCopy);
		Mesh_Frame(const MatrixXd& mV, const MatrixXi& mE, const vector<Frame3d>& vF = vector<Frame3d>(), const vector<Tag>& vnodeTraits = vector<Tag>(), const vector<Tag>& vframeTraits = vector<Tag>());
		void Init(const MatrixXd& mV, const MatrixXi& mE, const vector<Frame3d>& vF = vector<Frame3d>(), const vector<Tag>& vnodeTraits = vector<Tag>(), const vector<Tag>& vframeTraits = vector<Tag>());
		virtual ~Mesh_Frame(void);

		void Merge(PtrS<Mesh_Frame>& pMesh, const vector<PtrS<Mesh_Frame>>& vmeshes, Tag trait, Real tol);

		virtual Mesh* Clone() const override { return new Mesh_Frame(*this); }

		virtual void GetFrameVector(vector<Frame3d>& vF, Tag s) const;
		virtual void SetFrameVector(const vector<Frame3d>& vF, Tag s);

		virtual Frame3d& GetFrame(int i, Tag s) { return this->GetEdge(i)->Traits().Frame3d(s); }

		virtual void GetRod(Edge* pEdge, vector<Edge*>& vrod) const;
		virtual void ParallalelTransportForward(Tag s, const Frame3d& F, vector<Edge*> vrod);
		virtual void ParallalelTransportBackward(Tag s, const Frame3d& F, vector<Edge*> vrod);

		virtual void SetRefFrames_TwistFreeRods(Tag s);
		virtual void SetRefFrames_FromConnectionNormals(Tag s, const vector<Vector3d>& vn, bool twistFree);
		virtual void SetRefFrames_FromRotatedRefFrames(Tag sourceTrait, Tag targetTrait, bool twistFree);

	private:
		void UpdateMetadata();
		void FreeMetadata();


	};

}
