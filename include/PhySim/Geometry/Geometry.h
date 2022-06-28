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

#include <PhySim/Traits.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Poly;
	class Node;
	class Edge;
	class Face;
	class Cell;
	class Mesh;
	class Embedding;

	class Geometry
	{
	protected:
		TraitSet m_traits;
		vector<Poly*> m_velems;
		vector<Node*> m_vnodes;
		vector<Edge*> m_vedges;
		vector<Face*> m_vfaces;
		vector<Cell*> m_vcells;
		int m_ID;

	public:
		Geometry() : m_ID(-1) { };
		virtual ~Geometry() { };

		// Identifier

		virtual int& ID() { return this->m_ID; };
		virtual const int& ID() const { return this->m_ID; };

		// Topology

		virtual int NumNodes() const { return (int)this->m_vnodes.size(); }
		virtual int NumElems() const { return (int)this->m_velems.size(); }
		virtual int NumEdges() const { return (int)this->m_vedges.size(); }
		virtual int NumFaces() const { return (int)this->m_vfaces.size(); }
		virtual int NumCells() const { return (int)this->m_vcells.size(); }

		virtual vector<Poly*>& Elems() { return this->m_velems; }
		virtual vector<Node*>& Nodes() { return this->m_vnodes; }
		virtual vector<Edge*>& Edges() { return this->m_vedges; }
		virtual vector<Face*>& Faces() { return this->m_vfaces; }
		virtual vector<Cell*>& Cells() { return this->m_vcells; }

		virtual const vector<Poly*>& Elems() const { return this->m_velems; }
		virtual const vector<Node*>& Nodes() const { return this->m_vnodes; }
		virtual const vector<Edge*>& Edges() const { return this->m_vedges; }
		virtual const vector<Face*>& Faces() const { return this->m_vfaces; }
		virtual const vector<Cell*>& Cells() const { return this->m_vcells; }

		// Object

		void GetNodesTrait(vector<TraitSet>& vtraits) const;
		void SetNodesTrait(const vector<TraitSet>& vtraits);

		void GetElemsTrait(vector<TraitSet>& vtraits) const;
		void SetElemsTrait(const vector<TraitSet>& vtraits);

		template <class T> void GetNodesTrait(vector<T>& vvalues, Tag s) const;
		template <class T> void SetNodesTrait(const vector<T>& vvalues, Tag s);

		template <class T> void GetElemsTrait(vector<T>& vvalues, Tag s) const;
		template <class T> void SetElemsTrait(const vector<T>& vvalues, Tag s);

		virtual void GetElemsTrait(MatrixXd& mN, Tag s = Tag_Position_X) const;
		virtual void SetElemsTrait(const MatrixXd& mN, Tag s = Tag_Position_X);
		virtual void DelElemsTrait(Tag s = Tag_Position_X);

		virtual void GetNodesTrait(MatrixXd& mN, Tag s = Tag_Position_X) const;
		virtual void SetNodesTrait(const MatrixXd& mN, Tag s = Tag_Position_X);
		virtual void DelNodesTrait(Tag s = Tag_Position_X);

		//  Transforms

		virtual void Scale(const VectorXd& vs, Tag s = Tag_Position_X);
		virtual void Translate(const VectorXd& vt, Tag s = Tag_Position_X);
		virtual void RotateAxis(const VectorXd& vaxi, Tag s = Tag_Position_X);
		virtual void RotateEuler(const VectorXd& veul, Tag s = Tag_Position_X);
		virtual void Transform(const MatrixXd& mT, Tag s = Tag_Position_X);

		// Data

		inline TraitSet& Traits() { return this->m_traits; }
		inline const TraitSet& Traits() const { return this->m_traits; }
		template <class T> inline T& Trait(Tag s) { return this->m_traits.Trait<T>(s); }
		template <class T> inline const T& Trait(Tag s) const { return this->m_traits.Trait<T>(s); }

		virtual PtrS<Embedding> ComputeProjection(const Vector3d& vx, Tag s = Tag_Position_X);
		virtual PtrS<Embedding> ComputeEmbedding(const Vector3d& vx, Tag s = Tag_Position_X);

		virtual void ComputeProjection(const MatrixXd& vpoints, vector<PtrS<Embedding>>& vembed, Tag s = Tag_Position_X);
		virtual void ComputeEmbedding(const MatrixXd& vpoints, vector<PtrS<Embedding>>& vembed, Tag s = Tag_Position_X);

		virtual void ComputeEmbedding(vector<PtrS<Embedding>>& vembed, const VectorXi& vsel = VectorXi());
		virtual void EmbedNodes(vector<Node*>& toEmbed, const Tag& trait = Tag_Position_X);
		virtual void EmbedNodes(vector<Node*>& toEmbed, const VectorXi& vsel = VectorXi());
		virtual void EmbedMesh(Mesh& mesh, const Tag& trait = Tag_Position_X);
		virtual void EmbedMesh(Mesh& mesh, const VectorXi& vsel = VectorXi());

		virtual void InterpolateEmbeddedNodeTraits(Tag s0);
		virtual void InterpolateEmbeddedDeformation(Tag s0, Tag sX);

		// Geometry specific

		virtual int DimSpace() const = 0;
		virtual int DimBasis() const = 0;

		virtual Real VolumeBasis(Tag s = Tag_Position_X) const = 0;
		virtual Real VolumeSpace(Tag s = Tag_Position_X) const = 0;
		virtual Vector3d Centroid(Tag s = Tag_Position_X) const = 0;
		virtual Matrix3d Rotation(Tag f, Tag t) const = 0;

		virtual void MassProperties(Tag s, Real rho, Real& mass, Vector3d& vcom, Matrix3d& mI) const = 0;
	};
}